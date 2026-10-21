# Packet Rate Guide

VeriCSI-ESP32 introduces a deterministic AP-side packet generation path on top of the original Hernandez 2020 STA-AP architecture. This guide explains when to use it, how to configure it, and how it differs from the espressif/esp-csi `csi_recv` path.

## TL;DR

VeriCSI-ESP32 adds a FreeRTOS general-purpose timer (`gptimer`) on the AP side that drives `esp_wifi_80211_tx` at a period you set in microseconds. Enable it under `VeriCSI-ESP32 Config > Application-layer broadcast (AP-side timer)` in `idf.py menuconfig` on the AP node.

**What this is and is not for.** It gives you a *known, controlled* sounding cadence — the rate the device attempts is the number you configured, independent of an external router, of STA polling, and of ESP-NOW retry behaviour. That is what makes an acquisition-timing experiment possible at all.

It is **not** justified by "lower jitter improves frequency-domain sensing accuracy". That claim was tested and does not hold: the first-order sensitivity of a frequency estimate to timing jitter scales as `2π·f·ε`, so respiration at 0.3 Hz with 10 ms of jitter sees 0.019 rad ≈ 1.1°, and heart rate at 2 Hz sees 7.2°. Low-frequency signals barely move within a jitter interval. See `docs/retired_hypotheses.md` R1.

The effects that *do* matter for frequency-domain sensing are the **mean rate deficit** (a pipeline assuming the nominal rate stretches its own frequency axis) and **periodic MAC structure** (beacons at 100 TU = 9.765625 Hz can alias into the vital-sign band). Both are about which instants you actually got, not about how tightly spaced they were.

## Three packet generation paths

| Path | Where the rate is decided | Typical jitter at 100 Hz | Notes |
|---|---|---|---|
| Hernandez 2020 STA-poll (default v1) | STA-side TCP / HTTP polling rate | tens of milliseconds | Round-trip and STA scheduler dominate; usable for slow gestures, weak for breathing / HR |
| esp-csi `csi_recv_router` | external Wi-Fi router's Ping reply | unspecified, depends on router | Couples to whatever router is in the room |
| esp-csi `csi_recv` (ESP-NOW) | Wi-Fi MAC + ESP-NOW retry | tighter than HTTP, but ESP-NOW retransmits introduce bursts | Designed for ESP-NOW deployments, not frequency-domain sensing |
| **gptimer broadcast (this guide)** | **FreeRTOS gptimer on the AP** | **tens to hundreds of microseconds at 100/200 Hz** | Bypasses STA-poll, ESP-NOW retry and external router; the rate the device tries to hit is the rate you wrote in menuconfig |

The third row's "tens to hundreds of microseconds" figure is the inter-packet jitter at the application layer (between successive `esp_wifi_80211_tx` invocations). The PHY/MAC layer's CSMA/CCA still imposes its own ceiling once the medium is busy - see "Limits" below.

## Configuration

In the AP project's `idf.py menuconfig`:

```
ESP32 CSI Tool Config
└── Application-layer broadcast (AP-side timer)
    ├── [*] Drive packet generation with a FreeRTOS gptimer
    ├── [1] PERIOD in microseconds (preferred)   e.g. 10000 = 100 Hz
    └── [2] RATE in whole Hz (legacy, ignored when [1] is non-zero)
```

Equivalent sdkconfig flags (set automatically by menuconfig):

```
CONFIG_ESP32_CSI_TOOL_AP_MODE_GPTIMER_BROADCAST=y
CONFIG_ESP32_CSI_TOOL_BROADCAST_PERIOD_US=10000
```

When this Kconfig is enabled the firmware emits `# CSI_BROADCAST_PERIOD_US=N` and `# CSI_BROADCAST_RATE_HZ=N` lines into the boot-comment header of every CSV (alongside `# CHIP=`, `# IDF_VERSION=`, ...). Read it from a capture file with `parse_csi.parse_metadata(path)["CSI_BROADCAST_RATE_HZ"]`. `experiments/e5_packet_rate/packet_rate_analyzer.py` picks it up automatically when `--target-hz` is omitted; supply `--target-hz` only for RX-side captures (`active_sta` / `passive`) where the comment is absent (those firmwares don't run the gptimer).

For the empirical test procedure, see `experiments/w1_instrument_check/README.md`. Note the receiver role there is `active_sta`, not `passive`.

Recommended target rates:

| Task | Recommended rate | Rationale |
|---|---|---|
| Breathing rate (0.1-0.5 Hz) | 100 Hz | Nyquist comfortably; jitter spectrum at 0.1-0.5 Hz is the metric to watch |
| Heart rate (0.8-3 Hz) | 200 Hz | Some headroom for harmonics |
| HAR / motion / gestures | 100 Hz | Hernandez 2020 used ~100-200 Hz |
| Channel sounding / packet jitter benchmark | 500 - 1000 Hz | Stress test; expect CSMA/CCA-limited bursts |

## Implementation summary

The implementation lives in `_components/gptimer_broadcast.h` and is enabled by Kconfig from `active_ap/main/main.cc`. Two pieces:

1. **gptimer ISR** (`_gptimer_alarm_cb`). Runs from interrupt context. The ISR is the only path that the timer fires into; it MUST NOT call any Wi-Fi API (those acquire mutexes that will deadlock). The ISR therefore only does `xTaskNotifyFromISR(s_bcast_tx_task, 0, eIncrement, ...)`.

2. **TX task** (`_gptimer_broadcast_tx_task`, priority 6, pinned to core 0 on dual-core). Blocks on `xTaskNotifyWait`, then transmits one frame per pending notification. The 64-byte frame is a 24-byte 802.11 Data header (`FC=0x08 0x02`, broadcast destination, AP MAC as Addr2/Addr3) followed by a 40-byte payload that carries a `CSI2` magic, the broadcast sequence number and the low 32 bits of `esp_timer_get_time()` at TX. The Wi-Fi driver assigns the 802.11 sequence control field (`en_sys_seq=true`).

The same broadcast sequence is fed back into the CSI consumer task via `csi_set_broadcast_seq()` and appears as `gptimer_seq` in column 30 of the v2 CSV (see `docs/csv_format_v2.md`).

> **`gptimer_seq` is transmit-side only.** It is written from the AP's own TX task into the AP's own firmware image. `active_sta` and `passive` builds never link `gptimer_broadcast.h` and never call the setter, so `gptimer_seq` is identically `0` in every STA-side and passive-side capture. Nothing decodes the sequence number embedded in the broadcast payload on the receive side.
>
> **Do not design receiver-side loss detection around `gptimer_seq`.** Use `rx_seq` (column 36), which the firmware reads from `wifi_csi_info_t.rx_seq` — the 802.11 MAC Sequence Control of the received frame. It is present on every received frame on every SoC, and is the only field that lets a receiver detect its own frame loss. Sequence numbers are per-transmitter, so count gaps per source MAC.

## Diagnostic counters

`_components/gptimer_broadcast.h` exposes four counters that downstream tooling can inspect over the IDF console:

```c
uint64_t gptimer_broadcast_tx_count(void);     // successful esp_wifi_80211_tx calls
uint64_t gptimer_broadcast_tx_failures(void);  // failures since boot
uint64_t gptimer_broadcast_isr_misses(void);   // xTaskNotifyFromISR returned not-OK
uint32_t gptimer_broadcast_seq(void);          // last sequence number assigned
```

A rough sanity check: at the end of a 60-second run at 100 Hz, `gptimer_broadcast_tx_count + gptimer_broadcast_tx_failures` should be close to 6000 and `tx_count` should dominate. If `tx_failures` is non-trivial, the medium is busy or the destination buffer pool is exhausted; lower the rate or move to a quieter channel.

## Limits

1. **CSMA/CCA shaping above ~500 Hz.** The 802.11 MAC backs off when the medium is busy. Even a perfectly periodic application-layer trigger will see actual transmit times jitter when the channel is contended. The firmware reports both the application-layer schedule (gptimer alarm time) and the actual transmit success count, so you can show this honestly in any analysis.

2. **Target rate range 1-2000 Hz.** Below 1 Hz the gptimer alarm period exceeds 1 s and you should think about why you need a timer at all. Above 2000 Hz the inter-frame margin is thinner than typical Wi-Fi inter-frame spacing and the failure rate climbs sharply.

3. **No retransmission.** This is a Data frame to broadcast; the Wi-Fi MAC does not request acks for broadcast destinations, so dropped frames are not retransmitted. This is by design - retransmissions would themselves be a source of jitter.

4. **STA pair mode only.** The intended deployment is one ESP32 in `active_ap` mode with this broadcast enabled, paired with one or more ESP32 in `active_sta` (or other) mode receiving the broadcast. Three-way deployments work as long as all RX nodes share a SSID and channel with the AP.

5. **Coexists with the legacy STA-poll path.** When this option is disabled (default), `active_ap` falls back to the Hernandez 2020 behaviour and the legacy STA-side polling continues to drive packet rate. There is no need to choose between v1 and v2 firmware - flip the Kconfig switch on the AP and rebuild.

## Reference

This path is the controlled sounding source used by the acquisition-timing study; see `idea-stage/IDEA_REPORT.md`.
