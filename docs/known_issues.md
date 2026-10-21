# Known Issues and Workarounds

This document tracks upstream ESP-IDF / firmware issues that affect CSI quality on the seven supported targets, and the workarounds applied by VeriCSI-ESP32. Each entry will be updated when the upstream status changes.

Last reviewed against ESP-IDF v6.0.1 stable on 2026-08-04; toolkit-side
entries updated 2026-09-15.

**Toolkit defect, fixed 2026-09-15**: the USB-CDC sink's retry path
enqueued bytes without counting them toward write progress, so the main
loop could re-send the same span and duplicate bytes on the wire when the
first enqueue returned 0 and the retry partially succeeded
(`usb_cdc_component.h`). Found while wiring the sink failure contract;
the writer now counts retry progress and reports a dropped tail through
its return code.

## Summary

| Target | Issue | Severity | Workaround |
|---|---|---|---|
| ESP32-C6 | [#14271](https://github.com/espressif/esp-idf/issues/14271) - missing L-LTF + HT-LTF subcarrier reorder in HE mode | Medium | Layout id `C6_HT20_NO_LLTF`, decoder reorder |
| ESP32-C61 | [#18118](https://github.com/espressif/esp-idf/issues/18118) - 802.11g CSI broken | Medium | Disable 11g protocol bit, enable `acquire_csi_force_lltf` |
| ESP32-C5 | [#18493](https://github.com/espressif/esp-idf/issues/18493) - static CSI IQ buffer on 5 GHz | Low | Not reproduced on v6.0.1 (below); 5 GHz opt-in |
| ESP32, ESP32-S2 | `esp_csi_gain_ctrl` 0.1.5 archives contain no code for these targets | Medium | `RX_GAIN_CTRL` stays `n` (default); gain columns read `-1` |

## Beamforming considerations (`acquire_csi_beamformed`)

Wi-Fi 6 routers apply explicit beamforming to most downlink unicast frames. The receiver's CSI estimate from a beamformed packet is `H_eff = H * V`, where `H` is the physical channel response and `V` is the AP-side steering matrix derived from a prior sounding exchange. The firmware exposes `acquire_csi_beamformed` as a Kconfig option (default n) so users can choose between the two interpretations explicitly.

### When to enable `CONFIG_ESP32_CSI_TOOL_ACQUIRE_BEAMFORMED`

- `active_sta` connected to a real 11ax router. Most downlink frames are beamformed; without this enabled the chip drops the majority of HE traffic.
- `passive` sniffing a Wi-Fi 6 network in production use. Same reasoning.
- Reproducing esp-csi's HE-LTF preset behaviour (`console_test` enables beamformed when the user picks HE-LTF / STBC-HE-LTF output).

### When to keep it disabled (the default)

- V7-final K5 anchor experiment: `active_ap` gptimer broadcast to FF:FF:FF:FF:FF:FF. 802.11 broadcast frames are never beamformed by design, so the flag has no effect on K5 captures.
- Any sensing task that depends on **clean physical-channel CSI** (`H`) rather than the post-steering composite `H_eff`:
  - phase unwrap / phase-based breathing rate
  - multi-antenna AoA / MUSIC (ESP32 is single-antenna anyway, but listed for completeness)
  - cross-subcarrier covariance analyses

### How beamformed CSI affects sensing features

| Feature | Impact |
|---|---|
| Absolute amplitude per subcarrier | Re-scaled by `|V_k|`. Per-subcarrier amplitude is biased; **temporal change** within a sounding epoch is still informative. |
| Absolute phase per subcarrier | Rotated arbitrarily by `arg(V_k)`. Phase calibration (e.g. Hampel filter unwrap) becomes ill-defined. |
| Multi-antenna AoA features | Destroyed - `V` projects `H` onto a single steering direction. |
| Frequency-domain (FFT) analysis | Sounding-epoch boundaries (~10 ms - 1 s, traffic-dependent) introduce discrete jumps in the time series. These leak into the spectrum as sub-Hz noise that overlaps the breathing band (0.1 - 0.5 Hz) and HR band (0.8 - 3 Hz). Use only with caution. |
| Motion / activity detection (variance-based) | Mostly preserved within a sounding epoch. Noisy across epoch boundaries. |

### Recommended workflow when capturing beamformed CSI

1. Enable `CONFIG_ESP32_CSI_TOOL_ACQUIRE_BEAMFORMED=y` AND `CONFIG_ESP32_CSI_TOOL_ACQUIRE_HE_SU=y` (and `CONFIG_ESP32_CSI_TOOL_ENABLE_11AX=y`).
2. In post-processing, segment the capture into approximately stationary "V epochs" by detecting discontinuities in CSI (e.g. amplitude jumps > 3 sigma over the rolling baseline).
3. Run sensing analyses **per epoch** rather than across the entire trace; treat epoch boundaries as missing data.
4. Alternatively, request that the AP fall back to non-beamformed by associating with weaker signal levels (some routers disable BF below an SNR threshold).

### `acquire_beamformed` CSV column

The v2 super-set CSV exposes a per-row `acquire_beamformed` field at column 34 (zero-indexed). It is a **session-level flag** replicated per row from the firmware-side `CONFIG_ESP32_CSI_TOOL_ACQUIRE_BEAMFORMED`:

- **`acquire_beamformed = 0`**: the driver filtered out beamformed frames at the radio level. **Strong guarantee** that no row in this capture contains beamformed CSI - safe to treat as `H`.
- **`acquire_beamformed = 1`**: beamformed frames are accepted into the capture. **Weak guarantee** - the file may contain a mix of beamformed (`H_eff = H * V`) and non-beamformed (`H`) rows, depending on which frame types the AP actually sent. **ESP-IDF v6.0.1 does not expose a per-frame beamformed bit**, so per-row discrimination is not possible from this column alone.

This asymmetry is useful at the dataset level: an ML pipeline curating "clean H" training data can `df = df[df.acquire_beamformed == 0]` to filter out files where contamination is even possible. The boot-time `# CSI_ACQUIRE_BEAMFORMED=...` comment line above the column header carries the same value once, for tools that prefer file-level metadata.

## Two layers must agree for HE-LTF: protocol AND acquisition

A precondition for any HE-LTF related capture (and the `acquire_csi_su` / `acquire_csi_mu` flags below) is that the chip is **actually negotiating 802.11ax with its peers** at the MAC layer. The firmware ensures this by setting `WIFI_PROTOCOL_11AX` in the bitmask passed to `esp_wifi_set_protocols()` for chips with `CONFIG_SOC_WIFI_HE_SUPPORT` (C5 / C6 / C61). Users wanting an 11n-only ablation can disable this via `CONFIG_ESP32_CSI_TOOL_ENABLE_11AX=n` in menuconfig.

If you observe `acquire_csi_su=1` set but no HE-LTF frames arriving, check `CONFIG_ESP32_CSI_TOOL_ENABLE_11AX` first.

## ESP32-C6 #14271 - missing L-LTF and HT-LTF subcarrier reorder in HE acquisition mode

**Symptom.** When the user enables `acquire_csi_su = 1` or `acquire_csi_mu = 1` in `wifi_csi_acquire_config_t` on ESP32-C6, the L-LTF data section is absent from the callback buffer (256 -> 128 byte), and the HT-LTF data is reordered relative to the order documented in the v6.0.1 Wi-Fi driver guide. Reference: espressif/esp-idf#14271 (open as of v6.0.1 stable).

**Workaround.**

1. The chip-aware decoder in `_components/chip_aware_decoder.h` detects `CONFIG_IDF_TARGET_ESP32C6 && data->len == 128` and tags the frame with the layout id `C6_HT20_NO_LLTF`.
2. The CSV super-set carries the layout id in the new `subcarrier_layout_id` column.
3. The host-side `python_utils/lib/chip_aware_decoder.py` reorders the subcarriers per the empirically derived mapping.

**Re-verification checklist when upstream merges a fix.**

- Re-run E2 8-board self-pair smoke on C6.
- If `data->len == 256` again under HE mode, drop the layout tag and update this section.

## ESP32-C61 #18118 - CSI broken for 802.11g packets

**Symptom.** On ESP32-C61, `wifi_csi_info_t.buf` does not change between consecutive 802.11g packets and the data is unusable for sensing. The fault was traced to esp-phy-lib commit `cb9f62b` (per the upstream issue thread). 802.11n / 802.11ax packets are unaffected.

**Workaround.**

1. `sdkconfig.defaults.esp32c61` disables `CONFIG_ESP_WIFI_PROTOCOL_11G` so that the AP and STA never negotiate 11g.
2. The same defaults file enables `CONFIG_SHOULD_COLLECT_ONLY_LLTF` which sets `acquire_csi_force_lltf = 1` at run time. This is the only path that consistently restores CSI for legacy frames on C61.
3. The CSV `acquire_force_lltf` column records the runtime setting per frame.

## ESP32-C5 #18493 - static CSI IQ buffer on 5 GHz channels

**Symptom.** When ESP32-C5 is locked to a 5 GHz channel (36 - 165), `wifi_csi_info_t.buf` only ever contains one of two fixed IQ patterns regardless of the actual physical channel state. 2.4 GHz operation is unaffected. Espressif marks the issue as "Done internally" on v5.5.4. On v6.0.1 the symptom did not reproduce here: across fifteen 5 GHz sessions (channels 36 to 48, three board pairs, two rooms) the fraction of consecutive records with bit-identical IQ payloads is exactly 0.0000. The 106-byte record length does reproduce and is a separate open layout question.

**Status.** The 5 GHz path is an opt-in Kconfig choice. The default is 2.4 GHz only, but users can switch to 5G_ONLY or AUTO via menuconfig if they want to verify whether the upstream fix has actually shipped on the v6.0.1 main branch.

**Configuration.** In `idf.py menuconfig`:

```
ESP32 CSI Tool Config
  -> ESP32-C5 band selection
     -> ESP32-C5 Wi-Fi band mode
        ( ) 2.4 GHz only (default; #18493 workaround)
        ( ) 5 GHz only (experimental)
        ( ) Auto / dual-band (experimental)
     -> ESP32-C5 5 GHz channel (40)            # only when 5G_ONLY/AUTO
     -> ESP32-C5 5 GHz bandwidth: HT40         # only when 5G_ONLY/AUTO
```

The runtime helper `wifi_band_apply()` in `_components/wifi_band_component.h` reads the choice and calls the v5.4 dual-band API trio (`esp_wifi_set_band_mode`, `esp_wifi_set_protocols`, `esp_wifi_set_bandwidths`). The same helper then sets the 5 GHz channel from `CONFIG_ESP32_CSI_TOOL_C5_5G_CHANNEL` and logs the channel read back from the driver.

Two ESP-IDF v6.0.1 API contracts matter here and are easy to get wrong:

- `esp_wifi_set_band_mode()` attention 6 — a band change with no channel yet set for the new band silently assigns a default: **channel 1 for 2.4 GHz, channel 36 for 5 GHz**. So skipping the explicit channel set does not produce an error, it produces a receiver quietly parked on channel 36. The helper's `WIFI_MODE_NULL` guard therefore covers only the per-interface calls; the global band/channel calls always run.
- `esp_wifi_set_channel()` attention 6 — in the 5 GHz band the secondary channel is derived from the primary per 802.11 and **any manually configured secondary channel is ignored**. The helper passes `WIFI_SECOND_CHAN_NONE` and reports whatever `esp_wifi_get_channel()` returns, rather than claiming a secondary channel it did not actually choose.

**Verification recipe (recommended before treating 5 GHz CSI as trustworthy).**

1. Configure `5G_ONLY` mode and a quiet UNII-1 channel (default 40).
2. Capture 60 seconds of CSI in a still room (no movement).
3. Capture 60 seconds with someone walking through the LOS.
4. Compare per-subcarrier amplitude variance between the two captures. If the variance is similar (i.e. movement does not move the CSI), the buffer is still static and #18493 has not been fixed for your v6.0.1 commit. Switch back to 2G_ONLY.

**Where the layout id appears in CSV.** On 5 GHz frames the v2 column 29 (`subcarrier_layout_id`) reads `C5_5GHZ` . Frames are no longer dropped by `chip_aware_should_drop`; only `UNKNOWN`-layout frames are dropped.

**Hardware verdict (2026-08-01): NOT REPRODUCED on this setup.** C5-to-C5 pair (both rev v1.0),
5G_ONLY, channel 40, HT20, 100 Hz broadcast at 6 Mbps 11a legacy OFDM, 60 s still vs 60 s with
a person walking the LOS (~11.2k frames each, 53-subcarrier layout):

- *Frozen-buffer test*: 0.0000 of consecutive frames have identical `csi_raw` in either capture
  — the literal "static IQ buffer" is absent.
- *Motion test*: gain-normalised per-subcarrier temporal std, 0.25 s smoothed to isolate the
  human-motion band, is **1.59x higher with movement** (0.1608 vs 0.1013).
- *Channel structure*: the still capture shows a frequency-selective mean |H| profile
  (0..85 across subcarriers), i.e. a real channel, not a flat artefact.

Conclusion: on IDF v6.0.1 with the legacy-OFDM (L-LTF) path, C5 5 GHz CSI responds to the
channel and is usable for sensing. Scope limits: one board pair, one channel (40), HT20,
legacy OFDM sounding only — #18493 may still affect other rates/formats (e.g. HE) or earlier
IDF revisions. The empirical recipe above remains the gate before trusting any new 5 GHz
configuration. Note 5 GHz beacons are OFDM (no DSSS band plan), so beacon MGMT frames DO
produce CSI on 5 GHz, unlike 2.4 GHz where SoftAP beacons default to 1 Mbps DSSS.

## ESP32-C5: linking `esp_driver_usb_serial_jtag` kills the USJ primary console (2026-07-31, root-caused)

**Symptom.** On ESP32-C5 under ESP-IDF v6.0.1, an application built with the primary console on
USB-Serial-JTAG (`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y`) produces **zero console output** — not
even the 2nd-stage bootloader banner reaches the host — and afterwards esptool fails every reset
mode with pySerial `Write timeout` (the CDC RX FIFO fills and is never drained). The device stays
enumerated on USB the whole time. Only manual BOOT+RST (or power cycle) recovers it. ROM download
mode is completely unaffected: flashing, hash verification and espefuse all work.

**Root cause, isolated by bisection on stock `hello_world`** (Waveshare ESP32-C5-WIFI6-KIT,
chip rev v1.0, all runs same board / same host method — a reader taking a *fresh* serial handle
every 2 s):

| build | component set | primary console | `esp_driver_usb_serial_jtag` linked | result |
|---|---|---|---|---|
| stock `hello_world` (`MINIMAL_BUILD ON`) | trimmed | USJ | no | prints |
| + `FREERTOS_HZ=1000` | trimmed | UART (USJ secondary) | no | prints |
| minus `MINIMAL_BUILD` | **full** | UART (USJ secondary) | yes | prints |
| minus `MINIMAL_BUILD` | **full** | **USJ** | yes | **silent** |
| stock + only `PRIV_REQUIRES esp_driver_usb_serial_jtag` | trimmed | **USJ** | **yes — the single delta** | **silent** |

The last row is the verdict: with zero application-code changes, merely **linking the
`esp_driver_usb_serial_jtag` component into a USJ-primary-console build breaks the console and
the entire USJ CDC** on C5. (Mechanism consistent with IDF's console architecture: when the
driver component is present, the VFS console switches from the polling HAL write path to the
driver's interrupt path; that path is broken on C5 in v6.0.1. On ESP32-C6 the identical
combination works — extensively used in this project — so this is C5-specific.) The same
signature was seen with C3 + Wi-Fi + USJ in the v5.0 era
([esp-idf #8046](https://github.com/espressif/esp-idf/issues/8046), fixed internally).
Upstream-report material; see `docs/references/esp_csi_official_c5_refs.md` and
`docs/references/esp32c5_usb_serial_jtag_refs.md` for sources.

Why this project hit it: none of the three apps trims components, so the full IDF component set
— including `esp_driver_usb_serial_jtag` — is always linked. Espressif's own esp-csi examples
never hit it because their `sdkconfig.defaults` puts the console on **UART**
(`CONFIG_ESP_CONSOLE_UART_CUSTOM=y`, 921600) on every target including C5.

**What does NOT work: excluding the component.** `set(EXCLUDE_COMPONENTS
"esp_driver_usb_serial_jtag")` builds without error but the component is silently re-added:
`vfs` hard-PRIV_REQUIRES it unconditionally (`vfs/CMakeLists.txt:17`, tracked upstream as
IDF-13984 "Refactor to conditionally include stdio drivers based on Kconfig values"), and this
project needs `vfs` for the SD/FATFS sink. Verified by checking `project_description.json` and
the map file after a fullclean build — always check the artifact, not the exit code.
`CONFIG_USJ_ENABLE_USB_SERIAL_JTAG=n` is no escape either: its help text states it clock-gates
the USJ module at startup, which kills the console by design.

**Fix (applied).** The breakage needs USJ as the **primary** console: that is what activates the
driver's VFS device (`-u usb_serial_jtag_vfs_include_dev_init`) and connection monitor
(`-u usb_serial_jtag_connection_monitor_include`) via `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG_ENABLED`
in the driver's CMakeLists. With USJ as the **secondary** console the simple polling writer is
used and everything works — the full-component build with UART primary + USJ secondary printed
its entire boot log over USB in the bisection (row 3 of the matrix). So
`sdkconfig/sdkconfig.defaults.esp32c5` no longer sets `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y` and
instead documents that C5 keeps the IDF default: **UART primary + USB-Serial-JTAG secondary**.
All output (bootloader log included) still arrives over the same USB port. Trade-off: console
*input* goes to UART0, so interactive stdin over USB is unavailable on C5; CSI capture does not
use stdin. C3/C6/C61/S3 keep their USJ-primary consoles — the bug is C5-specific (C6 with the
identical combination is extensively hardware-verified in this project).

**The DFRobot FireBeetle 2 (DFR1222, MAC `d0:cf:13:ea:33:e8`) that was written off as defective
is probably fine.** It exhibited this same signature *because our firmware was already on it*.
The one experiment that seemed to condemn it — stock `hello_world` silent after a power cycle —
used a capture loop that only reopened the port on an exception; a handle opened against a
pre-reset device instance can go stale without ever throwing, reading 0 bytes forever. That
methodology was later shown unreliable and replaced by the rolling fresh-handle reader. Re-test
the FireBeetle with the fixed firmware before returning it.

**Operational learnings kept from the investigation (all still valid):**

- Per [esptool's C5 troubleshooting page](https://docs.espressif.com/projects/esptool/en/latest/esp32c5/troubleshooting.html),
  after manually entering download mode with BOOT, USB-Serial-JTAG can only issue a **core
  reset**, which does **not** re-sample the boot straps — the chip stays in download mode and
  `--after hard-reset` does not get it out. Use **`--after watchdog-reset`** on C5 (or EN /
  power cycle). This is why live boots kept showing `boot:0x8 (DOWNLOAD)` during the
  investigation.
- `esptool --before no-reset` doubles as a state probe: it connects only if the chip is sitting
  in the ROM download stub. `Write timeout` there proves the chip is executing from flash.
- XTAL is NOT the issue on either board: `espefuse summary` shows `XTAL_48M_SEL = 1 (0b001)`
  (odd parity = 48 MHz, correct) with `XTAL_48M_SEL_MODE = eFuse`. Read the eFuse before
  considering a burn — a burn here would have been irreversible and wrong.
- Windows serial handles opened across a device reset can go stale silently. Any capture that
  must survive a reset needs to take a fresh handle every couple of seconds, not merely reopen
  on exceptions.

## Notes

- All three issues are upstream firmware / driver bugs. VeriCSI-ESP32 does not patch the IDF; it characterises the bug, isolates affected frames at runtime, and presents the necessary metadata in CSV so that downstream Python tooling can correctly interpret each frame.

## Bench-verified findings, 2026-08-08 (E12a/E13 bring-up)

Conditions common to all entries: ESP-IDF v6.0.1 stable, 2.4 GHz channel 6,
two co-located senders at 50 Hz gptimer broadcast, receivers in passive
promiscuous mode, 180 cm link.

### 1. esp_wifi_80211_tx cannot inject at HT rates

Configuring `wifi_tx_rate_config_t{phymode=WIFI_PHY_MODE_HT20, rate=MCS0}`
via `esp_wifi_config_80211_tx` is accepted (tx counter increments, no error
return) but the frames never appear on the air. Verified on ESP32-S3 with
plain Data and QoS-Data headers alike; receivers on-channel capture zero
frames while legacy 6 Mbps OFDM from the same board arrives at full rate.
Workaround: HT-format sounding transmits through ESP-NOW with
`esp_now_set_peer_rate_config` (broadcast peer, phymode HT20/HT40). The
gptimer broadcast component selects this path automatically via
`ESP32_CSI_TOOL_BCAST_PHY_FORMAT`.

### 2. 2.4 GHz HT40 is rejected while 11AX sits in the protocol mask

`esp_wifi_set_bandwidths(WIFI_BW40)` returns ESP_ERR_INVALID_ARG on the C6
when the AP protocol mask includes WIFI_PROTOCOL_11AX, and ESP-NOW then
rejects the HT40 phymode ("need change bandwidth to 40"). Same driver rule
as the documented 5 GHz VHT/HE case. Workaround (automatic in
`wifi_band_component.h`): requesting `ESP32_CSI_TOOL_2G_BANDWIDTH_HT40`
drops 11AX from the 2.4 GHz mask (11b/g/n).

### 3. USJ primary console stalls the application when no USB host is attached

A board whose primary console is USB-Serial-JTAG stops delivering its
broadcast within seconds of running from a bare power supply (no enumerated
host). Observed on S3 and C6 sender builds; the same firmware runs
indefinitely on a USB port. Related to the C5 secondary-mirror backpressure
already documented: every printf blocks on the unread USJ driver.
Workaround for headless roles: console to UART0
(`ESP_CONSOLE_UART_DEFAULT=y`) and disable CSI collection on sender-only
boards (`SHOULD_COLLECT_CSI=n`).

### 4. HT40 secondary-channel direction differs per chip generation

With bandwidth 40 on primary channel 6, the C6 softAP picks secondary
BELOW while the S3 picks ABOVE. A receiver sniffing the wrong pair decodes
nothing from the HT40 stream (0 of ~50 frames/s). The gptimer broadcast
component now pins SECOND_CHAN_BELOW in HT40 mode and the passive app
sniffs channel 6 BELOW when built with HT40 acquisition.

### 5. ESP32-C6 (MAC v2) delivers near-zero HT20 CSI while sniffing at 40 MHz

At SECOND_CHAN_BELOW the C6's HT40 records are healthy, but HT20 frames
yield 128-value arrays with amplitudes of 1 to 5 counts regardless of RSSI
(-53 observed), through the legacy and the ht20 acquire slots alike.
At 20 MHz sniffing HT20 records are healthy but HT40 frames cannot be
captured at all. MAC v2 therefore cannot produce both formats in one
capture. The C5 (MAC v3) handles the same dual-format capture correctly.
`ESP32_CSI_TOOL_ACQUIRE_LEGACY` (int, default 1) was added while isolating
this; keeping legacy acquisition off routes HT frames through HT slots.

### 6. CSI spectral orientation is not stable across extraction paths
(updated 2026-08-19)

The frequency axis of the reported CSI is not stable across extraction
paths, and the relation between the HT20 and HT40 axes is not a constant.
Measured on a single sender alternating HT20, HT40, HT20, HT20 at 100 Hz
over twelve 600 s runs on an undisturbed bench, with orientation resolved
per adjacent-frame pair by scoring both alignments:

* The winning alignment changes between runs on both receiver families.
  Per-run mirrored fractions span 0.01 to 0.73 on the ESP32 and 0.22 to
  1.00 on the C5. Six of twelve ESP32 runs prefer the direct alignment,
  so "HT40 is mirrored relative to HT20" is one of two states the same
  receiver produces, and which state a capture is in cannot be predicted.
* The per-pair decision is decisive: fewer than 6% of cross-format pairs
  fall within 2% of a tie, and restricting to pairs decided by a relative
  margin of at least 0.10 leaves the run-to-run spread unchanged. The
  movement is real orientation change and not counting noise.
* The C5 does not flip per frame. Within a run its decision is uniform
  (ten of twelve runs at a mirrored fraction of 0.988 to 1.000). The
  per-frame toggling reported by the earlier two-sender bench was an
  artifact of that bench, most plausibly the pairing across two boards.
* Same-format pairs from the same sender pick the direct alignment in all
  24 run-arms at relative margins of 1.8 to 4.2, which is what a stable
  axis looks like on this hardware.

Resolving orientation is necessary and not sufficient: after per-pair
resolution the cross-format amplitude error is still about twice the
same-format measurement floor (ratio 1.95, run-level 95% CI [1.80, 2.14]
on the ESP32; 2.23 [2.14, 2.33] on the C5). Keep a capture single-format
when amplitudes will be compared across records.

The earlier two-sender measurement (one board per format, paired-frame
amplitude correlation -0.85 direct / +0.87 mirrored on the ESP32) stands
as the original observation of the mirroring itself.

### 7. Subcarrier ordering is mixed-convention on the classic ESP32

Established from guard-band and DC fingerprints over 3000-frame averages:
HT20 records store L-LTF [0:64] and HT-LTF [64:128] in natural order
(-32..+31, DC at section index 32); HT40 records store L-LTF [0:64] in
natural order but HT-LTF40 [64:192] in FFT order (0..+63, -64..-1), tones
referenced to the primary channel center. The C5 (MAC v3) stores pruned
natural-order arrays (HT20: 57 tones -28..+28; HT40: 117 tones -58..+58,
primary-referenced). See csv_format_v2.md for the full table.

## ESP32-S2 hardware profile (consolidated, 2026-08-09)

The S2 is the only supported SoC with neither a USB-Serial-JTAG console
nor (on the bench boards) a UART bridge; everything below follows from its
native USB-OTG path and its older Wi-Fi MAC.

### 8. TinyUSB application CDC is intermittent

The application-mode CDC failed to enumerate throughout the primary
evaluation window (ROM bootloader enumerated 3 of 3; stock hello_world
failed identically). Later bring-ups on the identical firmware enumerate
and sustain multi-hour capture sessions, so the failure is intermittent,
not permanent. One further failure mode observed 2026-08-09: the CDC
drops off the bus mid-capture (host reads fail with ClearCommError, then
the port disappears); a reset re-enumerates it. Host tooling should use
non-blocking reads with reopen-on-exception and identify the ROM download
port by USB PID 0x0002 (application CDC is 0x4001); the COM number changes
across resets.

### 9. esp_wifi_config_80211_tx is wholly ineffective on the S2

On the S3, a legacy 6 Mbps OFDM rate request works and only HT rates fail
(finding 1). On the S2, even the legacy request is ignored: the config
call returns without error, gptimer_tx counts perfectly (100.00 frames/s,
zero failures), and no receiver on-channel produces a single CSI record,
the 1 Mbps DSSS fallback signature (DSSS frames carry no OFDM training
field). ESP-NOW with esp_now_set_peer_rate_config carries the broadcast
correctly on the S2 (HT20 MCS0 verified at 91.7 frames/s received).
Any controlled-rate sounding on the S2 must use the ESP-NOW path
(ESP32_CSI_TOOL_BCAST_PHY_HT20_MCS0).

### S2 role verification status (post-window bench evidence)

All three roles run with closure-audited captures on ESP-IDF v6.0.1:
passive 2 x 600 s at 72 frames/s, closure residuals 0.012% and 0.023%;
station 600 s at 130 frames/s received (11.3 frames/s sink drops all
accounted), closure 0.000%; AP broadcasting 100.00 frames/s with zero TX
failures, receiver-side closure 0.156%. Device clock rate +77.8 ppm
(wired edge-train measurement). 168,865 records replay through the
unmodified ESP32-CSI-Tool parser with zero exceptions and pointwise
identical amplitudes.

### Finding 6 addendum (2026-08-09): amplitude-only orientation resolution
is unreliable on the ESP32

A causal rule (per-stream EMA reference profile, one-time cross-stream
alignment over 100 pairs) evaluated on twelve dual-sender captures stays
within 5% of the per-pair oracle error on 73 to 100% of held-out pairs on
the C5, but locks into the wrong orientation on half the ESP32 runs
(agreement 2 to 44%). Orientation metadata from the extraction layer is
the needed fix; script: experiments/e11_e13_plan/e12a_orientation_rule.py.

## esp_csi_gain_ctrl ships empty archives for esp32 and esp32s2 (2026-08-18)

The vendor component `espressif/esp_csi_gain_ctrl` (0.1.5, Apache-2.0,
binary-only distribution) backs the schema v2.2 `agc_gain` / `fft_gain`
columns and the receive-gain lock. Its prebuilt archives for the `esp32`
and `esp32s2` targets contain no code: the `.text` section of the single
object inside is zero bytes in every IDF version directory from 4.4 to
6.0, while the c3 / c5 / c6 / c61 / s3 archives carry roughly 1 KB of
code each. All 41 archives match the package's own CHECKSUMS.json, so
this is the published state and not a corrupted download. The component
manifest lists both targets as supported, so the gap is invisible until
link time.

Consequence: `CONFIG_ESP32_CSI_TOOL_RX_GAIN_CTRL=y` fails at link with
`undefined reference to esp_csi_gain_ctrl_get_rx_gain` on esp32 and
esp32s2. The option defaults to `n`, and with it off both gain columns
read `-1` and every build is unaffected.

Hardware verification on the ESP32-C6 (2026-08-18): free-running
`agc_gain` took 13 distinct values (59 to 71) over a 25 s bench capture;
a forced gain of 40 read back as exactly 40 in 1230 of 1230 records,
deliberately outside the free-running range so constancy is attributable
to the lock. In a locked-versus-free comparison on two adjacent C6
boards, locking cost 5.7% of delivered pairs on a bench whose transmit
power stepped over a 16 dB ladder: frames below what the fixed gain can
demodulate are not delivered at all.

## Per-frame ESP-NOW rate-config switching drops the switch on rare frames (2026-08-18)

The `BCAST_PHY_ALT_PATTERN` option alternates the ESP-NOW peer phymode
per frame (pattern HT20, HT40, HT20, HT20). Across roughly 143,000
adjacent-frame pairs on two receivers, two pairs arrived as consecutive
HT40 frames, which the pattern cannot produce, so two format switches
did not reach the air. Rate: about 1 in 70,000. The transmitter counts
these failures and logs one warning per 256; analyses that key on the
pattern should drop pairs whose two formats are both HT40.

## ESP32-C5 / C61 #18982 - 106 字节 L-LTF 记录为 12-bit 模式（已获官方确认）

2026-08-16 本项目提交 issue #18982：106 字节 MAC v3 L-LTF 记录与文档的
"每子载波两有符号字节"布局矛盾（奇位字节仅取 <=6/256 种值）。
2026-08-26 维护者（QingzhaoYin, COLLABORATOR）回复确认：

- 驱动默认 `lltf_bit_mode = 0` = 12-bit 精度：每 4 个 int8 表示一个 CSI 值，
  I/Q 各为一个小端 16-bit 容器，取 [11:0] 有符号 12-bit。
- 26 个 CSI 点位于子载波 [-26:2:26]（每隔一个子载波，DC 除外）。
- 文档描述的两字节布局是可选的 8-bit 模式（`lltf_bit_mode = 1`），
  文档未跟上代码。

处置：固件在 MAC v3 分支显式写 `cfg.lltf_bit_mode = 0`（csi_component.h）；
host 解码器对 106 字节 C61_FORCE_LLTF / C5_5GHZ 记录走 12-bit 路径
（python_utils/lib/chip_aware_decoder.py），实测 C5 捕获 1303/1303 条
末容器恒零、幅度全在 12-bit 域。回复全文存档
references/web/espidf_issue_18982.md。注意该确认只覆盖 L-LTF 记录；
HT-LTF 记录（C5 的 114/234 字节）为常规 int8 对，实测奇位取值
104/92 种（全域），不受影响。
