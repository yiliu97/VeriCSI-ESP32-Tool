# VeriCSI-ESP32

A Wi-Fi Channel State Information (CSI) measurement toolkit for the ESP32 SoC
family, built on ESP-IDF v6.0.1. One codebase covers seven SoCs across three
Wi-Fi generations, including Wi-Fi 6 (802.11ax) parts and the dual-band
ESP32-C5.

The toolkit targets measurement validity. Three properties are built in:

* **Timing fidelity.** Sounding is driven by a hardware timer configured in
  microseconds, and every CSI record carries a device timestamp. Analysis uses
  the recorded timestamps.
* **Loss accounting.** Device-side counters and 802.11 receive sequence numbers
  close the delivery books: `callbacks - queue drops = delivered rows`.
* **Explicit metadata semantics.** On Wi-Fi 6 parts the driver's receive-control
  structure omits most PHY metadata fields. Those fields are reported as `-1`
  (not available). A measured zero is reported as `0`.

## Supported hardware

| SoC | Tested board | Console / sink default | Wi-Fi 6 HE-LTF | Status |
|---|---|---|---|---|
| ESP32 | TTGO T8 V1.7, NodeMCU ESP32-WROOM-32U | UART 921600, optional SD | no | supported |
| ESP32-S3 | ESP32-S3-DevKitC-1 | USB-Serial-JTAG | no | supported |
| ESP32-C3 | ESP32-C3-DevKitM-1 | USB-Serial-JTAG | no | supported |
| ESP32-C6 | Diymore ESP32-C6-WROOM1 | USB-Serial-JTAG | yes | supported |
| ESP32-C5 | Waveshare ESP32-C5-WIFI6-KIT, DFRobot FireBeetle 2 (DFR1222) | UART 921600 primary, USJ secondary | yes | supported, 2.4 GHz and 5 GHz |
| ESP32-C61 | ESP32-C61-DEVKITC-1-N8R2 | USB-Serial-JTAG | yes | supported |
| ESP32-S2 | LOLIN ESP32-S2 mini | UART 921600, TinyUSB CDC | no | builds only, untested on hardware |

See [`docs/known_issues.md`](./docs/known_issues.md) for the ESP32-S2 status
and for per-SoC upstream IDF issues.

The ESP32-C5 console uses UART as the primary channel. A USB-Serial-JTAG
primary console fails on that SoC under IDF v6.0.1. The cause and the
bisection are recorded in [`docs/known_issues.md`](./docs/known_issues.md).

## Capabilities

* **Roles.** Passive sniffer, station, and access-point operation on all
  supported SoCs.
* **Frame formats.** CSI from legacy OFDM, HT20, HT40, VHT20 (802.11ac,
  C5 only), and HE20 (802.11ax SU) frames. Verified record layouts range from
  106 B (L-LTF) to 384 B (ESP32 HT40, two training-field blocks). The C6
  HE20 record is driver-declared at 512 B, but the firmware copy caps at
  384 B and observed exports carry 128-value payloads, so no full-length
  HE20 export is hardware-verified; the three lengths of every record
  (driver-declared, copied, exported) are separate fields, see
  `docs/csv_format_v2.md`. On MAC v3 parts (C5, C61) the 106 B
  L-LTF record is the driver-default 12-bit mode: four bytes per CSI
  value (each I/Q component a little-endian 16-bit container holding a
  signed 12-bit value in bits [11:0]), 26 tones at every second
  subcarrier from -26 to +26 excluding DC, confirmed by the maintainer
  reply on [esp-idf #18982](https://github.com/espressif/esp-idf/issues/18982);
  `python_utils/lib/chip_aware_decoder.py` decodes it. The two-bytes-per-
  subcarrier layout in the Wi-Fi driver guide describes the optional
  8-bit mode (`lltf_bit_mode = 1`), which this toolkit does not use.
* **Bands.** 2.4 GHz on all SoCs; 2.4 and 5 GHz on the ESP32-C5.
* **ACK-frame CSI.** Optional `dump_ack_en` capture; control-frame rows
  are tagged in the `frame_type` column.
* **Sinks.** Serial (USB-Serial-JTAG and UART), SD-SPI, and UDP, usable
  concurrently. Writers return a failure code that feeds the shared
  `sink_fail` DIAG counter (an any-sink-failed flag; per-sink counters
  remain future work).
* **Receive gain.** Per-record `agc_gain` / `fft_gain` readout and an
  optional gain lock via the vendor `esp_csi_gain_ctrl` component
  (schema v2.2). Hardware-verified on the ESP32-C6; unavailable on the
  ESP32 and ESP32-S2, whose vendor archives contain no code (see
  [`docs/known_issues.md`](./docs/known_issues.md)).
* **Bench controls.** The broadcast sender can alternate HT20 and HT40 on
  a fixed four-frame pattern (`BCAST_PHY_ALT_PATTERN`) and step its
  transmit power over a 16 dB ladder (`BCAST_TX_POWER_LADDER`), both off
  by default, for cross-format and gain experiments that need a stimulus
  from the transmit side.

### ESP32-C5 specific behavior

1. A USB-Serial-JTAG primary console is silenced when
   `esp_driver_usb_serial_jtag` is linked (see above); the console runs
   on UART instead.
2. Interface-less promiscuous capture (`WIFI_MODE_NULL`) delivers no CSI
   callbacks. The `passive` app automatically runs its sniffer on an idle
   station-mode radio on this SoC; no association is made.
3. VHT and HE are 20 MHz-only on this generation: with 11ac or 11ax in
   the protocol mask the driver rejects a 40 MHz bandwidth request
   (`ESP_ERR_INVALID_ARG`). The 5 GHz HT40 option masks the protocols
   down to 11a/n automatically. The same rule applies to 11ax on the C6
   at 2.4 GHz.

## Applications

* `active_ap`: SoftAP that other nodes associate with. An optional
  hardware-timer broadcast generates sounding traffic at a configured period.
* `active_sta`: Station that associates with an AP and generates uplink
  traffic.
* `passive`: Promiscuous receiver. Captures CSI without associating.

## Requirements

ESP-IDF **v6.0.1**. Install it with the
[Espressif getting-started guide](https://docs.espressif.com/projects/esp-idf/en/v6.0.1/get-started/index.html)
and confirm that a stock example builds and flashes on your board before
opening an issue here.

## Quick start

```
cd active_ap                       # or active_sta / passive
idf.py set-target esp32c6          # esp32 / esp32s2 / esp32s3 / esp32c3 / esp32c5 / esp32c6 / esp32c61
idf.py menuconfig                  # VeriCSI-ESP32 Config
idf.py build
idf.py -p PORT flash
```

Per-target defaults in `sdkconfig/sdkconfig.defaults.<target>` are merged
automatically, including console channel and baud rate.

If flashing a freshly connected board fails with a port error (`Could not
open COMx, the port is busy or doesn't exist`), enter ROM download mode
manually: hold BOOT, press and release RESET (or re-plug the USB cable while
holding BOOT), release BOOT, then flash again. Press RESET once after
flashing to leave download mode. This procedure applies to every SoC in the
family. Observed trigger: the USB-Serial-JTAG port re-enumerates during the
automatic reset on a factory-fresh board, so the port name esptool opened is
gone by the time it connects; ROM download mode keeps the port stable.

On ESP32-C5, use `--after watchdog-reset` when flashing with esptool directly.
A core reset does not re-sample the boot straps on that SoC, so the chip stays
in download mode:

```
python -m esptool --chip esp32c5 --port PORT --before default-reset \
    --after watchdog-reset write-flash --flash-mode dio --flash-freq 80m \
    --flash-size 2MB 0x2000 build/bootloader/bootloader.bin \
    0x8000 build/partition_table/partition-table.bin 0x10000 build/APP.bin
```

## Configuration

`idf.py menuconfig` opens `VeriCSI-ESP32 Config`. The submenus cover:

| Submenu | Contents |
|---|---|
| Application-layer broadcast | Hardware-timer sounding period in microseconds |
| SoftAP radio parameters | Beacon interval in TU |
| Wi-Fi 6 HE-LTF acquisition | SU / MU / beamformed / STBC / DCM / HT40 / VHT acquisition flags (C5 / C6 / C61; VHT is C5-only) |
| Output sinks | UART, USB-Serial-JTAG, USB-CDC, UDP, SD card, binary (the binary sink is currently unusable over a console: LF-to-CRLF translation corrupts the frames; a redesign is pending) |
| PSRAM ring buffer | Queue depth and PSRAM backing (ESP32-S3 with Octal PSRAM) |
| Diagnostics | DIAG counter frame period, consumer task scheduling |
| Binary frame sink | Fixed-layout binary output |
| ESP32-C5 band selection | 2.4 GHz, 5 GHz, or dual band; 5 GHz channel and bandwidth |
| 2.4 GHz bandwidth | HT40 radio bandwidth for STA and AP roles (all SoCs) |

The sounding period is configured in microseconds because integer-hertz
settings cannot express rates such as the 802.11 beacon rate of 9.765625 Hz
(100 TU). Expressed as a period, that rate is exactly 102400 us.

## Capturing data

`python_utils/csi_live.py` reads a serial port and records the raw stream:

```
python ../python_utils/csi_live.py -p COM3 -o -t 300 --no-plot
```

Bare `-o` auto-names the file `YYYYMMDDHHMM_<CHIP>_<ROLE>.csv` under
`code/data/`. `--no-plot` disables the live plot for unattended captures.

On boards with a UART bridge (ESP32, ESP32-S2), add `--raw --baud 921600`.
The `--raw` mode opens the port with pyserial and leaves DTR/RTS de-asserted,
which avoids resetting USB-Serial-JTAG targets. It also takes a fresh port
handle periodically so that a capture survives a device reset.

For the UDP sink, run the listener on the host:

```
python ../python_utils/udp_csi_listener.py --port 5555 -o capture.csv -t 300
```

## Output format

CSV, one row per CSI record. The first 26 columns and the position of the CSI
array (column 25, zero-indexed) match the original ESP32-CSI-Tool schema.
Eighteen fields are appended after the array (schema v2.2, 44 columns):
`chip_id`, `firmware_version`, `csi_buf_len`, `subcarrier_layout_id`,
`he_ltf_present`, `acquire_beamformed`, `gptimer_seq`, `frame_type`,
`rx_seq`, the 64-bit `timestamp_us`, per-record validity flags, the on-die
temperature, and the per-record receive gain (`agc_gain`, `fft_gain`), all
documented in [`docs/csv_format_v2.md`](./docs/csv_format_v2.md).

Because the appended fields follow the CSI array, parsers written for the
original schema read these files without modification.

Boot-time comment lines beginning with `#` record the active configuration
(sounding period, beacon interval, acquisition flags). Periodic `# DIAG` lines
record the counters used for loss accounting.

## Python utilities

| Script | Purpose |
|---|---|
| `csi_live.py` | Serial capture, optional live amplitude plot |
| `udp_csi_listener.py` | Host-side UDP sink receiver |
| `parse_csi.py` | Parser and pandas loader; auto-detects column generation |
| `lib/csi_bin_to_csv.py` | Binary sink decoder |
| `lib/chip_aware_decoder.py` | Per-SoC subcarrier layout identification |
| `lib/serial_append_time.py` | Appends host timestamps to a serial stream |

## Setting device time

The ESP32 clock is not synchronized to wall time. Three options:

1. Type `SETTIME: <unix_seconds>` followed by Enter while a monitor is attached.
2. Run `active_ap`, which sends its timestamp to associated `active_sta` nodes.
3. Append host timestamps on the receiving computer with
   `lib/serial_append_time.py`.

## Lineage and citation

VeriCSI-ESP32 originates from the ESP32-CSI-Tool released by Steven M.
Hernandez and Eyuphan Bulut alongside their WoWMoM 2020 paper *Lightweight and
Standalone IoT Based WiFi Sensing for Active Repositioning and Mobility*
([project page](https://stevenmhernandez.github.io/ESP32-CSI-Tool/)). The CSV
schema of that tool is preserved as a prefix of the format described above, and
parsers written against it continue to work.

The measurement core has been redesigned for ESP-IDF v6 and heterogeneous
ESP32 SoCs. Added since the original release: the Wi-Fi 6 receive-control
compatibility layer with explicit NULL semantics, microsecond-resolution
timer-driven sounding, sequence-level loss accounting with device diagnostics,
dual-band (5 GHz) support on ESP32-C5, the multi-sink output layer, and
per-record receive-gain state with an optional lock.

If you use this toolkit, please cite both the original ESP32-CSI-Tool paper and
this work. Details in [`docs/citation_lineage.md`](./docs/citation_lineage.md).

## Further documentation

* [`docs/known_issues.md`](./docs/known_issues.md): upstream IDF issues and
  per-SoC limitations, with reproduction steps
* [`docs/csv_format_v2.md`](./docs/csv_format_v2.md): column reference
* [`docs/sink_guide.md`](./docs/sink_guide.md): sink selection and throughput
* [`docs/packet_rate_guide.md`](./docs/packet_rate_guide.md): sounding rate configuration
