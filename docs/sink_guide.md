# Sink Selection Guide

VeriCSI-ESP32 supports five concurrent CSV output paths, called *sinks*. Each sink is selected independently in `idf.py menuconfig` under `ESP32 CSI Tool Config -> Output sinks` (or via the legacy v1 names `CONFIG_SEND_CSI_TO_SERIAL` / `CONFIG_SEND_CSI_TO_SD`). Multiple sinks can be enabled simultaneously - the consumer task in `_components/csi_component.h` fans every CSV row out to all enabled sinks.

This guide helps you pick the right combination for your chip and use case.

## Sinks at a glance

| Sink | Kconfig | Chips | Throughput (sustained) | Driver / cable |
|---|---|---|---|---|
| UART (legacy console) | `SEND_CSI_TO_SERIAL` | All 7 | ~100 packets/s @ 921600 bps | CP2102 / CH340 / FT232 USB-UART bridge on most dev boards |
| USB-Serial-JTAG | `ESP32_CSI_TOOL_SINK_USB_JTAG` | C3, S3, C5, C6, C61 | 200+ packets/s (USB 1.1 FS, 12 Mbps) | Native peripheral; no driver install on Linux/macOS |
| TinyUSB CDC ACM | `ESP32_CSI_TOOL_SINK_USB_CDC` | S2 | 180+ packets/s (USB 1.1 FS) | `espressif/esp_tinyusb` managed component (auto-pulled for S2 builds) |
| SD card | `SEND_CSI_TO_SD` | ESP32 (TTGO T8 V1.7) | Limited by SD class (typical Class 10: 50+ packets/s sustained) | SPI on GPIO 13/14/15/2 (default; remap via Kconfig) |
| UDP | `ESP32_CSI_TOOL_SINK_UDP` | All 7 with active Wi-Fi | Network-limited; ~hundreds of packets/s on a quiet 2.4 GHz channel | One CSV line = one UDP datagram |

### Default per chip

These defaults are set by the `sdkconfig.defaults.<chip>` files:

| Chip | Default sinks | Notes |
|---|---|---|
| ESP32 (classic) | UART + SD | TTGO T8 V1.7 has SD slot; NodeMCU does not (SD calls are no-ops) |
| ESP32-S2 | UART + TinyUSB CDC | TinyUSB pulled in for S2 only |
| ESP32-S3 | USB-Serial-JTAG | Stdout routed to native peripheral |
| ESP32-C3 | USB-Serial-JTAG | Same |
| ESP32-C5 | USB-Serial-JTAG | 2.4 GHz only by default (5 GHz #18493 workaround) |
| ESP32-C6 | USB-Serial-JTAG | Headline Wi-Fi 6 RISC-V chip |
| ESP32-C61 | USB-Serial-JTAG | 802.11g disabled by default (#18118 workaround) |

## Picking a sink combination

### "I just want to look at CSI in real time"

Use whatever the chip's default sink is and run `python_utils/csi_live.py` directly (it spawns `idf.py monitor` itself; recommended over the legacy pipe pattern, especially on Windows). On C3 / S3 / C5 / C6 / C61 the bridge is USB-Serial-JTAG; on ESP32 / S2 it is the legacy UART bridge. Both work transparently:

```
python ../python_utils/csi_live.py -p /dev/ttyACM0 --subcarriers 0,8,16,24,32,40,48,56
```

(Use `/dev/ttyUSB0` on the ESP32 / S2 USB-UART bridge, or `COMx` on Windows. Add `-o FILE -t 30` to record + auto-exit, or `--no-plot` for headless record-only.)

### "I want a long autonomous capture without a host computer attached"

You need the SD sink. The ESP32-classic TTGO T8 V1.7 board is the only board in the verified hardware list that ships with an on-board SD slot. Set:

```
CONFIG_SEND_CSI_TO_SD=y
CONFIG_SEND_CSI_TO_SERIAL=n   # optional - reduces UART chatter
```

Each boot opens the next free `/sdcard/N.csv` file. Pull the SD card and read the CSV directly with `python_utils/parse_csi.py`.

### "I want to push CSI to a host PC over Wi-Fi"

Use the UDP sink. Useful for multi-receiver lab setups where each ESP32 streams to a central collector. Set:

```
CONFIG_ESP32_CSI_TOOL_SINK_UDP=y
CONFIG_ESP32_CSI_TOOL_UDP_HOST="192.168.4.2"
CONFIG_ESP32_CSI_TOOL_UDP_PORT=5005
```

Each CSV line is sent as one UDP datagram. On the host:

```
nc -ulp 5005 > captured.csv
```

Or pipe to the live viewer directly (legacy stdin mode):

```
nc -ulp 5005 | python python_utils/csi_live.py --stdin
```

### "I want maximum sustained packet rate (V7-final K5 benchmark style)"

Use the native USB path of the chip:

- C3 / S3 / C5 / C6 / C61: USB-Serial-JTAG (default).
- S2: TinyUSB CDC.
- ESP32 (classic): UART at 921600 bps - this is the chip's ceiling; the V7-final K5 benchmark on the AP-side uses ESP32-C6 specifically because the higher-rate paths are not available on the original ESP32.

The `_components/csi_component.h` consumer task drains the CSI queue without back-pressure on the Wi-Fi callback, so the bottleneck moves to the host-side reader.

### "I want both UART boot logs AND a high-rate CSV stream"

This is the S2 default. Boot messages and the `SETTIME:` flow stay on UART (where the existing Hernandez 2020 tooling expects them); CSV CSI data goes over the higher-throughput USB-CDC. Both sinks see every CSV row, and the user can tee them however they like.

## Performance notes

### UART baud rate

`CONFIG_ESP_CONSOLE_UART_BAUDRATE` defaults to 921600 in `sdkconfig.defaults`. Higher rates (1.5 Mbps, 1.55 Mbps) sometimes work depending on the USB-UART bridge chip and host OS; see Hernandez 2020 issue #5 for community findings. This tool keeps the upstream behaviour.

### USB-Serial-JTAG vs UART on the same chip

The on-chip USB-Serial-JTAG controller (C3 / S3 / C5 / C6 / C61) replaces the UART path when `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y`. The IDF console redirects `printf` / `fwrite(stdout)` to the USB peripheral automatically, so the `usb_jtag_component.h` is structurally identical to the UART sink - one is just much faster than the other. Keep both Kconfig flags consistent: when `USB-Serial-JTAG` is selected, leave `SEND_CSI_TO_SERIAL=n` to avoid duplicate writes.

### UART console fallback on USJ-primary targets

Every USJ-primary target (C3 / S3 / C6 / C61) also has a UART0 console path for boards or hosts where the USJ port is unusable (broken cable-mode detection, VM passthrough limits, or a C5-style console failure, see `docs/known_issues.md`). Configure it with:

```
idf.py -DUART_CONSOLE=1 reconfigure build flash
```

This appends `sdkconfig/sdkconfig.uart_console` (UART0 primary at 921600, same custom-baud pattern as the esp32/s2/c5 defaults) after the per-target defaults. The fallback console is bidirectional: CSV output and the `SETTIME:` input flow (`_components/input_component.h` reads the primary console's stdin) both move to UART0, so a USB-UART bridge on TXD0/RXD0 gives full read/write access. The secondary USJ console (IDF default when the primary is UART) mirrors output only; stdin stays on UART0. Firmware download over the same bridge works in every console configuration through the ROM UART download mode. The CMake cache variable persists across builds; `idf.py -DUART_CONSOLE=0 reconfigure` restores the per-target defaults.

### Flash-time console/port check

`python_utils/check_console_port.py` runs automatically before every `idf.py flash` (a dependency of the `flash` targets in each app CMakeLists). It compares the configured primary console in `sdkconfig` against the USB identity of the flashing port (native USB-Serial-JTAG is VID:PID 303A:1001; CP210x / CH34x / FTDI / Prolific are UART bridges) and prints a `console-port-check:` line per port: `OK` when console output will appear there, `WARNING` with the corrective action when it will not (for example a USJ-console build flashed through a UART bridge). The check is diagnostic only and never blocks the flash.

### TinyUSB CDC ACM (S2)

Implemented in `_components/usb_cdc_component.h`. CSV is queued in 256-byte chunks and flushed after each line; the per-line flush keeps host-side latency under one packet period at 100/200 Hz target rates. If the host disconnects while data is being produced, individual lines are dropped (with a rate-limited warning every 64 drops) rather than the whole consumer task stalling.

The S2 has no on-chip USB-Serial-JTAG. TinyUSB is the recommended high-rate path on S2 and the default in `sdkconfig.defaults.esp32s2`.

### SD card

The default pin map matches TTGO T8 V1.7:

| Signal | GPIO |
|---|---|
| MISO | 2 |
| MOSI | 15 |
| CLK | 14 |
| CS | 13 |

Override via `CONFIG_ESP32_CSI_TOOL_SD_*` Kconfig values for other boards.

The SD sink is implemented in `_components/sd_component.h` against the v6 SDMMC API (`sdspi_device_config_t` + `spi_bus_initialize` + `esp_vfs_fat_sdspi_mount`). On boards without an SD slot, calling `sd_init()` returns gracefully without crashing.

### UDP

One CSV line per datagram. UDP has no kernel-side flow control, so a slow / disconnected host quietly drops packets - the firmware logs a rate-limited warning every 256 send failures. Use UDP over a wired gigabit LAN if the receiver also runs CSI capture (Wi-Fi medium contention degrades both directions).

## Combining sinks

All five sinks are independent; enable any combination. `_components/sink_component.h` calls every enabled sink's `*_write()` for each CSV row. There is no priority or fallback - every sink sees a full copy.

Common combinations:

- **TTGO T8 (long autonomous run)**: SD + UART (UART for boot messages only).
- **S3 / C6 dev kit (interactive plotting)**: USB-Serial-JTAG only.
- **S2 dev kit (V7-final K5 packet rate benchmark)**: USB-CDC only (UART can be disabled to free the queue).
- **Multi-RX lab capture**: UDP from each receiver to a central collector PC.

## Adding a sink in your fork

The 5 sinks above cover the verified hardware in `docs/chip_support_matrix.md`. To add a new sink (e.g. SPI peripheral, BLE notify):

1. Add a header `_components/<your_sink>_component.h` exposing `<your_sink>_sink_init()`, `int <your_sink>_sink_write(data, len)` and `<your_sink>_sink_flush()`. The writer returns 0 when the full line was accepted and -1 on a detectable failure, and its comment states the detection boundary (what the return can and cannot see); a failed write lands in the `sink_fail` counter of the DIAG frame (2026-09-15 acquisition contract).
2. Wire it into `_components/sink_component.h` under a new Kconfig flag (`CONFIG_ESP32_CSI_TOOL_SINK_<YOUR>`).
3. Add the Kconfig entry to each `*/main/Kconfig.projbuild`.
4. Add a row to this guide.

Keep the API contract: `*_write` is called once per CSV row, never from ISR context, and is allowed to block briefly (a few tens of milliseconds at most) without back-pressuring the Wi-Fi rx callback.
