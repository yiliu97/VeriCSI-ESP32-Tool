#!/usr/bin/env python3
"""Console/port consistency check, run automatically before every flash.

Reads the project sdkconfig for the configured primary console (USB-Serial-JTAG,
UART, or USB-CDC), classifies the flashing port by USB VID:PID, and warns when
console output will not appear on that port. Diagnostic only: always exits 0 so
the flash proceeds.

Port resolution: the ESPPORT environment variable when set (idf.py -p passes it
through to the flash target), otherwise every detected serial port is
classified. Wired as a dependency of the `flash` targets in each app
CMakeLists.
"""
import argparse
import os
import sys

ESPRESSIF_VID = 0x303A
USJ_PID = 0x1001  # on-chip USB-Serial-JTAG controller (C3/S3/C5/C6/C61)
BRIDGE_VIDS = {
    0x10C4: "CP210x UART bridge",
    0x1A86: "CH34x UART bridge",
    0x0403: "FTDI UART bridge",
    0x067B: "Prolific UART bridge",
}

TAG = "console-port-check:"


def parse_sdkconfig(path):
    cfg = {}
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            for line in f:
                line = line.strip()
                if line.startswith("#") or "=" not in line:
                    continue
                key, val = line.split("=", 1)
                cfg[key] = val.strip('"')
    except OSError:
        pass
    return cfg


def console_of(cfg):
    if cfg.get("CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG") == "y":
        return "USJ"
    if (cfg.get("CONFIG_ESP_CONSOLE_UART_DEFAULT") == "y"
            or cfg.get("CONFIG_ESP_CONSOLE_UART_CUSTOM") == "y"):
        return "UART"
    if cfg.get("CONFIG_ESP_CONSOLE_USB_CDC") == "y":
        return "USB-CDC"
    return None


def classify(port):
    if port.vid is None:
        return "no USB metadata"
    if port.vid == ESPRESSIF_VID:
        if port.pid == USJ_PID:
            return "native USB-Serial-JTAG"
        return "native Espressif USB (PID 0x%04X)" % (port.pid or 0)
    return BRIDGE_VIDS.get(port.vid,
                           "unknown USB device (VID 0x%04X)" % port.vid)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdkconfig", required=True,
                        help="path to the project sdkconfig")
    args = parser.parse_args()

    cfg = parse_sdkconfig(args.sdkconfig)
    console = console_of(cfg)
    if console is None:
        print(f"{TAG} no console choice in {args.sdkconfig}; skipping")
        return 0
    mirror = cfg.get("CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG") == "y"

    try:
        from serial.tools import list_ports
    except ImportError:
        print(f"{TAG} pyserial unavailable; skipping")
        return 0

    ports = list(list_ports.comports())
    espport = os.environ.get("ESPPORT")
    if espport:
        ports = [p for p in ports if p.device.lower() == espport.lower()]
        if not ports:
            print(f"{TAG} port {espport} not among detected serial ports; "
                  f"skipping")
            return 0
    elif not ports:
        print(f"{TAG} no serial ports detected; skipping")
        return 0

    for p in ports:
        kind = classify(p)
        head = f"{TAG} {p.device}: {kind}; primary console {console}"
        if console == "USJ" and "UART bridge" in kind:
            print(f"{head} -> WARNING: console and CSI output appear on the "
                  f"board's USB-Serial-JTAG port, not on this bridge. Monitor "
                  f"and capture on the USJ port, or rebuild with the UART "
                  f"fallback (idf.py -DUART_CONSOLE=1 reconfigure).")
        elif console == "UART" and kind == "native USB-Serial-JTAG":
            if mirror:
                print(f"{head} (USJ secondary mirror on) -> OK: output is "
                      f"mirrored to this port; stdin stays on UART0.")
            else:
                print(f"{head} -> WARNING: output goes to the UART0 pins; "
                      f"this port stays silent.")
        elif console == "USB-CDC" and "UART bridge" in kind:
            print(f"{head} -> WARNING: output appears on the native USB port, "
                  f"not on this bridge.")
        else:
            print(f"{head} -> OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
