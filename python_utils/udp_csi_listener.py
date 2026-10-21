#!/usr/bin/env python3
"""Host-side UDP listener for the ESP32-CSI UDP sink (sink validation).

Binds 0.0.0.0:<port>, appends every datagram payload verbatim to the output
file, prints a once-per-second rate line. Ctrl+C or --duration to stop.

Usage:
    python udp_csi_listener.py --port 5555 -o ..\\data\\udp_c3_100hz_r1.csv -t 300
"""
import argparse
import socket
import sys
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5555)
    ap.add_argument("-o", "--output", required=True)
    ap.add_argument("-t", "--duration", type=float, default=None)
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", args.port))
    sock.settimeout(0.5)
    print(f"[udp_listener] bound 0.0.0.0:{args.port} -> {args.output}")

    n = 0
    n_prev = 0
    t0 = time.time()
    t_stat = t0
    try:
        with open(args.output, "ab") as f:
            while True:
                now = time.time()
                if args.duration and now - t0 >= args.duration:
                    break
                try:
                    data, _ = sock.recvfrom(65535)
                except socket.timeout:
                    data = None
                if data:
                    f.write(data)
                    if not data.endswith(b"\n"):
                        f.write(b"\n")
                    n += 1
                if now - t_stat >= 1.0:
                    print(f"[udp_listener] datagrams={n} rate={n - n_prev}/s")
                    n_prev = n
                    t_stat = now
    except KeyboardInterrupt:
        pass
    print(f"[udp_listener] done: {n} datagrams in {time.time()-t0:.0f} s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
