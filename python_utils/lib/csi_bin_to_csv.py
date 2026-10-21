"""
Decode a VeriCSI-ESP32 binary capture into CSV.

Use this when the firmware was built with `CONFIG_ESP32_CSI_TOOL_SINK_BINARY=y`.
The captured stream is a sequence of frames, each beginning with the magic
b'CSIF' and ending with a CRC32 over the frame body. See
`docs/binary_format.md` and `_components/binary_sink.h` for the wire format.

Run as a standalone script:

    # decode an entire capture file
    python csi_bin_to_csv.py --input capture.bin --output capture.csv

    # decode a live stream from idf.py monitor (the firmware emits binary
    # frames intermixed with the IDF console text; this filter detects
    # binary frames by their CSIF magic and writes the recovered CSV to
    # stdout)
    idf.py monitor | python csi_bin_to_csv.py --input - --output -

Or as a library:

    from csi_bin_to_csv import iter_records, frame_to_csv_row
"""

from __future__ import annotations

import argparse
import binascii
import struct
import sys
from typing import Iterator, Optional, Tuple

# ---------------------------------------------------------------------------
# Layout MUST match _components/binary_sink.h binary_meta_t struct.
# Any change to the C struct also requires updating META_FORMAT here.
# ---------------------------------------------------------------------------

FRAME_MAGIC  = b"CSIF"
STREAM_MAGIC = b"CSI6"

# struct binary_meta_t is __attribute__((packed)), little-endian fields.
# All multi-byte ints are little-endian; chip_id / firmware_version /
# subcarrier_layout_id are NUL-terminated fixed-size byte strings.
META_FORMAT = (
    "<"          # little-endian, no padding
    "B"          # type_code
    "B"          # role_code
    "6s"         # mac
    "b"          # rssi (signed)
    "B"          # rate
    "B"          # sig_mode
    "B"          # mcs
    "B"          # bandwidth
    "B"          # smoothing
    "B"          # not_sounding
    "B"          # aggregation
    "B"          # stbc
    "B"          # fec_coding
    "B"          # sgi
    "b"          # noise_floor (signed)
    "B"          # ampdu_cnt
    "B"          # channel
    "B"          # secondary_channel
    "I"          # local_timestamp (uint32)
    "B"          # ant
    "H"          # sig_len (uint16)
    "B"          # rx_state
    "B"          # real_time_set
    "Q"          # real_timestamp_ns (uint64)
    "H"          # len (uint16)
    "16s"        # chip_id
    "16s"        # firmware_version
    "H"          # csi_buf_len (uint16)
    "24s"        # subcarrier_layout_id
    "I"          # gptimer_seq (uint32)
    "B"          # he_ltf_present
    "B"          # he_stbc_mode
    "B"          # acquire_force_lltf
    "B"          # acquire_beamformed (was `reserved` earlier; same offset, value 0 then)
)
META_SIZE = struct.calcsize(META_FORMAT)
# Sanity guard: the firmware-side binary_meta_t is 108 bytes under
# __attribute__((packed)). If this assertion fires, the C struct layout in
# _components/binary_sink.h has drifted from META_FORMAT here.
assert META_SIZE == 108, f"META_SIZE expected 108, got {META_SIZE}"

ROLE_DECODE = {0: "STA", 1: "AP", 2: "PASSIVE"}


# ---------------------------------------------------------------------------
# CRC32: ESP-ROM uses the standard IEEE 802.3 polynomial; binascii.crc32 in
# Python's standard library matches it bit-for-bit. See
# _components/binary_sink.h for the firmware-side computation.
# ---------------------------------------------------------------------------

def crc32_le(data: bytes) -> int:
    return binascii.crc32(data) & 0xFFFFFFFF


# ---------------------------------------------------------------------------
# Stream parser
# ---------------------------------------------------------------------------

def _read_exact(stream, n: int) -> Optional[bytes]:
    """Read exactly n bytes or return None on EOF / partial read."""
    buf = b""
    while len(buf) < n:
        chunk = stream.read(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


def _scan_for_magic(stream, magic: bytes) -> Optional[int]:
    """
    Read bytes from `stream` until `magic` is matched. Returns the number
    of bytes skipped before the magic was found (so the caller can warn
    about resync events), or None on EOF.
    """
    skipped = 0
    window = bytearray()
    while True:
        b = stream.read(1)
        if not b:
            return None
        window += b
        if len(window) > len(magic):
            window.pop(0)
            skipped += 1
        if bytes(window) == magic:
            return skipped


def _decode_meta(meta_bytes: bytes) -> dict:
    fields = struct.unpack(META_FORMAT, meta_bytes)
    (type_code, role_code, mac,
     rssi, rate, sig_mode, mcs, bandwidth,
     smoothing, not_sounding, aggregation, stbc, fec_coding, sgi,
     noise_floor, ampdu_cnt, channel, secondary_channel,
     local_timestamp, ant, sig_len, rx_state,
     real_time_set, real_timestamp_ns,
     length,
     chip_id, firmware_version, csi_buf_len, subcarrier_layout_id,
     gptimer_seq, he_ltf_present, he_stbc_mode, acquire_force_lltf,
     acquire_beamformed) = fields
    return {
        "type": "CSI_DATA" if type_code == 0 else f"UNKNOWN_{type_code}",
        "role": ROLE_DECODE.get(role_code, f"UNKNOWN_{role_code}"),
        "mac": ":".join(f"{b:02X}" for b in mac),
        "rssi": rssi,
        "rate": rate,
        "sig_mode": sig_mode,
        "mcs": mcs,
        "bandwidth": bandwidth,
        "smoothing": smoothing,
        "not_sounding": not_sounding,
        "aggregation": aggregation,
        "stbc": stbc,
        "fec_coding": fec_coding,
        "sgi": sgi,
        "noise_floor": noise_floor,
        "ampdu_cnt": ampdu_cnt,
        "channel": channel,
        "secondary_channel": secondary_channel,
        "local_timestamp": local_timestamp,
        "ant": ant,
        "sig_len": sig_len,
        "rx_state": rx_state,
        "real_time_set": real_time_set,
        "real_timestamp_ns": real_timestamp_ns,
        "len": length,
        "chip_id": chip_id.split(b"\x00", 1)[0].decode("utf-8", errors="replace"),
        "firmware_version": firmware_version.split(b"\x00", 1)[0].decode("utf-8", errors="replace"),
        "csi_buf_len": csi_buf_len,
        "subcarrier_layout_id": subcarrier_layout_id.split(b"\x00", 1)[0].decode("utf-8", errors="replace"),
        "gptimer_seq": gptimer_seq,
        "he_ltf_present": he_ltf_present,
        "he_stbc_mode": he_stbc_mode,
        "acquire_force_lltf": acquire_force_lltf,
        "acquire_beamformed": acquire_beamformed,
    }


def iter_records(stream) -> Iterator[Tuple[dict, bytes]]:
    """
    Yield (metadata_dict, csi_payload_bytes) for each binary frame on
    `stream`. The stream may begin with the optional CSI6 stream header
    (skipped) or directly with the first CSIF frame. Bytes between frames
    (e.g. ASCII boot log lines that v1 grep tools would have captured) are
    silently skipped.
    """
    # Optional stream header detection: read the first 16 bytes and see if
    # they start with CSI6. If not, rewind the bytes back into the buffer.
    head = stream.read(16)
    if head and head.startswith(STREAM_MAGIC):
        # Discard the rest of the 16-byte stream header.
        if len(head) < 16:
            tail = _read_exact(stream, 16 - len(head))
            if tail is None:
                return
    elif head:
        # Push the bytes back as a virtual prefix by wrapping the stream.
        stream = _PrefixedStream(head, stream)

    while True:
        skipped = _scan_for_magic(stream, FRAME_MAGIC)
        if skipped is None:
            return
        if skipped > 0:
            # Non-frame bytes between frames; harmless
            # mixed-stream design.
            pass

        # Read meta_len + meta + csi_len + csi + crc32.
        len_bytes = _read_exact(stream, 2)
        if len_bytes is None:
            return
        meta_len = struct.unpack("<H", len_bytes)[0]
        if meta_len != META_SIZE:
            # Either the format version drifted or this was a false-positive
            # CSIF inside random data; resync.
            print(f"warning: unexpected meta_len={meta_len}; resyncing", file=sys.stderr)
            continue

        meta_bytes = _read_exact(stream, meta_len)
        if meta_bytes is None:
            return

        csi_len_bytes = _read_exact(stream, 2)
        if csi_len_bytes is None:
            return
        csi_len = struct.unpack("<H", csi_len_bytes)[0]
        if csi_len > 4096:
            print(f"warning: implausible csi_len={csi_len}; resyncing", file=sys.stderr)
            continue
        csi_bytes = _read_exact(stream, csi_len)
        if csi_bytes is None:
            return

        crc_bytes = _read_exact(stream, 4)
        if crc_bytes is None:
            return
        crc_recv = struct.unpack("<I", crc_bytes)[0]

        crc_calc = crc32_le(
            FRAME_MAGIC + len_bytes + meta_bytes + csi_len_bytes + csi_bytes
        )
        if crc_calc != crc_recv:
            print(f"warning: CRC mismatch (recv 0x{crc_recv:08x}, "
                  f"calc 0x{crc_calc:08x}); skipping frame", file=sys.stderr)
            continue

        yield _decode_meta(meta_bytes), csi_bytes


class _PrefixedStream:
    """Stream wrapper that returns `prefix` first, then reads from `inner`."""
    def __init__(self, prefix: bytes, inner):
        self._prefix = bytearray(prefix)
        self._inner = inner

    def read(self, n: int) -> bytes:
        if self._prefix:
            take = self._prefix[:n]
            self._prefix = self._prefix[n:]
            if len(take) >= n:
                return bytes(take)
            return bytes(take) + self._inner.read(n - len(take))
        return self._inner.read(n)


# ---------------------------------------------------------------------------
# CSV emission
# ---------------------------------------------------------------------------

# Order matches docs/csv_format_v2.md: 25 v1 columns + CSI_DATA + 10 v2 columns.

V2_COLUMN_HEADER = (
    "type,role,mac,rssi,rate,sig_mode,mcs,bandwidth,smoothing,not_sounding,"
    "aggregation,stbc,fec_coding,sgi,noise_floor,ampdu_cnt,channel,secondary_channel,"
    "local_timestamp,ant,sig_len,rx_state,real_time_set,real_timestamp,len,"
    "CSI_DATA,"
    "chip_id,firmware_version,csi_buf_len,subcarrier_layout_id,gptimer_seq,"
    "he_ltf_present,he_stbc_mode,acquire_force_lltf,acquire_beamformed,frame_type\n"
)


def frame_to_csv_row(meta: dict, csi_payload: bytes) -> str:
    """
    Render one binary frame as a v2 super-set CSV row. CSI bytes are
    formatted with the trailing-space-before-bracket convention so that
    serial_plot_csi_live.py's csi_data.pop() compatibility shim still works.
    """
    csi_str = "[" + " ".join(str(b if b < 128 else b - 256) for b in csi_payload) + " ]"
    real_ts_seconds = meta["real_timestamp_ns"] / 1e9
    parts = [
        meta["type"],
        meta["role"],
        meta["mac"],
        str(meta["rssi"]),
        str(meta["rate"]),
        str(meta["sig_mode"]),
        str(meta["mcs"]),
        str(meta["bandwidth"]),
        str(meta["smoothing"]),
        str(meta["not_sounding"]),
        str(meta["aggregation"]),
        str(meta["stbc"]),
        str(meta["fec_coding"]),
        str(meta["sgi"]),
        str(meta["noise_floor"]),
        str(meta["ampdu_cnt"]),
        str(meta["channel"]),
        str(meta["secondary_channel"]),
        str(meta["local_timestamp"]),
        str(meta["ant"]),
        str(meta["sig_len"]),
        str(meta["rx_state"]),
        str(meta["real_time_set"]),
        f"{real_ts_seconds:.6f}",
        str(meta["len"]),
        csi_str,
        meta["chip_id"],
        meta["firmware_version"],
        str(meta["csi_buf_len"]),
        meta["subcarrier_layout_id"],
        str(meta["gptimer_seq"]),
        str(meta["he_ltf_present"]),
        str(meta["he_stbc_mode"]),
        str(meta["acquire_force_lltf"]),
        str(meta.get("acquire_beamformed", 0)),
        # frame_type is CSV-only; the binary META_FORMAT does not carry it.
        # Emit "?" so the column count matches live captures and downstream
        # tooling can detect "unknown" vs DATA/MGMT/ACK/CTRL.
        "?",
    ]
    return ",".join(parts) + "\n"


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> int:
    p = argparse.ArgumentParser(description="VeriCSI-ESP32 binary -> CSV")
    p.add_argument("--input", default="-",
                   help="Binary capture file path, or '-' for stdin (default).")
    p.add_argument("--output", default="-",
                   help="CSV output path, or '-' for stdout (default).")
    p.add_argument("--no-header", action="store_true",
                   help="Skip the v2 column-header line (useful when concatenating).")
    args = p.parse_args()

    in_stream = sys.stdin.buffer if args.input == "-" else open(args.input, "rb")
    out_stream = sys.stdout if args.output == "-" else open(args.output, "w", encoding="utf-8")

    try:
        if not args.no_header:
            out_stream.write(V2_COLUMN_HEADER)
        n_frames = 0
        for meta, csi in iter_records(in_stream):
            out_stream.write(frame_to_csv_row(meta, csi))
            n_frames += 1
        print(f"[csi_bin_to_csv] wrote {n_frames} frames", file=sys.stderr)
        return 0
    finally:
        if args.input != "-":
            in_stream.close()
        if args.output != "-":
            out_stream.close()


if __name__ == "__main__":
    sys.exit(main())
