"""
Parse one or more CSI_DATA rows from a VeriCSI-ESP32 CSV
(or a legacy ESP32-CSI-Tool CSV).

The VeriCSI-ESP32 CSV is a strict super-set of the legacy schema: 25 v1 metadata columns,
then CSI_DATA at column 25 (zero-indexed), then the v2 columns appended at the
end. See ../docs/csv_format_v2.md.

Schema v2.2, 44 columns, is what this tool emits and the single reference
layout. Its tail is `first_word_invalid`, `rx_estimate_vld`, `gain_force_val`,
`die_temp_c10`, `agc_gain`, `fft_gain`; the last two are the per-record
receive gain and read -1 unless the build links esp_csi_gain_ctrl.

Widths earlier builds wrote are still read, because captures in those layouts
exist and one of them generates a figure in the paper: 34 (no
`acquire_beamformed`, `frame_type` or `rx_seq`), 35, 36, 37, 38 (schema v2.0,
adds `timestamp_us`), 41 (v2.1 without `timestamp_us`) and 42 (v2.1). Fields a
width does not carry are reported as 0 (`acquire_beamformed`), "?"
(`frame_type`) or -1 (the rest).

Width 41 is why the tail cannot be read with `if len(parts) > N` tests: it has
no `timestamp_us`, so index 37 there holds `first_word_invalid`. Every tail
block is dispatched on the exact width.

This means there is **no need for v1 -> v2 or v2 -> v1 converters**: a v1
reader sees a v2 file as "a v1 file with some extra trailing columns it
ignores", and a v2-aware reader detects the schema by counting commas.

Run this file directly to demo the parser on the bundled example_csi.csv:

    python parse_csi.py example_csi.csv

Import as a module to use parse_line() / parse_file() in your own pipeline.
"""

from __future__ import annotations

import os
import re
import sys
from math import sqrt, atan2
from typing import Dict, Iterable, Iterator, List, Optional, Tuple, Union


# Schema v2.2 is the current record layout and the only one this tool emits.
# Everything else here describes what older builds wrote, which captures on
# disk are still in, and it is stated as data rather than as a constant per
# generation. Eight named _V2_NUM_COLS_Gn constants had accumulated by
# 2026-08-18; the width is the fact, the generation number was never used for
# anything.
NUM_COLS = 44              # schema v2.2: the current layout
_V1_NUM_COLS = 26          # Hernandez 2020 ESP32-CSI-Tool

# Widths earlier builds emitted. Reading them is not legacy politeness: the
# figure of Section VI-D is generated from a 37-column capture, so dropping
# these breaks the paper's own plots.
_LEGACY_COLS = (34, 35, 36, 37, 38, 41, 42)

# Which tail blocks a given width carries. Dispatch is on the exact width and
# never on "index exists", because width 41 has no timestamp_us and index 37
# there holds first_word_invalid.
_COLS_WITH_TS = (38, 42, NUM_COLS)          # 64-bit timestamp_us at index 37
_COLS_WITH_V21 = (41, 42, NUM_COLS)         # the four v2.1 validity fields
_COLS_WITH_V22 = (NUM_COLS,)                # agc_gain and fft_gain

_V2_ACCEPTED_COLS = _LEGACY_COLS + (NUM_COLS,)

_CSI_BRACKET_RE = re.compile(r"\[(.*)\]")


def _split_csv_with_bracket(line: str) -> Optional[List[str]]:
    """
    Split one CSV row into fields, treating the bracketed CSI_DATA at column 25
    as a single field even though it contains spaces.

    Returns None if the row does not look like a CSI_DATA data row.
    """
    line = line.strip()
    if not line or line.startswith("#"):
        return None
    if not line.startswith("CSI_DATA,"):
        return None

    # The CSI bracket must be present and balanced; without it we cannot trust
    # the row.
    m = _CSI_BRACKET_RE.search(line)
    if m is None:
        return None
    bracket_start = m.start()
    bracket_end = m.end()  # one past the ']'

    # Anything before bracket_start is normal CSV; anything after bracket_end
    # is also normal CSV (the v2 trailing fields). We split each chunk
    # separately and re-stitch the bracketed CSI as one field at position 25.
    pre = line[:bracket_start].rstrip(",")
    post = line[bracket_end:].lstrip(",")
    bracket = line[bracket_start:bracket_end]

    pre_parts = pre.split(",")
    post_parts = post.split(",") if post else []
    return pre_parts + [bracket] + post_parts


def _parse_csi_bracket(s: str) -> List[int]:
    """Parse the '[1 -2 3 ...]' int8 array literal into a list of ints."""
    m = _CSI_BRACKET_RE.search(s)
    if m is None:
        return []
    inner = m.group(1).strip()
    if not inner:
        return []
    return [int(x) for x in inner.split() if x]


def parse_line(line: str) -> Optional[Dict]:
    """
    Parse a single CSI_DATA row.

    Returns a dict with the v1 metadata fields, the raw CSI int8 array, and -
    if the row carries the v2 super-set - the 8 v2 fields. Returns None for
    comment lines, the column header, blank lines and malformed rows.

    Schema detection is by column count alone: NUM_COLS is the current layout,
    _LEGACY_COLS the widths earlier builds wrote, _V1_NUM_COLS the Hernandez
    2020 one. Any other width is a partial or malformed row and returns None,
    which is what keeps serial-framing casualties out of the parsed set.
    """
    parts = _split_csv_with_bracket(line)
    if parts is None:
        return None
    if len(parts) not in (_V1_NUM_COLS,) + _V2_ACCEPTED_COLS:
        return None

    try:
        record: Dict = {
            "schema_version":   2 if len(parts) in _V2_ACCEPTED_COLS else 1,
            "type":             parts[0],
            "role":             parts[1],
            "mac":              parts[2],
            "rssi":             int(parts[3]),
            "rate":             int(parts[4]),
            "sig_mode":         int(parts[5]),
            "mcs":              int(parts[6]),
            "bandwidth":        int(parts[7]),
            "smoothing":        int(parts[8]),
            "not_sounding":     int(parts[9]),
            "aggregation":      int(parts[10]),
            "stbc":             int(parts[11]),
            "fec_coding":       int(parts[12]),
            "sgi":              int(parts[13]),
            "noise_floor":      int(parts[14]),
            "ampdu_cnt":        int(parts[15]),
            "channel":          int(parts[16]),
            "secondary_channel": int(parts[17]),
            "local_timestamp":  int(parts[18]),
            "ant":              int(parts[19]),
            "sig_len":          int(parts[20]),
            "rx_state":         int(parts[21]),
            "real_time_set":    int(parts[22]),
            "real_timestamp":   float(parts[23]),
            "len":              int(parts[24]),
            "csi_raw":          _parse_csi_bracket(parts[25]),
        }
    except (ValueError, IndexError):
        return None

    if record["schema_version"] == 2:
        try:
            record.update({
                "chip_id":               parts[26],
                "firmware_version":      parts[27],
                "csi_buf_len":           int(parts[28]),
                "subcarrier_layout_id":  parts[29],
                "gptimer_seq":           int(parts[30]),
                "he_ltf_present":        int(parts[31]),
                "he_stbc_mode":          int(parts[32]),
                "acquire_force_lltf":    int(parts[33]),
                # acquire_beamformed: session-level flag added in gen-2.
                # 0 in gen-1 captures (column absent) means "unknown but
                # likely false" - gen-1 firmware always set the IDF flag
                # to false unless the user manually patched the source.
                "acquire_beamformed":    int(parts[34]) if len(parts) > 34 else 0,
                # frame_type: 802.11 frame class string (DATA / MGMT / ACK /
                # CTRL / RSVD / "?"). Added in gen-3; "?" for older captures
                # or when the firmware could not parse the MAC header.
                "frame_type":            parts[35] if len(parts) > 35 else "?",
                # rx_seq: 802.11 MAC Sequence Control of the RECEIVED frame,
                # taken from wifi_csi_info_t.rx_seq. Added in gen-4. This is the
                # only field that lets a receiver detect its own frame loss -
                # gptimer_seq is TX-side only and is 0 on STA / passive
                # captures. -1 marks "column absent" (pre-gen-4 capture) so
                # that drop detection can distinguish it from a genuine 0.
                "rx_seq":                int(parts[36]) if len(parts) > 36 else -1,
                # timestamp_us: 64-bit esp_timer microseconds stamped in the
                # CSI callback. -1 on older captures; column 23
                # (real_timestamp, steady-clock seconds) carries the same
                # clock on all generations. Read by exact width: a 41-column
                # v2.1 row has no timestamp_us and index 37 is a v2.1 field.
                "timestamp_us":          (int(parts[37])
                                          if len(parts) in _COLS_WITH_TS
                                          else -1),
            })
            # Schema v2.1 tail. first_word_invalid and rx_estimate_vld are the
            # per-record validity flags the vendor documents for the MAC v3
            # parts. gain_force_val is the receive-gain state this build
            # applied, -1 when the gain runs free, which is the default.
            # die_temp_c10 is the on-die temperature in tenths of a degree,
            # sampled on a slow timer and not per record, so consecutive
            # records repeat a value.
            if len(parts) in _COLS_WITH_V21:
                b = 38 if len(parts) in _COLS_WITH_TS else 37
                record.update({
                    "first_word_invalid": int(parts[b]),
                    "rx_estimate_vld":    int(parts[b + 1]),
                    "gain_force_val":     int(parts[b + 2]),
                    "die_temp_c10":       int(parts[b + 3]),
                })
                record["schema_minor"] = 1
            else:
                record.update({
                    "first_word_invalid": -1,
                    "rx_estimate_vld":    -1,
                    "gain_force_val":     -1,
                    "die_temp_c10":       -1,
                })
                record["schema_minor"] = 0
            # Schema v2.2 tail: the per-record receive gain read through the
            # esp_csi_gain_ctrl component. Both read -1 when that component
            # is not linked, which is what separates a free-gain capture from
            # a locked one without the consumer having to guess.
            if len(parts) in _COLS_WITH_V22:
                b = 42
                record.update({
                    "agc_gain": int(parts[b]),
                    "fft_gain": int(parts[b + 1]),
                })
                record["schema_minor"] = 2
            else:
                record.update({"agc_gain": -1, "fft_gain": -1})
        except (ValueError, IndexError):
            # v2 fields malformed - keep v1 metadata + raw CSI but degrade to v1.
            record["schema_version"] = 1

    return record


def parse_file(path: Union[str, "os.PathLike"]) -> Iterator[Dict]:
    """Yield one record per CSI_DATA row in `path`. Comment / header lines skipped."""
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            rec = parse_line(line)
            if rec is not None:
                yield rec


def parse_file_to_dataframe(path: Union[str, "os.PathLike"]):
    """
    Parse a v1 / v2 CSV file into a pandas DataFrame (O-16).

    pandas is imported lazily inside this function so that `parse_csi.parse_line`
    and `parse_csi.parse_file` keep working without pandas installed. Only
    callers that explicitly want DataFrame output pay the import cost.

    The returned DataFrame has one row per CSI_DATA line. The `csi_raw`
    column is an object column whose entries are Python lists of int (one
    list per row); convert to a fixed-width matrix downstream if you need it.

    Returns an empty DataFrame if the file contains no CSI_DATA rows.

    >>> import os
    >>> df = parse_file_to_dataframe(os.path.join(os.path.dirname(__file__),
    ...                                            "example_csi.csv"))
    >>> df["schema_version"].unique().tolist()
    [1]
    >>> "csi_raw" in df.columns
    True
    """
    try:
        import pandas as pd
    except ImportError as exc:
        raise ImportError(
            "parse_file_to_dataframe requires pandas; install with "
            "`pip install pandas` (it is also listed in python_utils/requirements.txt)"
        ) from exc

    records = list(parse_file(path))
    if not records:
        return pd.DataFrame()
    return pd.DataFrame.from_records(records)


def parse_metadata(path: str) -> Dict[str, str]:
    """
    Scan the boot-time `# KEY=VALUE` comment header of a CSI capture file and
    return them as a dict.

    Firmware emits these lines once at session startup (see
    `_components/csi_component.h::_print_csi_csv_header()`). Examples:

        # CSI_TOOL_VERSION=2.0
        # CSI_FORMAT_VERSION=2
        # CHIP=esp32
        # IDF_VERSION=v6.0.1
        # RECORDING_SESSION_ID=startup-12345
        # CSI_FORCE_LLTF=0
        # CSI_ACQUIRE_BEAMFORMED=0
        # CSI_DUMP_ACK_FRAMES=1            (only when enabled)
        # CSI_BROADCAST_RATE_HZ=100        (active_ap + gptimer broadcast only)

    The function stops scanning at the first non-`#` non-blank line (i.e. the
    `type,role,mac,...` column header line, or the first CSI_DATA row), so it
    works on partially-flushed captures. Lines that don't match `# KEY=VALUE`
    are skipped silently. Multiple occurrences of the same key keep the last
    value (firmware never emits duplicates today, but this is defensive).

    Useful for packet-rate analysis: get the configured
    `BROADCAST_RATE_HZ` directly from the capture without an external metadata
    file.

    Returns an empty dict if the file has no boot comments (e.g. v1 captures
    or partial files where the header was cut off).
    """
    meta: Dict[str, str] = {}
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            for line in f:
                stripped = line.strip()
                if not stripped:
                    continue
                if not stripped.startswith("#"):
                    # Not a comment.
                    #
                    # Historically the scan stopped here, which returned an
                    # empty dict for any capture teed from `idf.py monitor`,
                    # because such a file starts with the monitor banner and
                    # boot-ROM output before the firmware's `#` header.
                    #
                    # It cannot stop at the first CSI row either: a capture
                    # that spans a device reboot begins with stale rows from
                    # the PREVIOUS session, and the current session's header
                    # appears after them. Observed 2026-07-29 on ESP32-C6,
                    # where one leftover frame preceded the boot header at
                    # line 274 and hid the whole block.
                    #
                    # So: never stop on data. Scan on, and let a new
                    # CSI_TOOL_VERSION reset the block, so what is returned is
                    # the header of the LAST boot in the file - the session the
                    # data actually belongs to.
                    continue
                # Strip the leading '#' and any whitespace, then look for
                # `KEY=VALUE` (firmware always uses '=', no spaces around it).
                body = stripped[1:].strip()
                if "=" not in body:
                    continue
                key, _, value = body.partition("=")
                key = key.strip()
                value = value.strip()
                if not key:
                    continue
                # The firmware emits CSI_TOOL_VERSION first in its boot block,
                # so seeing it again means a new session started; drop whatever
                # the previous session declared.
                if key == "CSI_TOOL_VERSION" and meta:
                    meta = {}
                meta[key] = value
    except OSError:
        return meta
    return meta


def amplitude_phase(csi_raw: List[int]) -> Tuple[List[float], List[float]]:
    """
    Convert a flat int8 array into matched (amplitude, phase) lists.

    The interleaving convention (which of byte[i] is real / imaginary) follows
    Hernandez 2020: byte[i*2]   = imaginary
                    byte[i*2+1] = real
    Phase is in radians.

    This is the RAW positional reading, kept for backward compatibility with
    the ESP32-CSI-Tool pipeline. It is NOT a physical interpretation: it
    misreads the 106-byte MAC v3 12-bit record (53 wrong values instead of
    26 tones) and carries no subcarrier indices. Consumers that want
    physical tones use lib/chip_aware_decoder.decode_record(), which
    resolves the verified decoding per record and reports an explicit
    unknown where none is established.
    """
    n_complex = len(csi_raw) // 2
    amplitudes: List[float] = []
    phases: List[float] = []
    for i in range(n_complex):
        im = csi_raw[i * 2]
        re = csi_raw[i * 2 + 1]
        amplitudes.append(sqrt(im * im + re * re))
        phases.append(atan2(im, re))
    return amplitudes, phases


# --------------------------------------------------------------------------
# Standalone demo (preserves the original Hernandez 2020 entry point so that
# anyone running `python parse_csi.py` keeps the v1 behaviour).
# --------------------------------------------------------------------------
if __name__ == "__main__":
    """
    This script file demonstrates how to transform raw CSI out from the ESP32 into CSI-amplitude and CSI-phase.
    """

    if len(sys.argv) > 1:
        FILE_NAME = sys.argv[1]
    else:
        FILE_NAME = "./example_csi.csv"

    # Explicit UTF-8 (matches parse_file above). Without it Python on Chinese
    # Windows defaults to GBK and trips on ESP32 boot-ROM byte sequences.
    f = open(FILE_NAME, "r", encoding="utf-8", errors="replace")

    # The demo routes every record through the chip-aware decoder so that
    # what it prints is the verified physical interpretation (or an explicit
    # unknown), never a raw positional guess (2026-09-15; audit item I12:
    # the previous demo read a 106-byte MAC v3 record as 53 int8 values,
    # the exact misreading the toolkit documents).
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    "lib"))
    from chip_aware_decoder import decode_record

    for j, l in enumerate(f.readlines()):
        record = parse_line(l)
        if record is None:
            continue

        dec = decode_record(record["csi_raw"],
                            record.get("subcarrier_layout_id",
                                       "STANDARD_HT20"),
                            record.get("chip_id"),
                            first_word_invalid=record.get(
                                "first_word_invalid"),
                            secondary_channel=record.get(
                                "secondary_channel"),
                            tool=record.get("firmware_version"))
        if dec["blocks"]:
            amplitudes = [(idx, abs(c))
                          for b in dec["blocks"] for idx, c in b["tones"]]
            phases = [(idx, atan2(c.imag, c.real))
                      for b in dec["blocks"] for idx, c in b["tones"]]
        else:
            amplitudes, phases = [], []

        print("-------------------")
        print("schema:           v{}.{}".format(record["schema_version"],
                                                record.get("schema_minor", 0)))
        if record["schema_version"] == 2:
            print("chip_id:          {}".format(record.get("chip_id", "?")))
            print("layout:           {}".format(record.get("subcarrier_layout_id", "?")))
            print("gptimer_seq:      {}".format(record.get("gptimer_seq", "?")))
            print("timestamp_us:     {}".format(record.get("timestamp_us", -1)))
            if record.get("schema_minor") == 1:
                print("first_word_inval: {}".format(record["first_word_invalid"]))
                print("rx_estimate_vld:  {}".format(record["rx_estimate_vld"]))
                print("gain_force_val:   {}".format(record["gain_force_val"]))
                print("die_temp_c10:     {}".format(record["die_temp_c10"]))
        print("decode status:    {} ({})".format(dec["status"], dec["note"]))
        print("csi_amplitude#{}:".format(j), amplitudes)
        print("csi_phase#{}:    ".format(j), phases)
        print("-------------------")
