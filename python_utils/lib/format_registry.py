"""
Single authoritative registry for CSV schema widths and CSI record formats.

Motivation (third-party audit 2026-09-14, items I01/I07/I12-I15): several
analysis scripts carried their own copy of the column-width dispatch, and the
copies diverged; the 44-column v2.2 layout fell through to the wrapping
32-bit clock in more than one of them. Field positions, record-format
semantics, and mapping verification levels now live here, and every consumer
resolves them through this module. parse_csi.parse_line remains the full
parser; the accessors here serve the fast paths that split a raw line and
read a few fields without building a record dict.

Three lengths describe every record, and they are different quantities:

  driver_len   what the Wi-Fi driver reported (v1 column 24, "len")
  copied_len   what the firmware copied into its queue item (column 28,
               "csi_buf_len"; the copy is capped at CSI_MAX_BUF_LEN = 384,
               so copied_len < driver_len marks a truncated record)
  exported     the number of int8 values actually present in the bracketed
               payload (an LLTF-only build trims the export to 128 bytes,
               so exported < copied_len marks an intentional selection)

None of the three substitutes for another, and no consumer may treat
driver_len as the payload size.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple

# ---------------------------------------------------------------------------
# Schema widths. Tail layout after the bracketed payload, verified against
# the firmware writer (csi_component.h, csi_format_record):
#   v2 base (12): chip_id, firmware_version, csi_buf_len, layout_id,
#                 gptimer_seq, he_ltf_present, he_stbc_mode,
#                 acquire_force_lltf, acquire_beamformed, frame_type,
#                 rx_seq, timestamp_us
#   v2.1 (+4):    first_word_invalid, rx_estimate_vld, gain_force_val,
#                 die_temp_c10
#   v2.2 (+2):    agc_gain, fft_gain
# Widths 37 and 41 are the corresponding layouts without timestamp_us.
# Offsets are FROM THE END of the naive comma-split, so they hold for every
# width without re-deriving absolute positions.
# ---------------------------------------------------------------------------
SCHEMAS: Dict[int, Dict[str, Optional[int]]] = {
    37: dict(name="v2.0-nots", ts=None, seq=-1, layout=-8,  buf_len=-9),
    38: dict(name="v2.0",      ts=-1,   seq=-2, layout=-9,  buf_len=-10),
    41: dict(name="v2.1-nots", ts=None, seq=-5, layout=-12, buf_len=-13),
    42: dict(name="v2.1",      ts=-5,   seq=-6, layout=-13, buf_len=-14),
    44: dict(name="v2.2",      ts=-7,   seq=-8, layout=-15, buf_len=-16),
}
WIDTHS: Tuple[int, ...] = tuple(sorted(SCHEMAS))
V1_LEN_IDX = 24        # driver-reported length, v1 position
V1_T32_IDX = 18        # wrapping 32-bit rx_ctrl timestamp, v1 position
V1_SEC_IDX = 17        # rx_ctrl secondary_channel, v1 position


def timestamp_us(parts: Sequence) -> Optional[int]:
    """64-bit device timestamp of a naive comma-split row, or None when the
    layout does not carry one. Never falls back to the wrapping 32-bit
    column: a caller that wants that fallback must take it explicitly and
    own the wrap risk."""
    s = SCHEMAS.get(len(parts))
    if s is None or s["ts"] is None:
        return None
    try:
        return int(parts[s["ts"]])
    except (ValueError, IndexError):
        return None


def rx_seq(parts: Sequence) -> Optional[int]:
    s = SCHEMAS.get(len(parts))
    if s is None:
        return None
    try:
        return int(parts[s["seq"]])
    except (ValueError, IndexError):
        return None


def secondary_channel(parts: Sequence) -> Optional[int]:
    """Receiver secondary-channel setting of a naive comma-split row
    (rx_ctrl secondary_channel, v1 column 17): 0 none, 1 above, 2 below.
    None when the field is absent or does not parse, as on a line that
    a serial framing fault joined to the start of another record."""
    try:
        return int(parts[V1_SEC_IDX])
    except (ValueError, IndexError):
        return None


def layout_id(parts: Sequence) -> Optional[str]:
    s = SCHEMAS.get(len(parts))
    if s is None:
        return None
    try:
        v = parts[s["layout"]]
        return v.decode("ascii", "replace") if isinstance(v, bytes) else str(v)
    except IndexError:
        return None


def lengths(parts: Sequence, exported_values: Optional[int] = None) -> Dict:
    """The three-length contract for one row. `exported_values` is the count
    of int8 values in the bracketed payload when the caller has parsed it."""
    s = SCHEMAS.get(len(parts))
    out: Dict = dict(driver_len=None, copied_len=None,
                     exported=exported_values, truncated=None, selected=None)
    try:
        out["driver_len"] = int(parts[V1_LEN_IDX])
    except (ValueError, IndexError):
        pass
    if s is not None:
        try:
            out["copied_len"] = int(parts[s["buf_len"]])
        except (ValueError, IndexError):
            pass
    if out["driver_len"] is not None and out["copied_len"] is not None:
        out["truncated"] = out["copied_len"] < out["driver_len"]
    if out["copied_len"] is not None and exported_values is not None:
        out["selected"] = exported_values < out["copied_len"]
    return out


# first_word_invalid, offset from the end of the naive split (v2.1 tail).
_FWI_OFFSET = {41: -4, 42: -4, 44: -6}


def leading_invalid_entries(parts: Sequence) -> Optional[int]:
    """Number of leading payload entries the driver marks unusable.

    The driver's first_word_invalid flag states that the first four payload
    bytes are invalid. Over the released .csv captures it is set on all
    129,932 ESP32 records that carry it and on none of the 1,984,176 records
    of the other six SoCs (field_zero_rates.py); the host-logged .raw
    captures of the ESP32 give 781,937 of 781,938. Four bytes are two complex entries of an int8-pair record and one
    tone of the 106 B MAC v3 record (four bytes per 12-bit tone). Returns 2
    (or 1 for the 106 B record) when the flag is set, 0 when it is clear,
    and None when the schema carries no flag (v1 and v2.0 rows) or the field
    does not parse."""
    off = _FWI_OFFSET.get(len(parts))
    if off is None:
        return None
    try:
        flag = int(parts[off])
    except (ValueError, IndexError):
        return None
    if flag == 0:
        return 0
    if flag != 1:
        return None
    try:
        copied = int(parts[SCHEMAS[len(parts)]["buf_len"]])
    except (ValueError, IndexError):
        copied = None
    return 1 if copied == 106 else 2


# ---------------------------------------------------------------------------
# Tone orderings. "natural" is ascending subcarrier index; "fft" is
# 0..N/2-1 then -N/2..-1. On the four pre-HE parts (ESP32, S2, S3, C3) the
# order of a 64-entry (128-byte) record follows the receiver's
# secondary-channel setting (rx_ctrl secondary_channel, v1 column 17).
# The CSI table of the ESP-IDF v6.0.1 Wi-Fi driver guide
# (docs/en/api-guides/wifi-driver/wifi-vendor-features.rst, sub-carrier
# index row) gives "0~31, -32~-1" for secondary channel none, the FFT
# order, "0~63" for below and "-64~-1" for above. The last two are
# ascending indices of the 40 MHz pair, the natural order of the 20 MHz
# primary channel. On the ESP32-C6 the order is natural under every
# setting. Ramp-fit survey of the bench-transmitter records (supplement,
# phase_table.py; groups of one capture and one setting with at least 100
# records): all 33 pre-HE groups with no secondary channel prefer FFT order
# (ESP32 7, S2 7, S3 6, C3 13), all 8 with one prefer natural order (ESP32
# 4 above, S2 3 below, S3 1 below), and all 16 C6 groups (no secondary
# channel) prefer natural order, as does the C6 capture with secondary
# channel below (acquire_e25/F3_R3_rx.csv). No C3 capture carries a
# secondary channel, so the C3 natural case rests on the vendor table.
# ht20_64_order() therefore takes the chip and the setting.
# ---------------------------------------------------------------------------
NATURAL_64: List[int] = list(range(-32, 32))
FFT_64: List[int] = list(range(0, 32)) + list(range(-32, 0))

_CHIP_ALIASES = {
    "esp32": "esp32", "esp32s2": "esp32s2", "esp32s3": "esp32s3",
    "esp32c3": "esp32c3", "esp32c5": "esp32c5", "esp32c6": "esp32c6",
    "esp32c61": "esp32c61",
}

# Pre-HE parts: order by secondary-channel setting (0 none, 1 above,
# 2 below). ESP32-C6: one order under every setting.
HT20_64_ORDER_PRE_HE = {
    0: ("fft", FFT_64),
    1: ("natural", NATURAL_64),
    2: ("natural", NATURAL_64),
}
HT20_64_ORDER_C6 = ("natural", NATURAL_64)


def normalize_chip(chip_id: Optional[str]) -> Optional[str]:
    if not chip_id:
        return None
    return _CHIP_ALIASES.get(str(chip_id).strip().lower())


def ht20_64_order(chip_id: Optional[str],
                  secondary_channel: Optional[int] = None):
    """(label, index list) for a 64-entry HT20 record on `chip_id` received
    with the given secondary-channel setting (0 none, 1 above, 2 below;
    see secondary_channel()). A pre-HE chip gives ("fft", FFT_64) for 0 and
    ("natural", NATURAL_64) for 1 or 2; the ESP32-C6 gives ("natural",
    NATURAL_64) for any setting. Returns ("unknown", None) when the chip is
    missing or has no 64-entry record, and on a pre-HE chip when the
    setting is missing or outside 0 to 2. An explicit unknown is the
    correct output: guessing an ordering produces exactly the silent
    misindexing this toolkit audits."""
    c = normalize_chip(chip_id)
    if c == "esp32c6":
        return HT20_64_ORDER_C6
    if c not in PRE_HE:
        return "unknown", None
    try:
        sec = int(secondary_channel)
    except (TypeError, ValueError):
        return "unknown", None
    return HT20_64_ORDER_PRE_HE.get(sec, ("unknown", None))


# ---------------------------------------------------------------------------
# Record formats: (exported payload bytes, chip) -> interpretation and its
# verification level. This is the registry behind the paper's format table;
# the table may only claim what this registry marks verified. The key is the
# pair: 384 B is the two-block HT40 record on the ESP32 and an HE-LTF record
# cut by the 384 B queue-item copy on the C5, C6, and C61, so the length
# alone does not name a format.
#   mapping: "confirmed"    vendor-confirmed (106 B, esp-idf issue #18982),
#                           for records this tool wrote (see below)
#            "fingerprint"  established by DC/null fingerprints + ramp fits
#            "per-chip"     order resolved from the chip and, on the
#                           pre-HE parts, the secondary-channel setting
#                           via ht20_64_order()
#            "documented"   block structure from vendor docs, tone order
#                           not hardware-verified here
#            "unsupported"  no verified interpretation; raw bytes only
# Chip sets list the parts on which the length is registered with that
# meaning; any other (length, chip) pair resolves to an explicit
# unsupported entry.
# ---------------------------------------------------------------------------
PRE_HE: Tuple[str, ...] = ("esp32", "esp32s2", "esp32s3", "esp32c3")
HE_ERA: Tuple[str, ...] = ("esp32c5", "esp32c6", "esp32c61")
MAC_V3: Tuple[str, ...] = ("esp32c5", "esp32c61")

RECORD_FORMATS: Dict[int, Tuple[Dict, ...]] = {
    106: (dict(chips=MAC_V3, training="L-LTF (MAC v3)",
               precision="12-bit LE containers", entries=26,
               mapping="confirmed", needs_tool=True,
               note="26 tones at every 2nd subcarrier -26..+26, no DC; "
                    "two containers per tone, real part first; "
                    "53rd container is zero pad"),),
    114: (dict(chips=("esp32c5",),
               training="HT-LTF or VHT-LTF (C5 HT20, VHT20)",
               precision="int8 pairs", entries=57, mapping="fingerprint",
               note="natural -28..+28"),),
    234: (dict(chips=("esp32c5",), training="HT-LTF (C5 HT40)",
               precision="int8 pairs", entries=117, mapping="fingerprint",
               note="natural -58..+58"),),
    128: (dict(chips=PRE_HE + ("esp32c6",),
               training="HT-LTF (64-tone HT20)", precision="int8 pairs",
               entries=64, mapping="per-chip",
               note="pre-HE parts: FFT order with no secondary channel, "
                    "natural with one above or below (ESP-IDF CSI table, "
                    "ramp-fit survey); C6: natural under every setting"),),
    256: (dict(chips=PRE_HE, training="L-LTF + HT-LTF (pre-HE HT20)",
               precision="int8 pairs", entries=128, mapping="documented",
               note="two training-field blocks: 128 B L-LTF then 128 B "
                    "HT-LTF (vendor docs); observed on the ESP32; per-block "
                    "tone order not hardware-verified here; do not "
                    "flatten"),),
    384: (dict(chips=("esp32",), training="L-LTF + HT-LTF (ESP32 HT40)",
               precision="int8 pairs", entries=192, mapping="documented",
               note="two training-field blocks: 128 B L-LTF then 256 B "
                    "HT-LTF (vendor docs); per-block tone order not "
                    "hardware-verified here; do not flatten"),
          dict(chips=HE_ERA, training="HE-LTF (truncated)",
               precision="int8 pairs", entries=None, mapping="unsupported",
               note="HE-LTF record cut at the 384 B queue-item copy (the "
                    "driver declares 490 B on the C5 and 512 B on the C6); "
                    "no tone map established; raw bytes only")),
    512: (dict(chips=HE_ERA, training="HE-LTF (declared)",
               precision="int8 pairs", entries=None, mapping="unsupported",
               note="driver-declared 512 B; the firmware copy is capped at "
                    "384 B, so no capture carries a full-length export or "
                    "validates a tone map"),),
}


# ---------------------------------------------------------------------------
# The ten receive-control fields of the NULL rule and the frame formats that
# give them a source. On the pre-HE parts nine of them come from the HT-SIG
# field (ESP-IDF v6.0.1 esp_wifi_types_native.h: mcs "If is HT(11n)
# packet", cwb, smoothing, not_sounding, aggregation, stbc, fec_coding
# "for 11n packets", sgi, ampdu_cnt), so a non-HT record (sig_mode 0) reads
# 0 in them without a source; the antenna index applies to every frame.
# The interface of the HE-era parts populates none of the ten, and the
# firmware writes -1 there. Parsing stays positional (parse_csi returns the
# values as written); with_missing() is the opt-in reading for numeric code
# that must not average the -1 marker or a zero without a source.
# ---------------------------------------------------------------------------
RX_FIELDS: Tuple[str, ...] = ("mcs", "bandwidth", "smoothing",
                              "not_sounding", "aggregation", "stbc",
                              "fec_coding", "sgi", "ampdu_cnt", "ant")
HT_SIG_FIELDS: Tuple[str, ...] = RX_FIELDS[:-1]


def field_has_source(field: str, sig_mode) -> Optional[bool]:
    """Whether a frame of this sig_mode gives one of the ten fields a
    source: True for the antenna index and for the HT-SIG fields of an HT
    or VHT record (sig_mode 1 or 3), False for the HT-SIG fields of a
    non-HT record (sig_mode 0), None when sig_mode is missing or names
    another format (4, HE, where the fields read -1)."""
    if field not in RX_FIELDS:
        raise KeyError(field)
    if field == "ant":
        return True
    if sig_mode in (1, 3):
        return True
    if sig_mode == 0:
        return False
    return None


def with_missing(record: Dict) -> Dict:
    """Copy of a parse_csi record in which each of the ten fields reads None
    where the record carries -1 (not populated by the interface) or where
    its frame format gives it no source (field_has_source() is False)."""
    out = dict(record)
    for f in RX_FIELDS:
        if f in out and (out[f] == -1 or
                         field_has_source(f, out.get("sig_mode")) is False):
            out[f] = None
    return out


# ---------------------------------------------------------------------------
# DIAG frames. The firmware emits a periodic "# DIAG,..." heartbeat and,
# under the termination protocol, one terminal "# DIAG_END,...,drained=".
# Fields are name=value pairs; consumers read them BY NAME, never by
# position, so appended fields stay compatible. Only a DIAG_END with
# drained=1 certifies that the capture tail is complete.
# ---------------------------------------------------------------------------
import re as _re

_DIAG_HEAD = _re.compile(rb"^# (DIAG|DIAG_END),")
_DIAG_FIELD = _re.compile(rb"([a-z_]+)=(-?\d+)")


def parse_diag(line) -> Optional[Dict]:
    """Parse one DIAG / DIAG_END line (bytes or str) into a dict of int
    fields plus 'terminal' (bool). Returns None for other lines."""
    if isinstance(line, str):
        line = line.encode("utf-8", "replace")
    m = _DIAG_HEAD.match(line)
    if m is None:
        return None
    out: Dict = {"terminal": m.group(1) == b"DIAG_END"}
    for k, v in _DIAG_FIELD.findall(line):
        out[k.decode("ascii")] = int(v)
    return out


def _unsupported(note: str) -> Dict:
    return dict(chips=(), training="?", precision="int8 pairs", entries=None,
                mapping="unsupported", note=note)


# ---------------------------------------------------------------------------
# Provenance of the 106 B reading. The MAC v3 parts deliver the L-LTF record
# in a 12-bit or an 8-bit precision mode (lltf_bit_mode of
# wifi_csi_acquire_config_t), and both modes give 106 bytes, so the length
# and the chip do not name the precision. VeriCSI-ESP32 sets the 12-bit mode
# in its source (csi_component.h, cfg.lltf_bit_mode = 0; no build option
# changes it), and every record names the tool in column 27
# (firmware_version: "VeriCSI-ESP32", with a ".N" schema suffix on builds
# before 2026-08-19, and "v2.0.0" on the earliest released captures).
# record_format() returns the confirmed 12-bit entry for a 106 B record only
# when that column names this tool, and an explicit unsupported entry
# otherwise.
# ---------------------------------------------------------------------------
TOOL_COLUMN = 27
_THIS_TOOL = _re.compile(r"VeriCSI-ESP32(\.\d+)?|v2\.0\.0")


def written_by_this_tool(tool) -> bool:
    """True when a firmware_version value names VeriCSI-ESP32."""
    if tool is None:
        return False
    if isinstance(tool, bytes):
        tool = tool.decode("ascii", "replace")
    return _THIS_TOOL.fullmatch(str(tool).strip()) is not None


def record_tool(parts: Sequence) -> Optional[str]:
    """firmware_version of a naive comma-split row, or None when the schema
    carries no such column (v1 rows)."""
    if len(parts) not in SCHEMAS:
        return None
    v = parts[TOOL_COLUMN]
    return (v.decode("ascii", "replace") if isinstance(v, bytes)
            else str(v)).strip()


def record_format(exported_bytes: int,
                  chip_id: Optional[str] = None,
                  tool: Optional[str] = None) -> Dict:
    """Format entry for an exported payload size on a chip, or an explicit
    unsupported entry when the pair is not registered.

    Without a chip the lookup succeeds only where the length names a single
    format; 384 B names two (ESP32 HT40, truncated HE-LTF) and returns the
    explicit unsupported entry until the chip is given. An entry marked
    needs_tool (the 106 B 12-bit reading) also requires `tool`, the
    record's firmware_version column, to name this tool. Returns a copy."""
    variants = RECORD_FORMATS.get(exported_bytes, ())
    found = None
    if not chip_id:
        if len(variants) == 1:
            found = variants[0]
        elif variants:
            names = ", ".join(v["training"] for v in variants)
            return _unsupported(
                f"{exported_bytes} exported bytes name more than one format "
                f"({names}); the chip identifier decides")
        else:
            return _unsupported(
                f"no registered format for {exported_bytes} exported bytes")
    else:
        chip = normalize_chip(chip_id)
        for v in variants:
            if chip in v["chips"]:
                found = v
                break
        if found is None:
            return _unsupported(
                f"no registered format for {exported_bytes} exported bytes "
                f"on {chip_id}")
    if found.get("needs_tool") and not written_by_this_tool(tool):
        return _unsupported(
            f"{exported_bytes} B on a MAC v3 part: the 12-bit and 8-bit "
            f"L-LTF modes give the same length, and the 12-bit reading is "
            f"established for records of VeriCSI-ESP32, which sets the "
            f"12-bit mode; tool column {tool!r} does not name it")
    return dict(found)
