"""
Host-side mirror of _components/chip_aware_decoder.h.

The firmware tags every captured frame with one of the layout id strings
listed below. This module maps that string to the correct subcarrier index
ordering so that downstream analysis sees a uniform "ascending subcarrier
index -> complex value" representation regardless of which chip and which
acquisition mode produced the bytes.

The string set is duplicated verbatim from the C header. If you add a new
layout there, add it here too.
"""

from __future__ import annotations

from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np


# ----------------------------------------------------------------------------
# Subcarrier orderings per layout.
#
# These ranges are best-effort initial mappings. Each chip's PHY documents
# its own ordering; corrections from hardware tests should land
# here. Where the upstream firmware bug means certain subcarriers carry
# garbage (#14271, #18118, #18493), the layout reflects that.
# ----------------------------------------------------------------------------
LAYOUTS = {
    # ESP32 / S2 / S3 / C3 default HT20: 64 subcarriers. The IDF CSI table
    # gives 0..31 then -32..-1 (FFT order) when the receiver has no
    # secondary channel and ascending order (natural) when it has one
    # above or below. The ramp fits confirm FFT order on all four parts
    # and natural order on the ESP32, S2 and S3 (no C3 capture carries a
    # secondary channel). One label cannot carry both, so this list is the
    # legacy positional convention of the no-secondary-channel case only;
    # callers that know the chip and the record's secondary_channel pass
    # both to decode()/decode_record(), which resolve the order through
    # format_registry.ht20_64_order(); decode_record() returns an explicit
    # unknown otherwise.
    "STANDARD_HT20":   list(range(0, 32)) + list(range(-32, 0)),

    # ESP32-C6 #14271: in HE acquisition mode the buffer comes back at 128
    # bytes (HT-LTF only, no L-LTF), the 64 HT subcarriers without the
    # leading L-LTF block. Hardware-verified natural order (ascending
    # subcarrier index, tab:phase) under every secondary-channel setting.
    # Corrected 2026-09-15: this entry
    # previously held the FFT ordering while its comment said natural
    # (audit item I13); the ramp fits establish natural.
    "C6_HT20_NO_LLTF": list(range(-32, 32)),

    # ESP32-C61 with acquire_csi_force_lltf=1 (#18118 workaround). A 106-byte
    # buffer from this layout is the MAC v3 12-bit L-LTF record and is decoded
    # by the dedicated path below, not by this int8 map; the map remains for
    # non-106-byte buffers only.
    "C61_FORCE_LLTF":  list(range(1, 27)) + list(range(-26, 0)),

    # ESP32-C5 on a 5 GHz channel (opt-in via
    # CONFIG_ESP32_CSI_TOOL_C5_BAND_MODE_5G_ONLY). The 106-byte case is the
    # MAC v3 12-bit L-LTF record, decoded by the dedicated path below.
    "C5_5GHZ":         list(range(0, 122)) + list(range(-122, 0)),

    # Wi-Fi 6 HE-LTF SU/MU mode at HT20. The 802.11ax spec defines 242
    # subcarriers for a 20 MHz HE-RU; the chip-level ordering is to be
    # verified empirically.
    "HE_LTF_SU":       list(range(0, 122)) + list(range(-122, 0)),
    "HE_LTF_MU":       list(range(0, 122)) + list(range(-122, 0)),

    # HT40: 128 subcarriers. Order verified against IDF Wi-Fi driver guide.
    "HT40":            list(range(0, 64)) + list(range(-64, 0)),

    "UNKNOWN":         None,
}

# All known string ids. Keep this in sync with chip_aware_layout_name() in
# the firmware header so that test_chip_aware_decoder.py can verify both
# directions.
KNOWN_LAYOUT_IDS: Tuple[str, ...] = tuple(LAYOUTS.keys())

# ----------------------------------------------------------------------------
# MAC v3 (ESP32-C5 / ESP32-C61) 12-bit L-LTF records.
#
# esp-idf issue #18982 (maintainer reply, 2026-08-26): with the driver default
# lltf_bit_mode = 0 the L-LTF CSI is delivered in 12-bit precision, four int8
# elements per CSI value. Each I/Q component is a little-endian 16-bit
# container whose bits [11:0] are a signed 12-bit value, and the 26 tones sit
# at every second subcarrier from -26 to +26, DC excluded. A 106-byte buffer
# is 53 containers: 52 pair into 26 complex tones and the trailing container
# is padding (observed identically zero). The component order within a pair
# follows the documented imaginary-first convention; the reply's example is
# ambiguous on this point, and amplitude and phase-ramp analyses are
# insensitive to it.
# ----------------------------------------------------------------------------
MAC_V3_12BIT_LAYOUTS = ("C61_FORCE_LLTF", "C5_5GHZ")
MAC_V3_12BIT_BYTES = 106
_MAC_V3_TONES = [k for k in range(-26, 27, 2) if k != 0]


def _decode_12bit(csi_raw: Sequence[int]) -> List[Tuple[int, complex]]:
    arr = np.asarray(csi_raw, dtype=np.int8).astype(np.uint8)
    containers = (arr[0::2].astype(np.uint16)
                  | (arr[1::2].astype(np.uint16) << 8))
    vals = (containers & 0x0FFF).astype(np.int16)
    vals[vals >= 0x0800] -= 0x1000
    # Real part first: the order of the maintainer reply on esp-idf issue
    # #18982, which the 114- and 234-byte records of the same transmitter
    # confirm (supplement, macv3_orientation.py).
    real = vals[0:52:2].astype(np.float32)
    imag = vals[1:52:2].astype(np.float32)
    cplx = real + 1j * imag
    return list(zip(_MAC_V3_TONES, cplx))


def should_drop(layout_id: str) -> bool:
    """
    Frames the firmware would drop. Only `UNKNOWN` is dropped;
    `C5_5GHZ` frames are kept (the user opted in via Kconfig and accepts
    the upstream #18493 risk). See `_components/chip_aware_decoder.h`.
    """
    return layout_id == "UNKNOWN"


def _int8_complex(csi_raw: Sequence[int]) -> "np.ndarray":
    """Interleaving convention follows Hernandez 2020:
        csi_raw[i * 2]     = imaginary part of value i
        csi_raw[i * 2 + 1] = real part of value i
    """
    arr = np.asarray(csi_raw, dtype=np.int8)
    n_complex = arr.size // 2
    imag = arr[0::2][:n_complex].astype(np.float32)
    real = arr[1::2][:n_complex].astype(np.float32)
    return real + 1j * imag


def decode(csi_raw: Sequence[int], layout_id: str,
           chip_id: str = None,
           secondary_channel: int = None) -> List[Tuple[int, complex]]:
    """
    Convert an int8 CSI buffer into [(subcarrier_index, complex), ...] sorted
    in the order the layout documents.

    Parameters
    ----------
    csi_raw : sequence of int
        The bracketed payload from CSI_DATA, parsed by parse_csi.parse_line.
    layout_id : str
        The 'subcarrier_layout_id' field of the v2 record (or 'STANDARD_HT20'
        for v1 records that pre-date the layout tag).
    chip_id : str, optional
        The record's chip_id column.
    secondary_channel : int, optional
        The record's rx_ctrl secondary_channel (v1 column 17; 0 none,
        1 above, 2 below). A 64-tone HT20 record resolves its order through
        format_registry.ht20_64_order(): on a pre-HE chip FFT order with
        no secondary channel and natural order with one above or below, on
        the C6 natural order under every setting. When the registry returns
        unknown (chip missing, or the setting missing on a pre-HE chip) the
        legacy positional list of the layout applies, which matches a
        pre-HE record received with no secondary channel.

    Returns
    -------
    list of (subcarrier_index, complex)
        Empty list when the layout is unknown or when the buffer is LONGER
        than the layout describes: the extra values are a structured record
        (or an unregistered format), and silently dropping them reproduces
        the misindexing this toolkit audits. Structured and unsupported
        records go through decode_record() instead.
    """
    # A 106-byte buffer is the MAC v3 12-bit L-LTF record; only the two
    # MAC v3 parts produce it, and the 12-bit reading is vendor-confirmed
    # (esp-idf #18982). Dispatch on the length, not on the label: a C5
    # capture on 2.4 GHz carries the same record under a different label
    # (audit item I14). This layout-level reading assumes the 12-bit mode
    # this firmware sets; decode_record() checks the record's tool column
    # and returns unsupported for a 106-byte record of another tool.
    if len(csi_raw) == MAC_V3_12BIT_BYTES:
        return _decode_12bit(csi_raw)

    layout = LAYOUTS.get(layout_id)
    if layout is None:
        return []

    cplx = _int8_complex(csi_raw)
    n_complex = cplx.size
    if n_complex == 0:
        return []

    if n_complex == 64:
        from format_registry import ht20_64_order
        label, order = ht20_64_order(chip_id, secondary_channel)
        if order is not None:
            return list(zip(order, cplx))
        # order unknown: fall through to the legacy positional list.

    if n_complex > len(layout):
        # Out-of-spec long buffer: never silently truncate (audit I15).
        return []
    return list(zip(layout[:n_complex], cplx[:n_complex]))


def decode_record(csi_raw: Sequence[int], layout_id: str,
                  chip_id: str = None, first_word_invalid: int = None,
                  secondary_channel: int = None, tool: str = None) -> dict:
    """
    Structured decode with an explicit verification status. This is the
    default entry point for consumers that want physical indices.

    The format is resolved from the payload size, the chip, and for the
    106-byte record the tool (format_registry.record_format), so a 384-byte
    HE-LTF record from a C5, C6, or C61 is reported unsupported instead of
    being read as the ESP32 HT40 two-block record, and a 106-byte record is
    read as 12-bit only when `tool`, the record's firmware_version column,
    names VeriCSI-ESP32, which sets that mode. A 128-byte record takes its
    tone order from the chip and the record's secondary_channel (v1 column
    17) through format_registry.ht20_64_order(); without the setting a pre-HE
    record is reported 'unverified-order'. When first_word_invalid is 1,
    the leading entries that the driver marks invalid (the first four
    payload bytes) are removed from the first block.

    Returns a dict:
      status   'ok' | 'unverified-order' | 'unsupported' | 'unknown-layout'
      mapping  the format_registry mapping level for this payload size
      blocks   list of {training, tones: [(index, complex), ...]} - one
               block for single-field records, two for the 384-byte
               L-LTF + HT-LTF record; empty when status is 'unsupported'
      raw      the positional int8-pair reading; empty for 12-bit records,
               where that pairing is not a valid reading of the bytes
      note     one-line explanation
    """
    out = _decode_record(csi_raw, layout_id, chip_id, secondary_channel,
                         tool)
    if first_word_invalid == 1 and out["blocks"]:
        n = 1 if len(csi_raw) == MAC_V3_12BIT_BYTES else 2
        out["blocks"][0]["tones"] = out["blocks"][0]["tones"][n:]
        out["note"] += (f"; first {n} entr{'y' if n == 1 else 'ies'} "
                        f"removed (first_word_invalid)")
    return out


def _decode_record(csi_raw: Sequence[int], layout_id: str,
                   chip_id: str = None, secondary_channel: int = None,
                   tool: str = None) -> dict:
    from format_registry import ht20_64_order, record_format

    nbytes = len(csi_raw)
    fmt = record_format(nbytes, chip_id, tool)
    out = dict(status="ok", mapping=fmt["mapping"], blocks=[],
               raw=list(_int8_complex(csi_raw)), note=fmt["note"])

    if fmt["mapping"] == "unsupported":
        out["status"] = "unsupported"
        return out

    if nbytes == MAC_V3_12BIT_BYTES:
        out["blocks"] = [dict(training=fmt["training"],
                              tones=_decode_12bit(csi_raw))]
        out["raw"] = []          # int8 pairing misreads a 12-bit record
        return out

    if nbytes == 384:
        # Two documented training-field blocks: 128 B L-LTF then 256 B
        # HT-LTF (vendor docs). The block split is documented; the per-block
        # tone order is not hardware-verified here, so the tones carry
        # positional indices and the status says so.
        cplx = _int8_complex(csi_raw)
        out["status"] = "unverified-order"
        out["blocks"] = [
            dict(training="L-LTF", tones=list(zip(range(64), cplx[:64]))),
            dict(training="HT-LTF", tones=list(zip(range(128), cplx[64:]))),
        ]
        return out

    if nbytes == 256:
        # Two documented training-field blocks of a pre-HE HT20 record:
        # 128 B L-LTF then 128 B HT-LTF (vendor docs), observed on the
        # ESP32. Positional indices, as for the 384-byte record.
        cplx = _int8_complex(csi_raw)
        out["status"] = "unverified-order"
        out["blocks"] = [
            dict(training="L-LTF", tones=list(zip(range(64), cplx[:64]))),
            dict(training="HT-LTF", tones=list(zip(range(64), cplx[64:]))),
        ]
        return out

    if nbytes == 128:
        label, order = ht20_64_order(chip_id, secondary_channel)
        cplx = _int8_complex(csi_raw)
        if order is None:
            out["status"] = "unverified-order"
            out["blocks"] = [dict(training=fmt["training"],
                                  tones=list(zip(range(64), cplx)))]
            out["note"] = ("64-tone HT-LTF record; order unknown without "
                           "chip_id and, on a pre-HE chip, secondary_channel "
                           "(FFT with none, natural with above or below; "
                           "natural on the C6)")
            return out
        out["blocks"] = [dict(training=fmt["training"],
                              tones=list(zip(order, cplx)))]
        out["note"] = f"64-tone HT-LTF, {label} order for {chip_id}"
        if secondary_channel is not None:
            out["note"] += f", secondary_channel {secondary_channel}"
        return out

    pairs = decode(csi_raw, layout_id, chip_id, secondary_channel)
    if not pairs:
        out["status"] = "unknown-layout"
        return out
    out["blocks"] = [dict(training=fmt["training"], tones=pairs)]
    return out


def amplitude(csi_decoded: Iterable[Tuple[int, complex]]) -> List[Tuple[int, float]]:
    """Return [(subcarrier_index, amplitude), ...] for each tuple."""
    return [(idx, abs(c)) for idx, c in csi_decoded]


def phase(csi_decoded: Iterable[Tuple[int, complex]]) -> List[Tuple[int, float]]:
    """Return [(subcarrier_index, phase_radians), ...] for each tuple."""
    import math
    return [(idx, math.atan2(c.imag, c.real)) for idx, c in csi_decoded]


def reorder_to_ascending(csi_decoded: Iterable[Tuple[int, complex]]
                         ) -> Tuple[List[int], List[complex]]:
    """
    Sort by subcarrier index in ascending order (most negative -> most
    positive), so that downstream Fourier-domain analysis sees a contiguous
    spectrum. Returns (indices, complex_values) as parallel lists.
    """
    pairs = sorted(csi_decoded, key=lambda p: p[0])
    if not pairs:
        return [], []
    indices, values = zip(*pairs)
    return list(indices), list(values)
