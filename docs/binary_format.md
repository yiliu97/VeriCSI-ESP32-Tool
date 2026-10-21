# Binary CSI Format (optional sink)

> **Status:** a first implementation exists behind the binary-sink Kconfig option but is unusable over console sinks (LF-to-CRLF translation corrupts frames); a redesign is pending. This document records the intended design.

## Why a binary format

ASCII CSV is the original (Hernandez 2020) format and remains the default for two good reasons: (a) every existing analysis pipeline can read it without modification, (b) `idf.py monitor | grep CSI_DATA > file.csv` keeps working as the simplest possible capture command. CSV will not be removed.

That said, ASCII overhead is significant. A 384-byte CSI buffer becomes a ~1.5 kB CSV row once each `int8` is rendered as `"%d "`. At 1000 packets/s the resulting bit rate exceeds the practical ceiling of every sink except USB-CDC, and even there the host-side parser spends most of its time tokenising digits.

Once the V7-final K5 packet-rate-precision benchmark is in the field, the headline result is likely to be "STA-AP timer is 5-10x lower jitter than ESP-NOW" - which immediately raises the question "how high can the rate go?". An optional binary format unlocks ~3-5x denser transmission and ~10-50x faster host-side parse without giving up ASCII compatibility.

## Design

### Stream layout

```
+-----------------+----------------+
| Stream header   | Frame records  |
| (once at start) | (one per CSI)  |
+-----------------+----------------+
```

The stream header is emitted once when the firmware starts, mirroring the `# CSI_TOOL_VERSION=2.0` comment lines in CSV.

### Stream header (16 bytes fixed + variable JSON)

| Offset | Size | Field | Notes |
|---:|---:|---|---|
| 0 | 4 | Magic | ASCII `CSI6` |
| 4 | 1 | Format version | `0x01` (binary-format version, independent of the CSV schema generation) |
| 5 | 1 | Flags | bit 0 = chip-aware decoding required (`subcarrier_layout_id` may vary across frames) |
| 6 | 2 | Reserved | zero |
| 8 | 4 | JSON metadata length | bytes |
| 12 | 4 | Reserved / future use | zero |
| 16 | N | JSON metadata | UTF-8, mirrors the v2 CSV `#`-comment header (CHIP, IDF_VERSION, AP_MAC, STA_MAC, CHANNEL, BANDWIDTH, recording_session_id, etc.) |

### Per-frame record

```
+----+-----+-------------+----+---------+--------+
| HL |  T  | Metadata HL | CL |   CSI   | CRC32  |
+----+-----+-------------+----+---------+--------+
   2 byte         HL byte        2 byte CL byte   4 byte
```

| Offset | Size | Field | Notes |
|---:|---:|---|---|
| 0 | 2 | `header_len` | length of the metadata block, little-endian |
| 2 | header_len | metadata | packed v2 fields (see below) |
| 2+HL | 2 | `csi_len` | length of the raw CSI payload, little-endian |
| 4+HL | csi_len | CSI payload | raw `int8` interleaved imag/real, identical to the bytes that go inside `[...]` in CSV |
| 4+HL+CL | 4 | CRC32 | over the entire record (HL through CSI payload), IEEE 802.3 polynomial |

### Metadata block (packed)

The v2 fields that appear in the CSV columns 0-24 plus 26-33, packed in a fixed layout. Strings (`mac`, `chip_id`, `firmware_version`, `subcarrier_layout_id`) become length-prefixed UTF-8. All multi-byte integers are little-endian.

```
struct __attribute__((packed)) frame_metadata_v21 {
    uint8_t  type_code;                /* 0 = CSI_DATA */
    uint8_t  role_code;                /* 0=STA, 1=AP, 2=PASSIVE */
    uint8_t  mac[6];
    int8_t   rssi;
    uint8_t  rate;
    uint8_t  sig_mode;
    uint8_t  mcs;
    uint8_t  bandwidth;
    uint8_t  smoothing;
    uint8_t  not_sounding;
    uint8_t  aggregation;
    uint8_t  stbc;
    uint8_t  fec_coding;
    uint8_t  sgi;
    int8_t   noise_floor;
    uint8_t  ampdu_cnt;
    uint8_t  channel;
    uint8_t  secondary_channel;
    uint32_t local_timestamp;
    uint8_t  ant;
    uint16_t sig_len;
    uint8_t  rx_state;
    uint8_t  real_time_set;
    double   real_timestamp;           /* IEEE 754 double for compatibility with v1 %.6f */
    uint16_t len;
    /* v2 fields begin */
    uint8_t  chip_id_len;              /* string length-prefixed; UTF-8 follows */
    /* uint8_t chip_id[chip_id_len]; */
    uint8_t  firmware_version_len;
    /* uint8_t firmware_version[firmware_version_len]; */
    uint16_t csi_buf_len;
    uint8_t  subcarrier_layout_id_len;
    /* uint8_t subcarrier_layout_id[subcarrier_layout_id_len]; */
    uint32_t gptimer_seq;
    uint8_t  he_ltf_present;
    uint8_t  he_stbc_mode;
    uint8_t  acquire_force_lltf;
};
```

Length-prefixed strings rather than fixed-width fields keep the format compact for typical chip ids (`esp32`, `esp32c6`) without losing the ability to extend later.

## Backward compatibility

The binary format is OPT-IN and OPT-OUT independently of CSV:

- `CONFIG_ESP32_CSI_TOOL_SINK_BINARY=y` (planned) routes a binary copy of every frame into a new sink alongside (or instead of) CSV.
- The CSV path is unchanged. Setting both `CONFIG_SEND_CSI_TO_SERIAL=y` and `CONFIG_ESP32_CSI_TOOL_SINK_BINARY=y` produces both formats over the same pipe with the binary frames clearly delimited by their `CSI6` magic, making the stream `grep CSI_DATA`-friendly for legacy tools.
- The standalone converter `python_utils/lib/csi_bin_to_csv.py` decodes a binary capture into a v2 CSV file that any v1 / v2 reader handles. It emits `?` for `frame_type` because the binary META record does not carry the frame class.

## Implementation notes

1. Add `_components/binary_sink.h` with the same `*_init` / `*_write(data, len)` / `*_flush` signature as the other sinks. The frame is written from the CSI consumer task (not the rx callback).
2. CRC32 should be computed incrementally over the metadata + payload to avoid a second buffer pass. ESP-IDF's `esp_rom_crc32_le` matches the IEEE 802.3 polynomial.
3. Keep `_components/csi_component.h` in charge of producing v1 CSV first; the binary serialiser is a separate function fed the same `csi_queue_item_t` so the two paths cannot diverge.
4. Round-trip a synthetic frame through `python_utils/lib/csi_bin_to_csv.py` and check parity with the v2 CSV emitted from the same `csi_queue_item_t`.

## Open questions

- Is bit-for-bit reproducibility of `real_timestamp` important enough to keep IEEE 754 double instead of e.g. fixed-point microseconds? Some downstream tools cast it to float anyway. Decision deferred to the redesign.
- Should we ship `python_utils/lib/csi_bin_to_csv.py` as a Python entry point or keep it as a script? Probably script-style to match the rest of `python_utils/lib/`.

## Out of scope for the current release

The current release ships ASCII CSV only. This document exists so that downstream tooling (CSIKit, csiread, custom forks) can plan against the binary format without committing to it. Contributions prototyped against the design above are welcome.
