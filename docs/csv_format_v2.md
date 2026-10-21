# CSV Output Format v2

VeriCSI-ESP32 emits CSV that is a strict super-set of the Hernandez 2020 v1 schema. **The `CSI_DATA` column stays at zero-indexed position 25**, matching v1 exactly, so position-indexing tools (Hernandez 2020's `serial_plot_csi_live.py` v1 used `all_data[25]`; this tool ships `csi_live.py` which uses `parse_csi.parse_line`) keep working without modification. The 18 v2 columns are **appended after** `CSI_DATA` (positions 26-43).

**Schema version vs. column count**: the current release (schema v2.2) emits 44 columns. The tool name is `VeriCSI-ESP32`, fixed across generations; ESP32 names the hardware family. The schema generation is determined by the column count, never by the name. Captures from firmware built before 2026-08-19 carry a `.N` schema-minor suffix after the name in the `firmware_version` column; current firmware writes the plain name. Each generation appended columns at the end, so every earlier consumer keeps working:

| Generation | Columns | Appended |
|---|---:|---|
| v1 (Hernandez 2020) | 26 | — |
| early v2 builds | 34 to 37 | chip_id .. rx_seq, added incrementally |
| v2.0 | 38 | `timestamp_us` |
| v2.1 | 42 | `first_word_invalid`, `rx_estimate_vld`, `gain_force_val`, `die_temp_c10` |
| **v2.2 (current)** | **44** | `agc_gain`, `fft_gain` |

`python_utils/parse_csi.py` recognises all of them (`NUM_COLS = 44`, legacy widths 34, 35, 36, 37, 38, 41, 42); older captures parse with the missing fields substituted as `-1` (or `"?"` for `frame_type`), so "column absent" stays distinguishable from a measured zero. The boot banner prints `CSI_FORMAT_VERSION=2` for every v2.x generation.

Existing scripts (CSIKit, csiread, the original `parse_csi.py` plus the Hernandez 2020 helpers `serial_append_time.py` / `serial_measure_rate.py` now under `python_utils/lib/`) parse v2 output without modification. the live tool `csi_live.py` reads via `parse_csi.parse_line` rather than position-indexing.

This document is the authoritative schema reference.

## Header (printed once at start-up)

The firmware prints a short metadata header before the CSV column header. Each line begins with `#` so that downstream parsers can either consume the metadata or skip it.

```
# CSI_TOOL_VERSION=2.0
# CSI_FORMAT_VERSION=2
# CHIP=esp32c6
# IDF_VERSION=v6.0.1
# RECORDING_START_UTC=2026-08-01T12:34:56Z
# AP_MAC=AA:BB:CC:DD:EE:FF
# STA_MAC=11:22:33:44:55:66
# CHANNEL=6
# BANDWIDTH=HT20
# CSI_ACQUIRE_LEGACY=1
# CSI_ACQUIRE_HT20=1
# CSI_ACQUIRE_HE_SU=1
# CSI_FORCE_LLTF=0
# CSI_ACQUIRE_BEAMFORMED=0
# CSI_DUMP_ACK_FRAMES=1
# BROADCAST_RATE_HZ=100
# RECORDING_SESSION_ID=20260801-123456-c6-001
```

## Column header (v2 super-set)

```
type,role,mac,rssi,rate,sig_mode,mcs,bandwidth,smoothing,not_sounding,
aggregation,stbc,fec_coding,sgi,noise_floor,ampdu_cnt,channel,secondary_channel,
local_timestamp,ant,sig_len,rx_state,real_time_set,real_timestamp,len,
CSI_DATA,
chip_id,firmware_version,csi_buf_len,subcarrier_layout_id,gptimer_seq,
he_ltf_present,he_stbc_mode,acquire_force_lltf,acquire_beamformed,frame_type,rx_seq,
timestamp_us,
first_word_invalid,rx_estimate_vld,gain_force_val,die_temp_c10,
agc_gain,fft_gain
```

| Position | Name | Origin | Notes |
|---:|---|---|---|
| 0 | type | v1 | Always literal `CSI_DATA` |
| 1 | role | v1 | `STA` / `AP` / `PASSIVE` |
| 2-24 | (v1 metadata) | v1 | Bit-for-bit identical to Hernandez 2020 |
| **25** | **CSI_DATA** | **v1** | **Bracketed `int8` IQ pairs - position preserved for `all_data[25]` indexing** |
| 26 | chip_id | v2 | `CONFIG_IDF_TARGET` string (e.g. `esp32c6`) |
| 27 | firmware_version | v2 | fixed tool name `VeriCSI-ESP32`; older captures carry a `.N` schema-minor suffix here, and the schema generation is read from the column count |
| 28 | csi_buf_len | v2 | **Copied** payload length in bytes: `min(data->len, CSI_MAX_BUF_LEN=384)`. Three lengths describe a record and none substitutes for another: the driver-reported length (v1 column 24, `len`), this copied length, and the exported value count (the bracketed array itself). `csi_buf_len < len` marks a truncated record; an array shorter than `csi_buf_len` marks an intentional selection (LLTF-only builds trim to 128 B). Corrected 2026-09-15: this row previously called the field "actual `data->len`", which conflated the first two lengths (audit item I01/I02). |
| 29 | subcarrier_layout_id | v2 | One of `STANDARD_HT20` / `C6_HT20_NO_LLTF` / `C61_FORCE_LLTF` / `C5_5GHZ` / `HE_LTF_SU` / `HE_LTF_MU` / `HT40` / `UNKNOWN` (see `_components/chip_aware_decoder.h`) |
| 30 | gptimer_seq | v2 | Broadcast sequence (active_ap timer mode); 0 in STA / passive |
| 31 | he_ltf_present | v2 | 0/1 |
| 32 | he_stbc_mode | v2 | 0/1/2 (mirror of `acquire_csi_he_stbc_mode`) |
| 33 | acquire_force_lltf | v2 | 0/1 |
| 34 | acquire_beamformed | v2 | 0/1, **session-level**: replicated per row from `CONFIG_ESP32_CSI_TOOL_ACQUIRE_BEAMFORMED`. **0 = no beamformed frames in this capture (driver-level filter); 1 = capture may contain a mix**. ESP-IDF v6.0.1 does not expose a per-frame beamformed bit, so per-frame discrimination is not possible from this column - see `docs/known_issues.md` for the sensing-task implications |
| 35 | frame_type | v2 | Short string: `DATA` / `MGMT` / `ACK` / `CTRL` / `RSVD` / `?`. Decoded **per frame** from the first byte of the 802.11 MAC header (Frame Control: type bits + ACK subtype). `?` means the firmware could not read `wifi_csi_info_t.hdr` (rare). When `CONFIG_ESP32_CSI_TOOL_DUMP_ACK_FRAMES=y` the capture contains `ACK` rows in addition to `DATA`/`MGMT`; downstream tooling can subset with `df = df[df.frame_type == "DATA"]`. Note: the `csi_bin_to_csv.py` converter emits `?` because the binary META_FORMAT does not carry the frame class - this column is CSV-only |

## Data line

A v2 data line begins with the literal token `CSI_DATA,`, places the bracketed CSI payload at position 25 (matching v1), and appends 18 v2 columns:

```
CSI_DATA,STA,AA:BB:..,-73,11,1,0,1,1,1,0,0,0,0,-93,0,1,1,80272146,0,101,0,0,80.363225,256,[3 -1 5 ...],esp32c6,VeriCSI-ESP32,256,STANDARD_HT20,12345,1,2,0,0,DATA,2417,80363225,0,1,-1,325,63,-35
```

### PHY metadata availability differs by SoC — `-1` means "not reported"

`wifi_pkt_rx_ctrl_t` is not one struct. ESP-IDF v6.0.1 typedefs it to
`esp_wifi_rxctrl_t` on HE-capable targets and to a different struct on the
rest (`components/esp_wifi/include/local/esp_wifi_types_native.h:33-87`). The
two layouts share only some fields, so several v1 columns cannot be populated
on Wi-Fi 6 chips.

All PHY metadata now goes through `_components/rxctrl_compat.h`, which returns
**`-1` for a field the SoC genuinely cannot report** — never `0`, so that
"unsupported on this chip" stays distinguishable from a real measurement of
zero.

| Column | ESP32 / S2 / S3 / C3 | C5 / C6 / C61 (Wi-Fi 6) |
|---|---|---|
| `rssi`, `rate`, `noise_floor`, `channel`, `local_timestamp`, `sig_len`, `rx_state` | reported | reported |
| `secondary_channel` | reported | reported (field is spelled `second` in the HE struct) |
| `sig_mode` | reported (0 non-HT, 1 HT, 3 VHT) | **reconstructed** from `cur_bb_format`, plus **`4` = HE (11ax)** |
| `mcs`, `bandwidth`, `smoothing`, `not_sounding`, `aggregation`, `stbc`, `fec_coding`, `sgi`, `ampdu_cnt`, `ant` | reported | **`-1`** — absent from the HE struct |

The `sig_mode = 4` value is a v2 extension. On HE targets the underlying
`cur_bb_format` distinguishes HE-SU / HE-MU / HE-ER-SU / HE-TB; the CSV
collapses all four to `4`. Use `subcarrier_layout_id` and `he_ltf_present`
when the finer distinction matters.

### `rx_seq` (column 36)

The 802.11 MAC Sequence Control of the **received** frame, taken from
`wifi_csi_info_t.rx_seq`. The field is unconditional in the ESP-IDF v6 CSI
struct, so it is populated identically on all seven supported SoCs.

### `timestamp_us` (column 37)

The 64-bit `esp_timer` microsecond count stamped inside the CSI callback,
exported as a plain integer. Three timestamp columns now coexist:

| Column | Field | Clock | Wraps |
|---:|---|---|---|
| 18 | `local_timestamp` | Wi-Fi driver `rx_ctrl.timestamp` (v1-compat) | at 2^32 us (~71.6 min) |
| 23 | `real_timestamp` | `std::chrono::steady_clock` seconds (v1-compat, 6 decimals) | no |
| 37 | `timestamp_us` | `esp_timer` microseconds, integer | no |

Columns 23 and 37 read the same underlying 64-bit clock; column 37 is the
integer form and the recommended field for timing analysis. Column 18 keeps
the v1 semantics for parser compatibility and **must not** be used for
captures longer than 71 minutes of device uptime.

This is the only field that lets a **receiver** detect its own frame loss.
`gptimer_seq` (column 30) cannot do this: it is generated by the AP's transmit
task inside the AP's own firmware image and is identically `0` in every
`active_sta` and `passive` capture.

Two things a consumer must handle:

* **Rollover.** The 802.11 sequence number is 12 bits (0–4095) inside a 16-bit
  Sequence Control field. Unwrap modulo the observed range before differencing.
* **Per-transmitter counters.** Each transmitter maintains its own sequence
  counter, so gaps must be counted **per source MAC**; pooling frames from two
  transmitters fabricates enormous phantom gaps.

### Schema v2.1 columns (38-41)

Appended after `timestamp_us`, at the end like every generation before, so v1
parsers and 38-column v2.0 consumers keep working unchanged.

| Position | Name | Notes |
|---:|---|---|
| 38 | first_word_invalid | Per-record validity flag from the driver CSI struct |
| 39 | rx_estimate_vld | Per-record validity flag; `-1` on non-HE targets, which do not report it |
| 40 | gain_force_val | The receive-gain value this build applied via the lock, as a build constant replicated per row; `-1` when the gain runs free, which is the default |
| 41 | die_temp_c10 | On-die temperature in tenths of a degree Celsius, sampled on a slow timer, so consecutive rows repeat the last sample |

### Schema v2.2 columns (42-43): per-record receive gain

| Position | Name | Notes |
|---:|---|---|
| 42 | agc_gain | Automatic-gain-control value the receiver applied to this record |
| 43 | fft_gain | FFT-gain value for this record |

Both read through the vendor component `esp_csi_gain_ctrl` and are enabled by
`CONFIG_ESP32_CSI_TOOL_RX_GAIN_CTRL`. With the option off (the default) both
columns read `-1`. Two caveats:

1. The component is distributed as prebuilt static libraries. The archives for
   the `esp32` and `esp32s2` targets contain no code on every ESP-IDF version
   from 4.4 to 6.0, so `RX_GAIN_CTRL=y` fails at link on those two targets
   (see `docs/known_issues.md`). Verified working on the ESP32-C6: free-running
   gain varies with the channel (13 distinct `agc_gain` values over a 25 s
   bench capture), and a forced gain reads back constant at the set value in
   every record.
2. `CONFIG_ESP32_CSI_TOOL_RX_GAIN_FORCE` locks the gain to
   `RX_GAIN_FORCE_VALUE` / `RX_GAIN_FORCE_FFT`. Pick the lock value from the
   `agc_gain` distribution of a free-running session on the same bench. A
   locked receiver loses dynamic range: frames whose level falls outside what
   the fixed gain can demodulate are not delivered at all.

Captures from older firmware have no such column; `parse_csi.py` substitutes
`rx_seq = -1` so that "column absent" is distinguishable from a genuine `0`,
and `packet_rate_analyzer.detect_drops()` reports `available: False` rather
than a misleading zero.

The CSI payload format inside the brackets is unchanged from v1 (space-separated signed `int8`, real-imag interleaved, with a trailing space before `]` to preserve Hernandez 2020 `serial_plot_csi_live.py`'s `csi_data.pop()` convention; this matters for any downstream parser that relied on it).

## Backward compatibility (v1 readers)

The v2 schema preserves the v1 column-25 invariant deliberately:

| v1 script | Parser style | v2 input result |
|---|---|---|
| `parse_csi.py` | regex `re.findall(r"\[(.*)\]", l)` | works (position-insensitive) |
| `serial_plot_csi_live.py` | `all_data = res.split(','); csi_data = all_data[25]` | works (CSI_DATA still at column 25) |
| `serial_append_time.py` | text-only append | works (does not parse fields) |
| `serial_measure_rate.py` | `line.find("-")` for RSSI | works (RSSI still at column 3) |
| CSIKit `CSVBeamformReader` | column-name lookup | works (column names unchanged for v1 columns; new columns ignored) |
| csiread `ESP32` | `np.fromstring` over CSI bracket payload | works |

The claims above are validated against the original Hernandez 2020 sample CSV (`python_utils/lib/example_csi.csv`).

## DIAG and DIAG_END frames (acquisition contract, 2026-09-15)

The consumer task emits a periodic heartbeat comment line:

```
# DIAG,timestamp_us=..,csi_rx=..,cb_reject=..,csi_drop=..,layout_drop=..,
  channel=..,heap_free=..,heap_min=..,queue_wait=..,format_fail=..,
  sink_fail=..,session=..[,gptimer_*][,cpu_load_permille=..]
```

Fields are `name=value` pairs; consumers read them **by name**, never by
position, so appended fields stay compatible. Counter increment sites:
`csi_rx` (callback entry, before any check), `cb_reject` (rejected before
enqueue, callback), `csi_drop` (queue full, callback), `layout_drop`
(layout classification, consumer), `format_fail` (CSV formatting returned
an error, consumer), `sink_fail` (at least one configured sink reported a
detectable write failure, consumer). `session` carries one value per boot,
so a window that straddles a reset is detectable. `queue_wait` is the
queue depth at emission.

Under the termination protocol (`csi_capture_stop()` or
`CONFIG_ESP32_CSI_TOOL_CAPTURE_DURATION_S`), the consumer drains its queue
and writes one terminal frame `# DIAG_END,...,drained=1` after the last
data row, then flushes every sink. **Only a DIAG_END with `drained=1`
certifies that the capture tail is complete**; an ordinary trailing DIAG
does not. The host-side verdict for one capture is
`supportive_folder/scripts/audit_verdict.py` (strict mode requires a
drained DIAG_END). Deterministic fault injection for validating the
classification sites is gated behind `CONFIG_ESP32_CSI_TOOL_FAULT_INJECT`
(test builds only; see the Kconfig help texts).

## Forward compatibility (v2 readers)

The v2 reader (`python_utils/parse_csi.py`) auto-detects the schema by counting columns. Both v1 and v2 inputs are accepted; v2 inputs additionally expose the new fields on the returned record dict.

## Conversion utilities


A round-trip RMSE on the Hernandez 2020 sample CSV must remain less than `1.0` in `int8` units.


## Subcarrier layout semantics (bench-established 2026-08-08)

Layouts below were established from guard-band, DC, and center-null
fingerprints over 3000-frame amplitude averages (ESP-IDF v6.0.1, 2.4 GHz).
Entries are complex I/Q pairs; indices refer to complex samples.

| Chip | Record | Samples | Sections | Order | Tone reference |
|---|---|---|---|---|---|
| ESP32 (pre-HE) | HT20 | 128 | [0:64] L-LTF, [64:128] HT-LTF20 | natural -32..+31 (DC at section idx 32) | primary |
| ESP32 (pre-HE) | HT40 | 192 | [0:64] L-LTF (natural), [64:192] HT-LTF40 | HT-LTF40 in FFT order 0..+63, -64..-1 | primary (m = k) |
| C5 (MAC v3) | HT20 | 57 | single LTF, pruned | natural -28..+28 (DC at idx 28) | primary |
| C5 (MAC v3) | HT40 | 117 | single LTF, pruned | natural -58..+58 (nulls at 57-59) | primary (m = k) |

Cautions:
1. The first two complex samples of ESP32 records are not valid tones
   (header artifact; amplitude of sample 0 is a constant large value).
2. Spectral orientation is not stable across extraction paths, and the
   relation between the HT20 and HT40 axes changes between captures on an
   undisturbed bench, on both the ESP32 and the C5 (known_issues.md
   finding 6, updated 2026-08-19). The C5 does not flip per frame: within a
   run its orientation decision is uniform. Resolving orientation per pair
   is necessary and it is not sufficient: measured on a single sender that
   alternates HT20 and HT40 frames, the cross-format amplitude error after
   orientation resolution is still about twice the same-format measurement
   floor on both receiver families. Keep a capture single-format when
   amplitudes will be compared.
3. HT40 overlap mapping: with secondary BELOW, HT20 primary tone k
   corresponds to HT40 tone m = k in these tables (both primary
   referenced); no +-32 shift is needed.
