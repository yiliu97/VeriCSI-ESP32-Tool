# Contributing

Thanks for thinking about helping out. VeriCSI-ESP32 is an independent toolkit in the ESP32-CSI-Tool lineage (Hernandez & Bulut, WoWMoM 2020) and keeps that tool's spirit: small, readable code that researchers can fork and audit.

This document captures the conventions the maintainers follow so that new patches feel like part of the same codebase.

## What kinds of contributions are welcome

- **Bug fixes** in `_components/`, `active_*/main/`, `passive/main/`, or `python_utils/`.
- **New chip support** when a new Espressif Wi-Fi SoC is released. Add a `sdkconfig.defaults.<target>` and a row in `docs/chip_support_matrix.md`.
- **New sinks** (Bluetooth notify, USB MSC, etc.) following the contract in `docs/sink_guide.md`.
- **CSV / decoder fixes** when a chip's actual subcarrier ordering turns out to differ from `python_utils/lib/chip_aware_decoder.py` once tested on hardware.
- **Documentation improvements** - especially clarifying upstream IDF issue workarounds in `docs/known_issues.md`.

## What is harder to land

- Anything that breaks `python_utils/parse_csi.py` parsing of the bundled `python_utils/lib/example_csi.csv` (Hernandez 2020 sample). Byte-for-byte compatibility with the original schema is a non-negotiable invariant - downstream papers built on it must keep working.
- Anything that depends on a particular non-mainline IDF fork. The tool pins to `espressif/idf:v6.0.1`; version bumps must keep all 21 build targets (3 applications x 7 SoCs) compiling.
- Large new sub-projects beyond the v1 trio (`active_sta`, `active_ap`, `passive`). The Hernandez 2020 architecture is intentionally compact; major new applications are usually better as separate forks.

## Branch model

```
upstream:   main         <- protected, only release tags land here
            develop      <- integration branch, PRs target this
            feature/*    <- short-lived per-PR branches
your fork:  ...
```

Open PRs against `develop`, not `main`. Maintainers fast-forward `main` to a tagged commit at release time.

## Code style

The original Hernandez 2020 code uses:

- 4-space indent, no tabs.
- `snake_case` for functions and variables.
- Header guards in `ESP32_CSI_<NAME>_COMPONENT_H` form.
- Header-only components included once from `main.cc` (so non-static globals live in headers; this is intentional, not a bug).
- Plain C-style `printf` for diagnostics; `ESP_LOGI` / `ESP_LOGW` / `ESP_LOGE` for events that downstream tooling might want to parse.
- Help text in `Kconfig.projbuild` written as full English sentences with `[Advanced Users]` callouts.

v2.0 keeps this style. New code should not introduce e.g. `camelCase`, exceptions, dynamic allocation in hot paths, or modern C++17/20 features beyond what the IDF v6 toolchain (gnu++26) requires for inline initialisers.

## Pre-PR checklist

Before opening a PR:

1. **Build at least one chip locally**:

    ```bash
    docker run --rm -it -v $(pwd):/workspace espressif/idf:v6.0.1 bash -lc \
        ". /opt/esp/idf/export.sh && cd /workspace/active_sta && \
         idf.py set-target esp32 && idf.py build"
    ```

2. **Run the Python tests**:

    The test suite ships with the paper's supplementary material
    (`tests/`), outside this repository. It reads this checkout through
    `VERICSI_CODE`:

    ```bash
    VERICSI_CODE=$(pwd) python -m pytest <supplement>/tests -v
    ```

    All tests must pass. If a Python change changes the parser semantics, add a new test case to the appropriate file under `tests/` before sending the PR.

3. **Update the changelog**:

    Describe the change in the commit message body (what changed and why).

4. **If you changed a Kconfig**:

    Confirm the legacy v1 alias still works (e.g. `CONFIG_SHOULD_COLLECT_CSI` should still gate the same code that `CONFIG_ESP32_CSI_TOOL_COLLECT_CSI` does). The `*_/main/Kconfig.projbuild` files have hidden mirror entries at the bottom that maintain this compatibility - keep them in sync.

5. **If you added a chip**:

    Add it to both places: `sdkconfig/sdkconfig.defaults.<target>` and `docs/chip_support_matrix.md`.

## Commit messages

The Hernandez 2020 history uses short, lower-case, present-tense imperative messages without ticket prefixes. This repository follows the same style:

```
add CRC32 to binary frame format

handle disconnect during USB-CDC flush gracefully

drop esp_spi_flash.h include (gone in IDF v5+)

fix #14271 chip-aware decoder layout id mismatch
```

Keep the first line under ~70 characters. If you need a longer explanation, add a blank line and a body. Reference upstream IDF issues by number (`espressif/esp-idf#14271`); reference our own issues without owner prefix (`#42`).

## License

All contributions are accepted under the existing MIT license. By submitting a PR you agree that your code can be redistributed under MIT.

The project inherits the upstream Hernandez 2020 MIT license terms. Contributors are listed by git history; if you want explicit attribution in a release, mention it in the PR.

## Hernandez lineage

We try hard to keep the project recognisable to v1 users. Concretely:

- `parse_csi.py` stays at the top of `python_utils/` — it's the canonical reader that v1 papers / tutorials reference. Don't rename or move it.
- This repository reorganised `python_utils/` once: the v1 helpers `serial_append_time.py` / `serial_measure_rate.py` / `read_stdin.py` / `wait_timer.py` plus the v2 add-ons `chip_aware_decoder.py` / `csi_bin_to_csv.py` / `example_csi.csv` are now under `python_utils/lib/` (they are library / specialised tools, not the everyday entry point). The v1 entry-point script `serial_plot_csi_live.py` was rewritten and renamed to `python_utils/csi_live.py` (single-process spawn-and-plot + `--no-plot` headless record). This was a one-time consolidation; don't churn the layout further.
- Don't change the `CSI_DATA` byte layout inside the `[...]` brackets of CSV rows.
- Don't break `idf.py monitor | grep CSI_DATA > file.csv` as the simplest capture command.

If something in v1 is genuinely broken and the fix is a behaviour change, document it loudly in the commit message and add a deprecation section to the README.

## Getting help

- **Tool questions** (how do I capture? how do I configure?): GitHub Discussions of this repo.
- **General ESP32 / IDF questions** (my board does not flash, what baud rate works on macOS?): Espressif's own forums. As stated in the upstream README, please do not file generic ESP32 issues here.
- **Upstream firmware bugs that affect this tool**: file in `espressif/esp-idf` first; reference the issue number in `docs/known_issues.md` when adding a workaround on our side.

Thanks again. Patches welcome.
