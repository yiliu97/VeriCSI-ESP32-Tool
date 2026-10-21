# Citation Lineage

VeriCSI-ESP32 is an independent toolkit in the ESP32-CSI-Tool lineage. It
preserves the original CSV format as a strict super-set and the standalone
STA-AP / passive-sniffer architecture, so pipelines built on the original
tool continue to compile, parse and reproduce. The measurement core is
designed for ESP-IDF v6 and heterogeneous ESP32 SoCs.

## Foundational citation (always cite)

```bibtex
@inproceedings{hernandez2020lightweight,
  title={Lightweight and Standalone IoT Based WiFi Sensing for Active Repositioning and Mobility},
  author={Hernandez, Steven M. and Bulut, Eyuphan},
  booktitle={2020 IEEE 21st International Symposium on a World of Wireless, Mobile and Multimedia Networks (WoWMoM)},
  pages={277--286},
  year={2020},
  organization={IEEE},
  doi={10.1109/WoWMoM49955.2020.00056}
}
```

## VeriCSI-ESP32 citation

To be added on publication of the VeriCSI-ESP32 paper. Until then, cite the
repository: https://github.com/yiliu97/VeriCSI-ESP32-Tool

## Lineage and history

- **2020-08, IEEE WoWMoM.** Original ESP32-CSI-Tool released as the artifact
  accompanying Hernandez & Bulut 2020. Targets ESP-IDF v4.3 on the original
  ESP32 only.
- **2020 to 2023.** Active maintenance, then README-only updates; last
  upstream commit 2023-03-29.
- **2023 to 2026.** Issue queue accumulates; fragmented forks emerge
  (espidf_v4 branches, single-chip S3 ports, esp-csi derivatives). None
  simultaneously covers (a) original CSV compatibility, (b) five or more
  SoCs, (c) HE-LTF acquisition, (d) ESP-IDF v6.
- **2026.** VeriCSI-ESP32: single codebase for seven ESP32-family SoCs on
  ESP-IDF v6.0.1, HE-era metadata semantics with explicit not-available
  encoding, microsecond timer-driven sounding, sequence-level loss
  accounting, dual-band (5 GHz) support on the ESP32-C5, and a multi-sink
  output layer, while preserving the original CSV schema as a strict
  super-set.

## Licence

MIT, inherited from the original tool. The Hernandez / Bulut authorship of
the foundational work is credited above.
