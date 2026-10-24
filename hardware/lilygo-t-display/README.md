# LilyGO TTGO T-Display: reference files

The gamepad bridge's board. These are LilyGO's own files from [Xinyuan-LilyGO/TTGO-T-Display](https://github.com/Xinyuan-LilyGO/TTGO-T-Display) at commit `6f89cdd` (2025-02-20), copied unchanged, under its MIT licence (`LICENSE`).

| Path | What it is |
|---|---|
| `3d_file/T-Display.step` | the board as a 3D model (Open CASCADE export, 2021-07-30) |
| `3d_file/prt0003_23.stl`, `3d_file/prt0005_14(2).stl` | LilyGO's case shell (upstream commit "3D Shell file", 2022-04-18) |
| `schematic/ESP32-TFT(6-26).pdf` | the schematic |
| `kicad/TTGO_T_Display/` | a KiCad symbol and footprint (by WolodiaM; its README says how to add them) |
| `eagle/TTGO-TDisplay.lbr` | an Eagle library |

The model is LilyGO's generic T-Display; check it against the v1.1 board fitted before relying on a dimension. Resellers give the PCB as about 51.4 × 25.2 mm, and 51.5 × 25.0 × 8.5 mm with components.
