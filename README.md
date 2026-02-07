# CircularNestingGUI

## Overview
CircularNestingGUI is a research-oriented 2D irregular nesting system with a Qt GUI.
The current version focuses on **circular sheet nesting** (single circular plate).
It supports importing polygonal parts (including holes), placing them into a circle under non-overlap and in-bound constraints, and reporting utilization.

Current repository (modified version):
https://gitee.com/lhqzx/auto.git

This repository is derived from the upstream open-source project:
https://github.com/lryan599/2DNesting

## Scope and Assumptions
- Circular sheet only.
- Single sheet (one circle) per run.
- The GUI uses **CGAL by default**, and allows switching to **Clipper** for faster boolean operations.

## Algorithms (High-Level)
This codebase combines constructive heuristics, local search, and robustness strategies:

- Geometry foundations:
  - NFP (No-Fit Polygon) for non-overlap constraints.
  - IFR (Inner-Fit Region) for in-bound feasibility.
  - Polygon boolean operations (union/intersection/difference) via CGAL or Clipper.

- Constructive placement:
  - Sequential greedy placement with candidate point sampling.
  - Candidate generation from NFP/IFR boundaries and arrangement-style intersections.
  - Radial sampling for circular domains.

- Local improvement:
  - Compaction with **Simulated Annealing** acceptance.
  - LNS-style ruin-and-recreate insertion when a part cannot be placed.
  - Optional multi-order scheduling search.
  - Optional diameter optimization (step and/or binary search).

## Build (Windows)
### Toolchain
- Visual Studio 2022 (MSVC v143), x64 recommended.
- Qt **6.9.3** (MSVC2022 64-bit).

### Build steps
1. Open `nesting_gui.sln` with Visual Studio.
2. Select `Release | x64`.
3. Build the solution.

## Run
- Run from Visual Studio during development.
- If you distribute the executable, please follow the packaging section below.

## Packaging (Standalone EXE)
On Windows, a Qt application cannot usually be executed by double-clicking a raw build output directory, because Qt runtime DLLs and plugins are not present.
To create a runnable folder:

1. Build `Release | x64`.
2. Run `windeployqt` in the folder containing `NestingGUI.exe`:
   - `windeployqt NestingGUI.exe`
3. Ensure CGAL runtime DLLs are present next to the exe:
   - `libgmp-10.dll`
   - `libmpfr-4.dll`
4. Ensure the target machine has the MSVC runtime installed:
   - Microsoft Visual C++ Redistributable 2015-2022 (x64)

## Repository Dependencies
The following dependencies are vendored in this repository:
- CGAL 5.6.1 (`CGAL-5.6.1/`)
- Boost 1.84 (`boost/`)
- Clipper2 (`clipper/`)
- libdxfrw (`libdxfrw/`)
- Auxiliary single-header libraries (`csv.h`, `wyhash.h`, `lru_size.h`, etc.)

## Experimental Records
Experiment logs / records can be found in `records/`.

## Competition Modification Notes
- Original author: lryan599 (upstream repository: https://github.com/lryan599/2DNesting)
- Modifier: lhqzx / Team: 排山倒海
- Modification date: 2025-12-21
- Modification scope:
  1. Refocused the problem setting to **circular sheet nesting**, and refactored/extended the main solver workflow based on `circle_nesting.*`.
  2. Introduced dual geometry backends: GUI default **CGAL** (exact geometry), with an optional switch to **Clipper** (integer boolean operations) for higher boolean performance.
  3. Implemented and used **Simulated Annealing** acceptance in the local optimization stage, to escape local minima during layout compaction.
  4. Added an **LNS-style ruin-and-recreate insertion strategy** for placement failures (locally remove nearby small parts → insert the target part → reinsert removed parts).
  5. Enabled quality-oriented strategies such as **multi-order scheduling search** and **diameter optimization (step / binary search)** in high-quality mode.
- License compliance statement: this modified version is released under **GNU General Public License v3.0**, consistent with the upstream license. See `LICENSE` in the repository root for the full text.
- Change statement: compared with the upstream project, this repository introduces substantial changes in problem setting (circular sheet), algorithmic workflow, and implementation details. While some foundational geometry and project structure remain, the current main pipeline differs significantly from the upstream version.

## License
This project is distributed under **GPL-3.0** (CGAL licensing constraints apply).

This repository also vendors third-party libraries with their own licenses, including:
- **Clipper2**: Boost Software License 1.0 (see `clipper/Clipper2-main/Clipper2-main/LICENSE`).

## Credits
This repository is based on the upstream open-source project:
https://github.com/lryan599/2DNesting