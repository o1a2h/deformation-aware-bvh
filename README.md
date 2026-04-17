# Deformation-Aware BVH Updates via Kinetic Extreme Vertex Tracking

Unity 6 reference implementation and benchmark harness for the paper:

> M. A. Ala'anzy and O. Bazarkhan, "Deformation-Aware BVH Updates via Kinetic Extreme Vertex Tracking for Real-Time Soft-Body Simulation," *IEEE Access*, 2026. Paper DOI: `10.1109/ACCESS.20XX.XXXXXXX` (pending). Code archive: https://doi.org/10.5281/zenodo.19632040.

This repository contains four BVH update strategies implemented end-to-end in C# inside Unity 6, along with the benchmark harness that produced every figure and table in Section IV of the paper.

## Overview

Real-time soft-body simulation requires bounding volume hierarchies that update under continuous mesh deformation. Existing approaches run BVH maintenance as a separate pass after the physics solver, paying $O(N)$ cost regardless of how much geometry actually moved. The method in this repository, `AwareUpdater`, instead embeds a twelve-comparison kinetic check directly inside the per-vertex solver loop. Vertices that remain inside their leaf bounds exit the update in $O(1)$; only those that escape propagate dirty flags up the hierarchy. The remaining pass recomputes bounds for flagged nodes only.

Three baselines are also included, each reimplemented faithfully against the original papers:

| Method | File | Reference |
|---|---|---|
| `AwareUpdater` (ours) | `Assets/Scripts/AwareUpdater.cs` | This paper |
| `KineticUpdater` | `Assets/Scripts/KineticUpdater.cs` | Zachmann & Weller, 2006 |
| `RefitRotateUpdater` | `Assets/Scripts/RefitRotateUpdater.cs` | Kopta et al., 2012 |
| `DynamicBVHUpdater` | `Assets/Scripts/DynamicBVHUpdater.cs` | Larsson & Akenine-Möller, 2006 |

## Repository layout

```text
Assets/
  Scripts/               BVH updaters, tree, benchmark harness
  Scripts/Editor/        Custom inspectors for the benchmark components
  Mesh/                  Test meshes (Dragon, Suzanne, Plane)
  Scenes/                Benchmark scene
Packages/                Unity package manifest (locked versions)
ProjectSettings/         Unity project configuration
```

## Requirements

- Unity **6.0 LTS** or newer (tested on Unity 6.0.23f1)
- Windows, macOS, or Linux with a GPU capable of running the Unity Editor
- Approximately 8 GB RAM for the highest-resolution benchmark meshes

No external packages are required beyond those pinned in `Packages/manifest.json`.

## Quick start

1. Clone the repository:
```bash
   git clone [https://github.com/o1a2h/deformation-aware-bvh.git](https://github.com/o1a2h/deformation-aware-bvh.git)
```
2. Open `deformation-aware-bvh/` in Unity Hub → **Add project from disk**.
3. Open the benchmark scene at `Assets/Scenes/Benchmark.unity`.
4. Select the `Benchmark` GameObject in the hierarchy. Its inspector exposes:
   - **Method**: `AwareBVH`, `KineticBVH`, `RefitRotate`, or `DynamicBVH`
   - **Deformation**: `None`, `SineWave`, `Localized`
   - **Max Tris Per Leaf**: `{2, 32, 512, 8192}`
5. Press **Play**, then click **▶ Benchmark (10×)** in the custom inspector.

Per-frame metrics (deformation time, BVH update time, collision-query time, total frame time, and structural counters) are logged to CSV in the path specified by the benchmark component.

## Reproducing the paper's figures

To run the full matrix used in the paper, select the `GlobalBenchmark` component (Scripts/GlobalBenchmark.cs) and press **▶ Start Global Benchmark**. This sweeps:

- 9 meshes (Dragon, Suzanne, Plane at three resolutions each)
- 3 deformation types (`None`, `Localized`, `SineWave`)
- 4 methods
- 4 leaf capacities

The default output directory is configured at the top of `GlobalBenchmark.cs`; edit the `saveDirectory` field before running. Expect approximately 2–3 hours on the hardware reported in the paper.

The CSV outputs are then processed by the Jupyter notebooks in `Assets/Benchmark/` to produce the figures in Section IV of the paper.

## Hardware used in the paper

- Windows 11 Pro
- AMD Ryzen 9 7940HS (4.00 GHz)
- 16 GB RAM
- Single CPU core (multithreading disabled for reported results)
- Unity 6.0.23f1

## Citation

**To cite the paper:**
```bibtex
@article{alaanzy2026deformation,
  title   = {Deformation-Aware {BVH} Updates via Kinetic Extreme Vertex Tracking for Real-Time Soft-Body Simulation},
  author  = {Ala'anzy, Mohammed Alaa and Bazarkhan, Olzhas},
  journal = {IEEE Access},
  year    = {2026},
  note    = {Code: \url{[https://github.com/o1a2h/deformation-aware-bvh](https://github.com/o1a2h/deformation-aware-bvh)}}
}
```

**To cite the code/benchmark directly:**
> Ala'anzy, M. A., & Bazarkhan, O. (2026). Deformation-Aware BVH Updates via Kinetic Extreme Vertex Tracking (v1.0.0). Zenodo. https://doi.org/10.5281/zenodo.19632040

## License

Released under the MIT License — see [`LICENSE`](LICENSE).

The Chinese Dragon mesh redistributed under `Assets/Mesh/` is from the Computer Graphics Archive (McGuire, 2017) and retains its original license; see that archive for terms.

## Contact

- Mohammed Alaa Ala'anzy — `m.alanzy@ieee.org`
- Olzhas Bazarkhan — `olzhasbazarkhan@gmail.com`

Issues and pull requests are welcome.
```
