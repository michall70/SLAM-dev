# AGENTS.md

## Build & Run

```sh
./cmake_rebuild.sh          # clean build (deletes & recreates build/)
./build/OrbExtractor        # ORB feature extraction from .bag
./build/TrajectoryCalculator # monocular VO → outputs to ./traj_output/
./build/TrajectoryViewer    # 3D Pangolin viewer for ./traj_output/trajectory.txt
```

- `cmake_rebuild.sh` does `rm -rf build && mkdir build && cmake .. && make -j$(nproc)`. Single command, always clean.
- Dependencies: OpenCV, Pangolin, OrbbecSDK (vendored in `lib/orbbecsdk/`). All must be installed system-wide.
- OrbbecSDK udev rules: `sudo lib/orbbecsdk/shared/install_udev_rules.sh`

## Source Layout

| File | Target | What it does |
|------|--------|--------------|
| `src/extract.cpp` | (commented out) | Extracts IR left/right frames from .bag to PNG |
| `src/depth.cpp` | (commented out) | Stereo depth via SGBM from IR left/right |
| `src/ORB.cpp` | `OrbExtractor` | ORB keypoints on IR left frames, every 8th frame |
| `src/trajectory.cpp` | `TrajectoryCalculator` | Monocular VO (ORB → BFMatcher → findEssentialMat → recoverPose) |
| `src/plotTrajectory.cpp` | `TrajectoryViewer` | Pangolin 3D viewer for trajectory.txt |
| `src/template.cpp` | (not in CMake) | Minimal playback pipeline template |

## Code Quirks

- **ORB warm-up**: First `detectAndCompute` call lazily allocates internal buffers (~2.7s). All executables that use ORB prime it with a random-noise image at startup.
- **Hardcoded .bag paths**: Every source file embeds an absolute path to `data/*.bag`. Change the file path string in each source to switch input.
- **Camera intrinsics**: `depth.cpp` uses hardcoded `FX_PX=620.0` and `BASELINE_MM=95.0` (Orbbec Gemini 336L). `trajectory.cpp` reads intrinsics from the SDK metadata.
- **No `.gitignore`**, no CI, no lint/formatter config.
- `include/` is empty — all headers pulled from OrbbecSDK or system paths.
