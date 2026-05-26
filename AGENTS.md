# AGENTS.md — RGBD Visual SLAM

## Build & Run

- **Build:** `bash projects/cmake_rebuild.sh` (deletes `build/`, runs cmake + make), or `cd projects/build && cmake .. && make -j$(nproc)`
- **Binaries go to** `projects/bin/` (set via `CMAKE_RUNTIME_OUTPUT_DIRECTORY`)
- **Only two targets are built** (others commented out in `projects/CMakeLists.txt`):
  - `TrajectoryCalculator` ← `projects/src/trajectory.cpp` (VO pipeline: ORB → essential matrix → trajectory)
  - `TrajectoryViewer` ← `projects/src/plotTrajectory.cpp` (Pangolin 3D viewer)
- **Activate a disabled target:** uncomment its `add_executable` + `target_link_libraries` in `projects/CMakeLists.txt`
- **Output directory:** `projects/output/` (relative `./output/...` from build)
- **C++17**, requires OpenCV and Pangolin

## Dependencies

- OrbbecSDK 2.8.6 (vendored at `projects/lib/orbbecsdk/`): include `libobsensor/ObSensor.hpp`, link `-lOrbbecSDK`
- SDK prebuilt executables at `projects/lib/orbbecsdk/bin/`
- udev rule setup: `projects/lib/orbbecsdk/shared/install_udev_rules.sh`
- OpenCV and Pangolin are system-installed (found via `find_package`)

## Data

- **Input:** ROS `.bag` files recorded from Orbbec stereo IR cameras, stored in `projects/data/`
  - Main files: `1280_800_30fps.bag`, `IR01.bag`, `IR02.bag`, `sturcture01.bag`
  - Subdirs: `vertical/` (4 bags), `parallel/` (4 bags)
- **Hardcoded absolute paths** in many `src/*.cpp` files (e.g., `/home/michall/AAAProjects/RGBD/projects/data/...`). Change these when moving to another machine.
- **Camera intrinsics** are hardcoded per bag file in the source (e.g., fx=620 for 1280_800, fx=410.7 for IR02)

## Code Patterns

- ORB detector **warm-up**: first `detectAndCompute` call lazily allocates (~2.7s). All source files do a dummy run on random noise to avoid this during playback.
- Orbbec playback pattern: create `PlaybackDevice(path)` → `Pipeline(playback)` → enable all sensor streams → `pipe->start(config)` → loop `pipe->waitForFrames()` until `OB_PLAYBACK_STOPPED` callback fires.
- IR frames wrapped into `cv::Mat` via `cv::Mat(h, w, CV_8UC1, (void*)frame->getData())` (zero-copy).
- `TrajectoryCalculator` saves `trajectory.txt` + match images to `./output/traj_output/`; `TrajectoryViewer` loads `./output/traj_output/trajectory.txt`.

## Source Files (projects/src/)

| File | Built? | Purpose |
|------|--------|---------|
| `trajectory.cpp` | Yes | Full monocular VO: ORB → BF matcher → essential matrix → accumulate pose |
| `plotTrajectory.cpp` | Yes | Pangolin 3D viewer for saved trajectory |
| `template.cpp` | No | Minimal Orbbec playback skeleton |
| `extract.cpp` | No | Extract IR left/right frames as PNG |
| `depth.cpp` | No | Stereo depth via SGBM from IR pair |
| `ORB.cpp` | No | ORB feature extraction + visualization demo |

## Other

- `slambook2-master/` is companion code for the book 《视觉SLAM十四讲》, not part of the active project. Most of its source files are gitignored.
- No test framework is configured. No linter, formatter, or typechecker is set up.
- VS Code uses g++-13 / gdb; `ros.distro` set to `jazzy`.
