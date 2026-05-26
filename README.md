# RGBD Visual SLAM

Monocular visual odometry from Orbbec stereo IR camera recordings (`.bag` files).

## Project Structure

```
output/             # Generated trajectory.txt, match images, etc.
projects/
├── cmake_rebuild.sh    # One-click build: delete build/ → cmake → make
├── CMakeLists.txt      # Build config — uncomment targets to activate
├── bin/                # Compiled executables (auto-created by cmake)
├── src/                # Source files (*.cpp)
│   ├── trajectory.cpp      # VO: ORB → essential matrix → trajectory
│   ├── plotTrajectory.cpp  # Pangolin 3D viewer
│   ├── extract.cpp         # Extract IR frames as PNG
│   ├── depth.cpp           # Stereo depth via SGBM
│   ├── ORB.cpp             # ORB feature extraction demo
│   └── template.cpp        # Minimal Orbbec playback skeleton (always disabled)
└── lib/orbbecsdk/      # Vendored OrbbecSDK 2.8.6
```

## Before Build

**Change `.bag` file path** in source code (in `projects/src/`)

## Build

**Prerequisites:** OpenCV, Pangolin (system-installed; found via `find_package`).

```bash
cd projects
bash cmake_rebuild.sh
```

This deletes the `build/` directory, runs cmake, and compiles with `make -j$(nproc)`.

Output binaries appear in `projects/bin/`.

## Activating Other Source Files

Only `TrajectoryCalculator` (`trajectory.cpp`) is currently built. To enable another target, edit `projects/CMakeLists.txt` and uncomment its `add_executable` + `target_link_libraries` lines:

```cmake
# Example: enable stereo depth
add_executable(DepthCalculator src/depth.cpp)
target_link_libraries(DepthCalculator OrbbecSDK)
target_link_libraries(DepthCalculator ${OpenCV_LIBS})
```

Then rebuild with `bash cmake_rebuild.sh`.

## Data

Input `.bag` files live in `projects/data/`. Hardcoded absolute paths in many `src/*.cpp` files point to specific bags — change these when moving to another machine.

- `.bag` files are not included in the repository.

## Run

The **compiled executables** are placed in `projects/bin/`

```bash
cd projects
./bin/TrajectoryCalculator    # computes trajectory.txt and saves to output/traj_output/
```
