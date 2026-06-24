# RANSAC PointCloud Tool

A Qt6 + OpenGL based 3D point cloud RANSAC geometric primitive detection tool, supporting synthetic point cloud generation, multi-format file loading, 6 types of automatic shape detection, and interactive 3D visualization.

![License](https://img.shields.io/badge/license-Apache%202.0-blue)
![C++](https://img.shields.io/badge/C%2B%2B-17-blue)
![Qt](https://img.shields.io/badge/Qt-6.x-green)
![CMake](https://img.shields.io/badge/CMake-3.15%2B-red)

---

## Features

### Point Cloud Generation & Loading

| Feature | Description |
|---------|-------------|
| **Synthetic Generation** | One-click generation of complex synthetic scenes containing planes, spheres, cylinders, cones, tori, and lines |
| **File Loading** | Supports PCD, PLY, STL, TXT/CSV formats |
| **Large File Optimization** | Automatic voxel downsampling with progress bar for smooth handling of millions of points |
| **Voxel Filtering** | Built-in downsampling and deduplication preprocessing |

### RANSAC Shape Detection

| Shape | Description |
|-------|-------------|
| 🔷 Plane | Walls, floors, tabletops and other planar structures |
| 🔵 Sphere | Spherical object recognition |
| 🟨 Cylinder | Pipes, columns and other cylindrical structures |
| 🔺 Cone | Conical object recognition |
| 🍩 Torus | Toroidal structure detection |
| ➖ Line | Linear edge extraction |

### Two Detection Presets

- **Architecture (Large Scenes)**: Detects planes only with higher inlier threshold, suitable for urban, building, and park scenes
- **Full (Mechanical/Parts)**: Detects all 6 shape types, suitable for precision part reverse engineering

### Interactive 3D Visualization

- Mouse rotation, zoom, and pan of the point cloud view
- Detection results tree with checkbox toggles for each shape
- Side-by-side comparison of original cloud vs. unclassified remaining points
- Each shape type rendered with a distinct highlight color

---

## Requirements

| Component | Version | Notes |
|-----------|---------|-------|
| **C++ Compiler** | MSVC 2019+ / GCC 11+ / Clang 14+ | Must support C++17 |
| **CMake** | 3.15+ | Build system |
| **Qt 6** | 6.x | Core / Gui / Widgets / Concurrent / OpenGLWidgets / OpenGL |
| **Eigen** | 5.0.0 | Linear algebra library |

---

## Build Instructions

### 1. Install Qt 6

Download the installer from [Qt Official Site](https://www.qt.io/download-qt-installer), select Qt 6.x, and check the following modules:

- MSVC 2019/2022 64-bit (or your compiler version)
- Qt Concurrent
- Qt OpenGL Widgets
- Qt OpenGL

Note the installation path, e.g., `F:/Qt/6.8.0/msvc2022_64`.

### 2. Install Eigen 5

Download Eigen 5.0.0 from the [Eigen website](https://eigen.tuxfamily.org/), extract it, and note the path (e.g., `F:/Algorithm/eigen-5.0.0`).

### 3. Configure CMakeLists.txt

Update the following two paths in `CMakeLists.txt` to match your local setup:

```cmake
# Line 21 — Eigen path
set(EIGEN5_ROOT "F:/Algorithm/eigen-5.0.0")

# Line 10 — Qt path (add CMAKE_PREFIX_PATH if CMake cannot find Qt6):
set(CMAKE_PREFIX_PATH "F:/Qt/6.8.0/msvc2022_64")
```

### 4. Build

```powershell
# Create build directory
mkdir build-gui && cd build-gui

# Configure
cmake ..

# Build
cmake --build . --config Release
```

The executable will be at `out/RANSAC_PointCloud_GUI.exe`.

### 5. Deploy (Windows)

```powershell
# Use windeployqt to automatically copy Qt DLLs to the output directory
F:/Qt/6.8.0/msvc2022_64/bin/windeployqt.exe out/RANSAC_PointCloud_GUI.exe
```

---

## Usage Guide

### Basic Workflow

```
Generate/Load Point Cloud -> RANSAC Detection -> View Results -> Export
```

### Step 1: Acquire Point Cloud

- **Generate Synthetic Cloud**: Click the **Generate** button to create a synthetic scene with planes, spheres, cylinders, cones, tori, and lines
- **Load File**: Click the **Load** button to open PCD / PLY / STL / TXT point cloud files
- **Save As**: Save the current cloud in PCD format

### Step 2: Detect Shapes

1. Select a detection preset from the toolbar:
   - **Architecture**: Large scene mode, planes only
   - **Full**: Complete detection, all 6 shape types
2. Click the **Detect** button to start RANSAC analysis
3. Wait for progress completion (large clouds may take tens of seconds)

### Step 3: View Results

- **Results Tree**: Left panel lists all detected shapes grouped by type
- **Checkboxes**: Toggle individual shape visibility
- **Right-click Menu**: Show all / Hide all
- **3D View**: Left-click to rotate, scroll to zoom, right-click to pan

### Step 4: Export

Click the **Export** button to save detection results, including:
- Per-shape inlier PCD files
- Remaining unclassified point cloud
- Detection statistics report

---

## Project Structure

```
RANSAC_GUI/
├── CMakeLists.txt              # CMake build configuration
├── README.md                   # Chinese documentation
├── README_EN.md                # English documentation (this file)
├── .gitignore                  # Git ignore rules
│
├── include/                    # Core library headers
│   ├── point_cloud.h           # Point cloud data structures
│   ├── detected_shape.h        # Detection result structs
│   ├── ransac_detector.h       # RANSAC detector
│   ├── schnabel_detector.h     # Schnabel detector wrapper
│   ├── point_cloud_generator.h # Synthetic cloud generation
│   ├── point_cloud_io.h        # File I/O (PCD/PLY)
│   ├── analysis_config.h       # Analysis configuration
│   └── region_segmenter.h      # Region segmentation
│
├── src/                        # Core library source
│   ├── ransac_detector.cpp
│   ├── schnabel_detector.cpp
│   ├── point_cloud_generator.cpp
│   ├── point_cloud_io.cpp
│   ├── detected_shape.cpp
│   └── ransac_lib_api.cpp
│
├── gui/                        # Qt GUI source
│   ├── include/
│   │   ├── main_window.h       # Main window
│   │   └── point_cloud_viewer_widget.h  # 3D point cloud view
│   └── src/
│       ├── main_gui.cpp        # Application entry point
│       ├── main_window.cpp     # Main window logic
│       └── point_cloud_viewer_widget.cpp  # OpenGL rendering
│
├── third_party/                # Third-party dependencies
│   └── ransac_sd/              # RANSAC SD primitive shape library
│
└── data/                       # Sample data
    ├── csv_sample.txt          # CSV format sample
    └── data01.stl              # STL format sample
```

---

## FAQ

### Q: Compilation error: Eigen not found?
A: Verify that `EIGEN5_ROOT` in `CMakeLists.txt` points to the correct Eigen path.

### Q: Missing Qt DLLs at runtime?
A: Use `windeployqt` to auto-deploy dependencies, or add the Qt bin directory to your PATH.

### Q: Application freezes loading a large file?
A: The program auto-downsamples via voxel filtering. For files > 500 MB, consider preprocessing with tools like CloudCompare.

### Q: Detection results are not ideal?
A: Try switching between Architecture and Full presets, or manually adjust threshold parameters in `ransac_detector.h`'s `Config` struct.

---

## License

Apache License Version 2.0

---

## Author

Su-yu-s

---

*README generated on 2026-06-24*
