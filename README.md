# RANSAC PointCloud Tool

基于 Qt6 + OpenGL 的三维点云 RANSAC 几何基元检测工具，支持合成点云生成、多格式文件加载、6 种几何形状自动检测和交互式 3D 可视化。

![License](https://img.shields.io/badge/license-Apache%202.0-blue)
![C++](https://img.shields.io/badge/C%2B%2B-17-blue)
![Qt](https://img.shields.io/badge/Qt-6.x-green)
![CMake](https://img.shields.io/badge/CMake-3.15%2B-red)

---

## 功能特性

### 点云生成与加载

| 功能 | 说明 |
|------|------|
| **合成点云生成** | 一键生成包含平面、球体、圆柱、圆锥、圆环、直线的复杂合成场景 |
| **文件加载** | 支持 PCD、PLY、STL、TXT/CSV 多种格式 |
| **大文件优化** | 自动体素降采样 + 进度条，流畅加载百万级点云 |
| **体素滤波** | 内置降采样与去重预处理 |

### RANSAC 形状检测

| 检测类型 | 说明 |
|----------|------|
| 🔷 平面 (Plane) | 墙面、地面、桌面等平面结构 |
| 🔵 球体 (Sphere) | 球状物体识别 |
| 🟨 圆柱 (Cylinder) | 管道、柱体等圆柱形结构 |
| 🔺 圆锥 (Cone) | 锥形物体识别 |
| 🍩 圆环 (Torus) | 环形结构检测 |
| ➖ 直线 (Line) | 线性边缘提取 |

### 两种检测预设

- **Architecture（建筑/大场景）**：仅检测平面，更高内点门槛，适合城市、建筑、园区场景
- **Full（机械/零件）**：检测全部 6 种形状，适合精密零件逆向工程

### 交互式 3D 可视化

- 鼠标旋转/缩放/平移点云视图
- 检测结果列表，勾选切换形状显示
- 原始点云与剩余未分类点云对比
- 每种形状用不同颜色高亮渲染

---

## 运行截图（占位）

> 运行后使用截图工具截取，替换此处。

---

## 环境要求

| 组件 | 版本要求 | 说明 |
|------|---------|------|
| **C++ 编译器** | MSVC 2019+ / GCC 11+ / Clang 14+ | 需支持 C++17 |
| **CMake** | 3.15+ | 构建系统 |
| **Qt 6** | 6.x | Core / Gui / Widgets / Concurrent / OpenGLWidgets / OpenGL |
| **Eigen** | 5.0.0 | 线性代数库 |

---

## 编译步骤

### 1. 安装 Qt 6

从 [Qt 官网](https://www.qt.io/download-qt-installer) 下载安装器，选择 Qt 6.x 版本，勾选以下模块：

- MSVC 2019/2022 64-bit（或对应编译器版本）
- Qt Concurrent
- Qt OpenGL Widgets
- Qt OpenGL

安装后记下 Qt 6 的安装路径，例如 `F:/Qt/6.8.0/msvc2022_64`。

### 2. 安装 Eigen 5

从 [Eigen 官网](https://eigen.tuxfamily.org/) 下载 Eigen 5.0.0，解压到本地，记下路径（例如 `F:/Algorithm/eigen-5.0.0`）。

### 3. 配置 CMakeLists.txt

修改 `CMakeLists.txt` 中的以下两处路径为本机实际路径：

```cmake
# 第 21 行 —— Eigen 路径
set(EIGEN5_ROOT "F:/Algorithm/eigen-5.0.0")

# 第 10 行 —— Qt 路径（添加 CMAKE_PREFIX_PATH）
# 如果 CMake 找不到 Qt6，加上这一行：
set(CMAKE_PREFIX_PATH "F:/Qt/6.8.0/msvc2022_64")
```

### 4. 编译

```powershell
# 创建构建目录
mkdir build-gui && cd build-gui

# 配置
cmake ..

# 编译
cmake --build . --config Release
```

编译产物位于 `out/RANSAC_PointCloud_GUI.exe`。

### 5. 部署（Windows）

```powershell
# 使用 windeployqt 自动复制 Qt DLL 到输出目录
F:/Qt/6.8.0/msvc2022_64/bin/windeployqt.exe out/RANSAC_PointCloud_GUI.exe
```

---

## 使用指南

### 基本工作流程

```
生成/加载点云 → RANSAC 检测 → 查看结果 → 导出
```

### 操作步骤

#### 第一步：获取点云

- **生成合成点云**：点击 **"生成"** 按钮，自动创建一个包含平面、球体、圆柱、圆锥、圆环和直线的合成场景
- **加载文件**：点击 **"加载"** 按钮，选择 PCD / PLY / STL / TXT 格式的点云文件
- **另存为**：可将当前点云保存为 PCD 格式

#### 第二步：检测形状

1. 在工具栏右侧选择检测预设：
   - **Architecture**：大场景模式，仅检测平面
   - **Full**：全检测模式，检测全部 6 种形状
2. 点击 **"检测"** 按钮开始 RANSAC 分析
3. 等待进度条完成（大点云可能需要数十秒）

#### 第三步：查看结果

- **结果树**：左侧列出所有检测到的形状，按类型分组
- **勾选框**：切换单个形状的显示/隐藏
- **右键菜单**：全部显示 / 全部隐藏
- **3D 视图**：鼠标左键旋转、滚轮缩放、右键平移

#### 第四步：导出

点击 **"导出"** 按钮，将检测结果导出为文件，包含：
- 每个形状的内点（inliers）PCD 文件
- 剩余未分类点云
- 检测统计报告

---

## 项目结构

```
RANSAC_GUI/
├── CMakeLists.txt              # CMake 构建配置
├── README.md                   # 本文件
├── .gitignore                  # Git 忽略规则
│
├── include/                    # 核心库头文件
│   ├── point_cloud.h           # 点云数据结构
│   ├── detected_shape.h        # 检测结果结构体
│   ├── ransac_detector.h       # RANSAC 检测器
│   ├── schnabel_detector.h     # Schnabel 检测器封装
│   ├── point_cloud_generator.h # 合成点云生成
│   ├── point_cloud_io.h        # 文件 IO（PCD/PLY）
│   ├── analysis_config.h       # 分析配置
│   └── region_segmenter.h      # 区域分割
│
├── src/                        # 核心库源码
│   ├── ransac_detector.cpp
│   ├── schnabel_detector.cpp
│   ├── point_cloud_generator.cpp
│   ├── point_cloud_io.cpp
│   ├── detected_shape.cpp
│   └── ransac_lib_api.cpp
│
├── gui/                        # Qt GUI 源码
│   ├── include/
│   │   ├── main_window.h       # 主窗口
│   │   └── point_cloud_viewer_widget.h  # 3D 点云视图
│   └── src/
│       ├── main_gui.cpp        # 程序入口
│       ├── main_window.cpp     # 主窗口逻辑
│       └── point_cloud_viewer_widget.cpp  # OpenGL 渲染
│
├── third_party/                # 第三方依赖
│   └── ransac_sd/              # RANSAC SD 基元形状库
│
└── data/                       # 示例数据
    ├── csv_sample.txt          # CSV 格式示例
    └── data01.stl              # STL 格式示例
```

---

## 常见问题

### Q: 编译报错找不到 Eigen？
A: 检查 `CMakeLists.txt` 中 `EIGEN5_ROOT` 是否指向正确的 Eigen 路径。

### Q: 运行提示缺少 Qt DLL？
A: 使用 `windeployqt` 自动部署依赖，或将 Qt bin 目录添加到 PATH 环境变量。

### Q: 加载大文件卡死？
A: 程序会自动体素降采样。如果文件过大（>500MB），建议先用 CloudCompare 等工具预处理。

### Q: 检测结果不理想？
A: 尝试切换检测预设（Architecture vs Full），或在 `ransac_detector.h` 的 `Config` 中手动调整阈值参数。

---

## 许可证

Apache License Version 2.0

---

## 作者

Su-yu-s

---

*README 生成于 2026-06-24*
