cmake_minimum_required(VERSION 3.15)
project(RANSAC_PointCloud_GUI_Project CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/DNOMINMAX>")

# vcpkg（Qt6 等）
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND EXISTS "F:/vcpkg/scripts/buildsystems/vcpkg.cmake")
    set(CMAKE_TOOLCHAIN_FILE "F:/vcpkg/scripts/buildsystems/vcpkg.cmake"
        CACHE FILEPATH "vcpkg toolchain")
endif()

# ====================== 选项（GUI 工程默认开窗口） ======================
option(BUILD_GUI "编译 Qt 图形界面 RANSAC_PointCloud_GUI" ON)
option(BUILD_CLI "编译命令行 detect_cli" OFF)
option(RANSAC_CORE_SHARED "ransac_core 为 DLL（GUI 建议 OFF 用静态库）" OFF)

# ====================== Eigen 5 ======================
set(EIGEN5_ROOT "F:/Algorithm/eigen-5.0.0")
set(EIGEN3_INCLUDE_DIR ${EIGEN5_ROOT})
set(EIGEN3_FOUND TRUE)

add_subdirectory(${CMAKE_SOURCE_DIR}/third_party/ransac_sd ${CMAKE_BINARY_DIR}/ransac_sd_lib)

# ====================== 核心库 ======================
set(CORE_SOURCES
    src/detected_shape.cpp
    src/point_cloud_io.cpp
    src/point_cloud_generator.cpp
    src/ransac_detector.cpp
    src/schnabel_detector.cpp
    src/ransac_lib_api.cpp
)

if(RANSAC_CORE_SHARED)
    add_library(ransac_core SHARED ${CORE_SOURCES})
    target_compile_definitions(ransac_core PRIVATE RANSAC_CORE_EXPORTS)
    target_compile_definitions(ransac_core PUBLIC RANSAC_CORE_DLL)
else()
    add_library(ransac_core STATIC ${CORE_SOURCES})
endif()

target_include_directories(ransac_core
    PUBLIC
        ${CMAKE_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(ransac_core PUBLIC QRANSAC_SD_PRIM_SHAPES_LIB)

target_compile_definitions(ransac_core PUBLIC POINTSWITHINDEX)

# ====================== Qt GUI ======================
if(BUILD_GUI)
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTOUIC ON)
    set(CMAKE_AUTORCC ON)

    find_package(Qt6 REQUIRED COMPONENTS
        Core Gui Widgets Concurrent OpenGLWidgets OpenGL
    )

    add_executable(RANSAC_PointCloud_GUI
        gui/src/main_gui.cpp
        gui/src/main_window.cpp
        gui/src/point_cloud_viewer_widget.cpp
        gui/include/main_window.h
        gui/include/point_cloud_viewer_widget.h
    )

    target_include_directories(RANSAC_PointCloud_GUI PRIVATE
        ${CMAKE_SOURCE_DIR}/gui/include
        ${CMAKE_SOURCE_DIR}/include
    )

    target_link_libraries(RANSAC_PointCloud_GUI PRIVATE
        ransac_core
        Qt6::Core
        Qt6::Gui
        Qt6::Widgets
        Qt6::Concurrent
        Qt6::OpenGLWidgets
        Qt6::OpenGL
    )

    set_target_properties(RANSAC_PointCloud_GUI PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/out"
        RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_SOURCE_DIR}/out"
        RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/out"
    )

    if(TARGET Qt6::windeployqt)
        qt_finalize_executable(RANSAC_PointCloud_GUI)
    endif()
endif()

# ====================== 命令行（可选） ======================
if(BUILD_CLI)
    add_executable(detect_cli tools/detect_cli.cpp)
    target_link_libraries(detect_cli PRIVATE ransac_core)
    set_target_properties(detect_cli PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/out"
        RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_SOURCE_DIR}/out"
        RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/out"
    )
endif()

install(TARGETS ransac_core
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
if(BUILD_GUI)
    install(TARGETS RANSAC_PointCloud_GUI RUNTIME DESTINATION bin)
endif()
install(DIRECTORY include/ DESTINATION include)
