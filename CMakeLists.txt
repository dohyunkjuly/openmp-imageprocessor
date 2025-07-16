cmake_minimum_required(VERSION 3.16)
project(OPENMP-IMAGEPROCESSOR)

# ======================
# Project Settings
# ======================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ======================
# Vcpkg Integration (Optional)
# ======================
if(DEFINED ENV{VCPKG_ROOT})
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
        CACHE STRING "Vcpkg toolchain file")
endif()

# ======================
# Directory Setup
# ======================
set(SRC_DIR       ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(EXTERNAL_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/external)

# ======================
# Source Files
# ======================
set(IMGUI_SOURCES
    ${EXTERNAL_DIR}/imgui/imgui.cpp
    ${EXTERNAL_DIR}/imgui/imgui_demo.cpp
    ${EXTERNAL_DIR}/imgui/imgui_draw.cpp
    ${EXTERNAL_DIR}/imgui/imgui_tables.cpp
    ${EXTERNAL_DIR}/imgui/imgui_widgets.cpp
    ${EXTERNAL_DIR}/imgui/backends/imgui_impl_glfw.cpp
    ${EXTERNAL_DIR}/imgui/backends/imgui_impl_opengl3.cpp
)

set(GL3W_SOURCES
    ${EXTERNAL_DIR}/gl3w/src/gl3w.c
)

set(SOURCES
    ${SRC_DIR}/main.cpp
    ${SRC_DIR}/ImageProcessor.cpp
    ${SRC_DIR}/PerformanceTimer.cpp
    ${IMGUI_SOURCES}
    ${GL3W_SOURCES}
)

# ======================
# Include Directories
# ======================
include_directories(${SRC_DIR})
include_directories(SYSTEM
    ${EXTERNAL_DIR}/imgui
    ${EXTERNAL_DIR}/imgui/backends
    ${EXTERNAL_DIR}/gl3w/include
    ${EXTERNAL_DIR}/stb
)

# ======================
# OpenMP Setup For MacOS
# ======================
if(APPLE)
    message(STATUS "Configuring OpenMP manually for macOS...")

    if(EXISTS "/opt/homebrew/opt/libomp")
        set(LIBOMP "/opt/homebrew/opt/libomp")
    elseif(EXISTS "/usr/local/opt/libomp")
        set(LIBOMP "/usr/local/opt/libomp")
    else()
        message(FATAL_ERROR "libomp not found. Please install it using: brew install libomp")
    endif()

    set(OpenMP_C_FLAGS       "-Xpreprocessor -fopenmp -I${LIBOMP}/include")
    set(OpenMP_CXX_FLAGS     "-Xpreprocessor -fopenmp -I${LIBOMP}/include")
    set(OpenMP_C_LIB_NAMES   "omp")
    set(OpenMP_CXX_LIB_NAMES "omp")
    set(OpenMP_omp_LIBRARY   "${LIBOMP}/lib/libomp.dylib")
endif()

find_package(OpenMP REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 CONFIG REQUIRED)

# ======================
# Executable Target
# ======================
add_executable(${PROJECT_NAME} ${SOURCES})

# ======================
# Link Libraries
# ======================
target_link_libraries(${PROJECT_NAME}
    OpenGL::GL
    glfw
    ${CMAKE_DL_LIBS}
)

if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)
endif()

# ======================
# Compiler-Specific Flags
# ======================
if(APPLE)
    target_link_libraries(${PROJECT_NAME}
        "-framework Cocoa"
        "-framework IOKit"
        "-framework CoreVideo"
    )
endif()
