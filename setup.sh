#!/bin/bash

echo "=== Setting Up OPENMP-IMAGEPROCESSOR ==="
PROJECT_ROOT=$(pwd)
echo "Project root: $PROJECT_ROOT"

# ---------- Clone Dependencies ----------
mkdir -p external && cd external

if [ ! -d "stb" ]; then
    git clone https://github.com/nothings/stb.git
fi

if [ ! -d "imgui" ]; then
    git clone https://github.com/ocornut/imgui.git
    cd imgui && git checkout v1.89.9 && cd ..
fi

if [ ! -d "gl3w" ]; then
    git clone https://github.com/skaslev/gl3w.git
fi

cd gl3w

if [ ! -f "src/gl3w.c" ]; then
    echo "Generating gl3w.c and gl3w.h..."
    python3 gl3w_gen.py
fi

cd "$PROJECT_ROOT"

# ---------- System Dependencies ----------
echo "Checking for required dependencies: CMake, GLFW, and OpenMP..."

if command -v brew >/dev/null 2>&1; then
    brew install glfw libomp cmake
else
    echo "Homebrew not found. Please install dependencies manually:"
    echo "  - Ubuntu: sudo apt-get install cmake libglfw3-dev libomp-dev"
    echo "  - Others: Refer to your platform's documentation"
fi

# ---------- Build Project ----------
echo "Creating build directory..."
mkdir -p build && cd build

echo "Configuring with CMake..."
cmake ..

echo "Building project..."
make -j$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

echo "Job done."
echo "To run the program:"
echo "  cd build"
echo "  ./OPENMP-IMAGEPROCESSOR"