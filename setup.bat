@echo off
echo === Setting Up OPENMP-IMAGEPROCESSOR ===

:: Save current directory
set "PROJECT_ROOT=%cd%"
echo Project root: %PROJECT_ROOT%

:: ---------- Clone Dependencies ----------
cd /d %PROJECT_ROOT%
if not exist external mkdir external
cd external

echo.
echo 1. Downloading STB library...
if not exist stb (
    git clone https://github.com/nothings/stb.git
)

echo.
echo 2. Downloading ImGui...
if not exist imgui (
    git clone https://github.com/ocornut/imgui.git
    cd imgui
    git checkout v1.89.9
    cd ..
)

echo.
echo 3. Downloading GL3W...
if not exist gl3w (
    git clone https://github.com/skaslev/gl3w.git
)

cd gl3w
if not exist src\gl3w.c (
    echo Generating gl3w.c and gl3w.h...
    python gl3w_gen.py
)
cd ..

:: ---------- System Dependencies ----------
echo.
echo 4. Ensure GLFW and OpenMP are installed on your system.
echo [!] If using vcpkg, make sure to install GLFW:
echo     > vcpkg install glfw3

:: ---------- Build Project ----------
cd /d %PROJECT_ROOT%
if not exist build mkdir build
cd build

echo.
echo 5. Configuring with CMake...

cmake .. -DCMAKE_TOOLCHAIN_FILE=%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake -A x64

echo.
echo 6. Building project...
cmake --build . --config Release

echo Job done.
echo To run the program:
echo   cd build\Release
echo   OPENMP-IMAGEPROCESSOR.exe
