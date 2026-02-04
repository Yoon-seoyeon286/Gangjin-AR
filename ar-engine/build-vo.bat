@echo off
REM ============================================================================
REM Visual Odometry (SLAM Core) Build Script (Windows)
REM OpenCV 필요
REM ============================================================================

echo Building Visual Odometry Module...

REM Emscripten 환경 확인
if "%EMSDK%"=="" (
    echo Error: EMSDK environment variable not set.
    echo Please run 'emsdk_env.bat' first.
    echo Example: C:\emsdk\emsdk_env.bat
    exit /b 1
)

REM OpenCV 경로 설정 (환경에 맞게 수정)
set OPENCV_INCLUDE=lib/opencv/include
set OPENCV_LIB=lib/opencv/build/lib

REM 출력 디렉토리 생성
if not exist "public\wasm" mkdir "public\wasm"

REM C++ 파일을 WebAssembly로 컴파일
call emcc ^
    src/cpp/visual_odometry.cpp ^
    src/cpp/vo_bindings.cpp ^
    -o public/wasm/visual-odometry.js ^
    -I src/cpp ^
    -I %OPENCV_INCLUDE% ^
    -L %OPENCV_LIB% ^
    -s WASM=1 ^
    -s ALLOW_MEMORY_GROWTH=1 ^
    -s MODULARIZE=1 ^
    -s EXPORT_NAME="createVOModule" ^
    -s EXPORTED_RUNTIME_METHODS="['ccall','cwrap']" ^
    -s TOTAL_MEMORY=67108864 ^
    --bind ^
    -O3 ^
    -std=c++17

if %ERRORLEVEL% NEQ 0 (
    echo Build failed!
    exit /b 1
)

echo.
echo Build complete!
echo Output: public/wasm/visual-odometry.js
echo Output: public/wasm/visual-odometry.wasm
echo.
echo To test: npm start
