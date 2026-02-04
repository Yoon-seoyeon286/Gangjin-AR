@echo off
REM ============================================================================
REM AR Vision Math Module Build Script (Windows)
REM Hello World급 빌드 - OpenCV 불필요
REM ============================================================================

echo Building AR Math Module...

REM Emscripten 환경 확인
if "%EMSDK%"=="" (
    echo Error: EMSDK environment variable not set.
    echo Please run 'emsdk_env.bat' first.
    echo Example: C:\emsdk\emsdk_env.bat
    exit /b 1
)

REM 출력 디렉토리 생성
if not exist "public\wasm" mkdir "public\wasm"

REM C++ 파일을 WebAssembly로 컴파일
call emcc ^
    src/cpp/math_bindings.cpp ^
    -o public/wasm/ar-math.js ^
    -I src/cpp ^
    -s WASM=1 ^
    -s ALLOW_MEMORY_GROWTH=1 ^
    -s MODULARIZE=1 ^
    -s EXPORT_NAME="createARMathModule" ^
    -s EXPORTED_RUNTIME_METHODS="['ccall','cwrap']" ^
    -s TOTAL_MEMORY=16777216 ^
    --bind ^
    -O3 ^
    -std=c++17

if %ERRORLEVEL% NEQ 0 (
    echo Build failed!
    exit /b 1
)

echo.
echo Build complete!
echo Output: public/wasm/ar-math.js
echo Output: public/wasm/ar-math.wasm
echo.
echo To test: npm start
