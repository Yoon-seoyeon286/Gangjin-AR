#!/bin/bash

# Emscripten 컴파일러 경로 설정
if [ -z "$EMSDK" ]; then
  echo "Error: EMSDK environment variable not set. Please run 'source /path/to/emsdk/emsdk_env.sh' first."
  exit 1
fi

# 출력 디렉토리 생성
mkdir -p public/wasm

# OpenCV 경로 (설치된 경로에 맞게 수정)
OPENCV_INCLUDE="lib/opencv/include"
OPENCV_LIB="lib/opencv/build/lib"

# C++ 파일들을 WebAssembly로 컴파일
emcc \
  src/cpp/slam_system.cpp \
  src/cpp/plane_detector.cpp \
  src/cpp/image_target.cpp \
  src/cpp/ar_tracker.cpp \
  src/cpp/bindings.cpp \
  -o public/wasm/ar-engine.js \
  -I src/cpp \
  -I $OPENCV_INCLUDE \
  -L $OPENCV_LIB \
  -s WASM=1 \
  -s ALLOW_MEMORY_GROWTH=1 \
  -s MODULARIZE=1 \
  -s EXPORT_NAME="createARModule" \
  -s EXPORTED_RUNTIME_METHODS="['ccall','cwrap']" \
  -s TOTAL_MEMORY=134217728 \
  --bind \
  -O3 \
  -std=c++17

echo "빌드 완료!"
