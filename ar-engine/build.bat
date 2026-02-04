@echo off

call C:\emsdk\emsdk_env.bat

if not exist "public\wasm" mkdir public\wasm

emcc src/cpp/feature_matcher.cpp src/cpp/pose_estimator.cpp src/cpp/ar_tracker.cpp src/cpp/bindings.cpp -o public/wasm/ar-engine.js -I src/cpp -s WASM=1 -s ALLOW_MEMORY_GROWTH=1 -s MODULARIZE=1 -s EXPORT_NAME="createARModule" -s EXPORTED_RUNTIME_METHODS="['ccall','cwrap']" --bind -O3

echo 빌드 완료!
pause