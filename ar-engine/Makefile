# ============================================================================
# AR Vision WebAssembly Makefile
# Emscripten을 사용한 C++ -> WebAssembly 빌드
# ============================================================================

# 디렉토리 설정
SRC_DIR = src/cpp
OUT_DIR = public/wasm

# 소스 파일들
# 수학 라이브러리만 빌드 (Hello World용)
MATH_SOURCES = $(SRC_DIR)/math_bindings.cpp

# Visual Odometry (SLAM 코어) 빌드용
VO_SOURCES = $(SRC_DIR)/visual_odometry.cpp \
             $(SRC_DIR)/vo_bindings.cpp

# 전체 AR 엔진 빌드용
AR_SOURCES = $(SRC_DIR)/slam_system.cpp \
             $(SRC_DIR)/plane_detector.cpp \
             $(SRC_DIR)/image_target.cpp \
             $(SRC_DIR)/ar_tracker.cpp \
             $(SRC_DIR)/bindings.cpp

# OpenCV 경로 (환경에 맞게 수정)
OPENCV_INCLUDE = lib/opencv/include
OPENCV_LIB = lib/opencv/build/lib

# 공통 컴파일 플래그
COMMON_FLAGS = -std=c++17 \
               -s WASM=1 \
               -s ALLOW_MEMORY_GROWTH=1 \
               -s MODULARIZE=1 \
               -s EXPORTED_RUNTIME_METHODS="['ccall','cwrap']" \
               --bind

# 개발 빌드 플래그 (디버깅 용이)
DEV_FLAGS = $(COMMON_FLAGS) \
            -O0 \
            -g \
            -s ASSERTIONS=2 \
            -s SAFE_HEAP=1 \
            -s STACK_OVERFLOW_CHECK=2

# 프로덕션 빌드 플래그 (최적화)
PROD_FLAGS = $(COMMON_FLAGS) \
             -O3 \
             -flto \
             -s ASSERTIONS=0

# 기본 타겟
.PHONY: all clean math math-dev vo vo-dev ar ar-dev help

all: math

# ============================================================================
# Hello World / 수학 라이브러리 빌드 (OpenCV 불필요)
# ============================================================================

# 프로덕션 빌드
math:
	@echo "Building math module (production)..."
	@mkdir -p $(OUT_DIR)
	emcc $(MATH_SOURCES) \
		-o $(OUT_DIR)/ar-math.js \
		-I $(SRC_DIR) \
		-s EXPORT_NAME="createARMathModule" \
		-s TOTAL_MEMORY=16777216 \
		$(PROD_FLAGS)
	@echo "Build complete: $(OUT_DIR)/ar-math.js"

# 개발 빌드 (디버깅 심볼 포함)
math-dev:
	@echo "Building math module (development)..."
	@mkdir -p $(OUT_DIR)
	emcc $(MATH_SOURCES) \
		-o $(OUT_DIR)/ar-math.js \
		-I $(SRC_DIR) \
		-s EXPORT_NAME="createARMathModule" \
		-s TOTAL_MEMORY=16777216 \
		$(DEV_FLAGS)
	@echo "Build complete: $(OUT_DIR)/ar-math.js (debug)"

# ============================================================================
# Visual Odometry (SLAM 코어) 빌드 (OpenCV 필요)
# ============================================================================

# 프로덕션 빌드
vo:
	@echo "Building Visual Odometry module (production)..."
	@mkdir -p $(OUT_DIR)
	emcc $(VO_SOURCES) \
		-o $(OUT_DIR)/visual-odometry.js \
		-I $(SRC_DIR) \
		-I $(OPENCV_INCLUDE) \
		-L $(OPENCV_LIB) \
		-s EXPORT_NAME="createVOModule" \
		-s TOTAL_MEMORY=67108864 \
		$(PROD_FLAGS)
	@echo "Build complete: $(OUT_DIR)/visual-odometry.js"

# 개발 빌드
vo-dev:
	@echo "Building Visual Odometry module (development)..."
	@mkdir -p $(OUT_DIR)
	emcc $(VO_SOURCES) \
		-o $(OUT_DIR)/visual-odometry.js \
		-I $(SRC_DIR) \
		-I $(OPENCV_INCLUDE) \
		-L $(OPENCV_LIB) \
		-s EXPORT_NAME="createVOModule" \
		-s TOTAL_MEMORY=67108864 \
		$(DEV_FLAGS)
	@echo "Build complete: $(OUT_DIR)/visual-odometry.js (debug)"

# ============================================================================
# 전체 AR 엔진 빌드 (OpenCV 필요)
# ============================================================================

# 프로덕션 빌드
ar:
	@echo "Building AR engine (production)..."
	@mkdir -p $(OUT_DIR)
	emcc $(AR_SOURCES) \
		-o $(OUT_DIR)/ar-engine.js \
		-I $(SRC_DIR) \
		-I $(OPENCV_INCLUDE) \
		-L $(OPENCV_LIB) \
		-s EXPORT_NAME="createARModule" \
		-s TOTAL_MEMORY=134217728 \
		$(PROD_FLAGS)
	@echo "Build complete: $(OUT_DIR)/ar-engine.js"

# 개발 빌드
ar-dev:
	@echo "Building AR engine (development)..."
	@mkdir -p $(OUT_DIR)
	emcc $(AR_SOURCES) \
		-o $(OUT_DIR)/ar-engine.js \
		-I $(SRC_DIR) \
		-I $(OPENCV_INCLUDE) \
		-L $(OPENCV_LIB) \
		-s EXPORT_NAME="createARModule" \
		-s TOTAL_MEMORY=134217728 \
		$(DEV_FLAGS)
	@echo "Build complete: $(OUT_DIR)/ar-engine.js (debug)"

# ============================================================================
# 유틸리티
# ============================================================================

clean:
	@echo "Cleaning build outputs..."
	rm -rf $(OUT_DIR)/*.js $(OUT_DIR)/*.wasm $(OUT_DIR)/*.wasm.map
	@echo "Clean complete."

help:
	@echo ""
	@echo "AR Vision WebAssembly Build System"
	@echo "==================================="
	@echo ""
	@echo "Prerequisites:"
	@echo "  - Emscripten SDK installed and activated"
	@echo "  - Run 'source /path/to/emsdk/emsdk_env.sh' first"
	@echo ""
	@echo "Available targets:"
	@echo ""
	@echo "  make math      - Build math module (production, no OpenCV needed)"
	@echo "  make math-dev  - Build math module (development, with debug info)"
	@echo "  make vo        - Build Visual Odometry / SLAM core (requires OpenCV)"
	@echo "  make vo-dev    - Build Visual Odometry (development)"
	@echo "  make ar        - Build full AR engine (requires OpenCV)"
	@echo "  make ar-dev    - Build full AR engine (development)"
	@echo "  make clean     - Remove build outputs"
	@echo "  make help      - Show this help message"
	@echo ""
	@echo "Quick start:"
	@echo "  1. Install Emscripten: https://emscripten.org/docs/getting_started/"
	@echo "  2. source /path/to/emsdk/emsdk_env.sh"
	@echo "  3. make math"
	@echo "  4. npm start"
	@echo ""
