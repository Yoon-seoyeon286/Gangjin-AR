# AR Vision

Web AR 엔진 개발 프로젝트

## 기술 스택

- Three.js - 3D 렌더링
- OpenCV.js - 컴퓨터 비전
- WebAssembly - 고성능 처리
- Webpack - 모듈 번들링

## 설치
```bash
npm install
```

## 실행
```bash
npm run dev
```

브라우저에서 `http://localhost:8080` 접속

## 프로젝트 구조
```
ar-engine/
├── src/
│   ├── main.js          # 메인 진입점
│   └── camera.js        # 카메라 관리
├── public/
│   ├── index.html
│   └── wasm/
│       └── opencv.js    # OpenCV.js (다운로드 필요)
└── webpack.config.js
```

## 개발 로드맵

- [x] Phase 1: 환경 구축
- [x] Phase 2: Three.js 렌더링
- [x] Phase 3: 카메라 통합
- [x] Phase 4: Feature Detection
- [ ] Phase 5: 평면 감지
- [ ] Phase 6: 객체 배치



## Phase 1 완료 확인현재 작동하는 기능:

- WebAssembly 모듈 로딩 ✓
- 카메라 스트림 받기 ✓
- Three.js 3D 렌더링 ✓
- AR 엔진과 연동 ✓
- 회전하는 큐브 표시 ✓