FROM node:18-alpine

WORKDIR /app

# ar-engine 디렉토리로 이동
COPY ar-engine/package.json ./

# npm install 실행
RUN npm install --include=dev

# 소스 복사
COPY ar-engine/ ./

# webpack 빌드
RUN npm run build

# 불필요한 devDependencies 제거
RUN npm prune --production

# 포트 노출
EXPOSE 3000

# 서버 실행
CMD ["node", "server.js"]
