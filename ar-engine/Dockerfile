FROM node:18-alpine

WORKDIR /app

# 의존성 설치 (webpack 빌드를 위해 devDependencies 포함)
COPY package.json package-lock.json* ./
RUN npm install --include=dev

# 소스 복사
COPY . .

# webpack 빌드
RUN npm run build

# 포트 노출
EXPOSE 3000

# 서버 실행
CMD ["node", "server.js"]
