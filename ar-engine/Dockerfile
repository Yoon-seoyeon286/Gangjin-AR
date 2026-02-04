FROM node:18-alpine

WORKDIR /app

# 패키지 파일 복사
COPY package.json ./

# npm install 실행 (package-lock.json 생성 포함)
RUN npm install --include=dev

# 소스 복사
COPY . .

# webpack 빌드
RUN npm run build

# 불필요한 devDependencies 제거 (프로덕션용)
RUN npm prune --production

# 포트 노출
EXPOSE 3000

# 서버 실행
CMD ["node", "server.js"]
