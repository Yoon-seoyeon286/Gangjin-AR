export class CameraManager {
    constructor() {
        this.video = null; // 비디오 엘리먼트를 담을 변수
        this.stream = null; // 카메라 스트림 정보를 담을 변수
        this.useFrontCamera = false; // 기본은 후면
    }

    async init(forceFront = false) {
        this.useFrontCamera = forceFront; // 인자로 받은 값에 따라 카메라 방향 설정
        
        try {
            // 연결된 모든 미디어 장치(카메라, 마이크 등)를 나열함
            const devices = await navigator.mediaDevices.enumerateDevices();
            // 그 중 비디오 입력 장치(카메라)만 골라냄
            const videoDevices = devices.filter(d => d.kind === 'videoinput');
            
            // 디버깅을 위해 콘솔에 카메라 목록 출력
            console.log('카메라 목록:', videoDevices.map((d, i) => `${i}: ${d.label}`));
            
            // 카메라가 하나도 없으면 에러 발생
            if (videoDevices.length === 0) {
                throw new Error('카메라를 찾을 수 없습니다');
            }

            this.video = document.createElement('video');
            this.video.setAttribute('playsinline', '');
            this.video.setAttribute('autoplay', '');
            this.video.style.display = 'none';
            document.body.appendChild(this.video);

            // 전면/후면 선택
            const facingMode = this.useFrontCamera ? 'user' : 'environment';
            console.log(`${this.useFrontCamera ? '전면' : '후면'} 카메라 시도...`);
            
            try {
                this.stream = await navigator.mediaDevices.getUserMedia({
                    video: { 
                        facingMode: { ideal: facingMode },
                        width: { ideal: 1280 },
                        height: { ideal: 720 }
                    },
                    audio: false // 마이크는 사용하지 않음
                });
                console.log(` ${this.useFrontCamera ? '전면' : '후면'} 카메라 성공!`);
            } catch (e1) {
                console.log('facingMode ideal 실패, exact 시도...');
                try {
                    // 2단계: 1단계 실패 시, exact(강제) 설정을 사용하여 해당 방향 카메라 요청
                    this.stream = await navigator.mediaDevices.getUserMedia({
                        video: { facingMode: { exact: facingMode } },
                        audio: false
                    });
                    console.log(` ${this.useFrontCamera ? '전면' : '후면'} 카메라 exact 성공!`);
                } catch (e2) {
                    // 3단계: 특정 방향 카메라 호출이 모두 실패하면, 기본 카메라로 연결
                    console.log('기본 카메라로 폴백...');
                    this.stream = await navigator.mediaDevices.getUserMedia({
                        video: true,
                        audio: false
                    });
                    console.log('기본 카메라');
                }
            }

            // 비디오 엘리먼트의 소스로 카메라 스트림을 연결
            this.video.srcObject = this.stream;
            // 비디오 재생 시작 (실제 데이터 흐름 시작)
            await this.video.play();

            console.log('카메라 크기:', {
                width: this.video.videoWidth,
                height: this.video.videoHeight
            });

            return true;

        } catch (error) {
            console.error(' 카메라 초기화 실패:', error);
            return false;
        }
    }

    async switchCamera() {
        console.log('카메라 전환 시작...');
        
        // 기존 스트림 정지
        this.stop();
        
        // 전면/후면 토글
        this.useFrontCamera = !this.useFrontCamera;
        
        // 비디오 엘리먼트 제거
        if (this.video && this.video.parentNode) {
            this.video.parentNode.removeChild(this.video);
        }
        
        // 재초기화
        return await this.init(this.useFrontCamera);
    }

    // 현재 사용 중인 비디오 엘리먼트를 반환
    getVideo() {
        return this.video;
    }

    // 현재 카메라의 실제 해상도를 반환
    getSize() {
        return {
            width: this.video ? this.video.videoWidth : 0,
            height: this.video ? this.video.videoHeight : 0
        };
    }

    // 모든 카메라 트랙을 정지시켜 카메라 불빛(사용 중 표시)을 끔

    stop() {
        if (this.stream) {
            this.stream.getTracks().forEach(track => track.stop());
        }
    }
}