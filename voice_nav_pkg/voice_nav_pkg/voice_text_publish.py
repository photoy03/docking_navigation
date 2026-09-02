import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import sherpa_onnx
import sys
import time
import sounddevice as sd

from fuzzywuzzy import fuzz


class VoiceTextPublisher(Node):
    def __init__(self):
        super().__init__('voice_text_publisher')

        self.publisher_ = self.create_publisher(String, 'voice_command', 10)

        # =========================
        # 호출어 사용 여부
        # =========================
        # 연구실 테스트할 때는 False
        # 데모할 때 "제트야/로봇아" 쓰려면 True
        self.use_wake_word = False

        # =========================
        # 상태 변수
        # =========================
        self.is_awake = False
        self.last_text = ""
        self.awake_time = 0.0
        self.awake_timeout = 8.0

        # =========================
        # FuzzyWuzzy 기준값
        # =========================
        # 전체적으로 기준 낮춘 버전
        self.default_threshold = 60

        # 접수처가 잘 안 잡혀서 접수처만 더 낮게 설정
        self.location_threshold = {
            "내과": 60,
            "진료실": 60,
            "화장실": 60,
            "접수처": 50,
            "병실": 60,
        }

        # 호출어 기준
        self.wake_threshold = 75

        # 호출어 후보
        self.wake_words = [
            "제트야",
            "제트",
            "재트야",
            "젯트야",
            "제트 야",
            "재트 야",
            "로봇아",
            "로봇",
            "안내로봇",
            "안내 로봇",
            "저기요",
            "여기요",
            "도와줘",
        ]

        # 목적지별 키워드 맵
        self.hospital_map = {
            "내과": [
                "내과",
                "배 아파",
                "배아파",
                "복통",
                "속 안 좋",
                "속안좋",
                "감기",
                "기침",
                "으슬으슬",
                "열나",
                "소화",
                "몸살",
            ],
            "진료실": [
                "진료실",
                "선생님",
                "검사",
                "치료",
                "진찰",
                "진료",
                "의사",
                "의사 선생님",
            ],
            "화장실": [
                "화장실",
                "화쟝실",
                "화장실이요",
                "급해",
                "소변",
                "대변",
                "손 씻",
                "손씻",
                "뒤간",
            ],
            "접수처": [
                "접수",
                "접수처",
                "접수처요",
                "접수 창구",
                "접수창구",
                "접수 데스크",
                "접수데스크",
                "안내 데스크",
                "안내데스크",
                "원무과",
                "등록",
                "처음",
                "처음 왔어요",
                "처음왔어요",
                "처음이에요",
                "처음이예요",
                "번호표",
                "번호표 어디",
                "번호표어디",
                "돈 내",
                "돈내",
                "수납",
                "결제",
                "카드",
                "어디로 가",
                "어디로가",
                "어디 가",
                "어디가",
                "접수 어디",
                "접수어디",
                "접수하고 싶어요",
                "접수하고싶어요",
                "접수하려고요",
                "접수 할게요",
                "접수할게요",
                "진료 접수",
                "진료접수",
                "예약 확인",
                "예약확인",
            ],
            "병실": [
                "병실",
                "입원",
                "면회",
                "누워",
                "병문안",
            ],
        }

        try:
            _engine = sherpa_onnx.lib._sherpa_onnx

            feat_config = _engine.FeatureExtractorConfig(
                sampling_rate=16000,
                feature_dim=80
            )

            model_path = "/home/seoyeong/software/korean-model"

            model_config = _engine.OnlineModelConfig(
                transducer=_engine.OnlineTransducerModelConfig(
                    encoder=f"{model_path}/encoder-epoch-99-avg-1.onnx",
                    decoder=f"{model_path}/decoder-epoch-99-avg-1.onnx",
                    joiner=f"{model_path}/joiner-epoch-99-avg-1.onnx"
                ),
                tokens=f"{model_path}/tokens.txt",
                num_threads=1,
                debug=False,
                model_type='zipformer'
            )

            recon_config = _engine.OnlineRecognizerConfig(
                feat_config=feat_config,
                model_config=model_config,
                enable_endpoint=True,
                decoding_method='modified_beam_search',
                max_active_paths=4
            )

            self.recognizer = _engine.OnlineRecognizer(recon_config)
            self.stream = self.recognizer.create_stream()

            self.sd_stream = sd.InputStream(
                device=6,
                channels=1,
                samplerate=16000,
                dtype='float32',
                callback=self.audio_callback
            )

            self.sd_stream.start()

            if self.use_wake_word:
                self.get_logger().info('😴 대기 중... "제트야" 또는 "로봇아"라고 불러주세요!')
            else:
                self.get_logger().info('🎧 호출어 OFF 테스트 모드')
                self.get_logger().info('목적지만 말하면 됩니다. 예: "화장실", "내과", "접수 어디예요"')

        except Exception as e:
            self.get_logger().error(f'초기화 실패: {e}')
            sys.exit(1)

    # =========================
    # 텍스트 전처리
    # =========================
    def normalize_text(self, text):
        """
        띄어쓰기, 문장부호를 제거해서 비교 안정성을 높임
        """
        text = text.strip()

        remove_chars = [" ", ".", ",", "?", "!", "~", "ㅋ", "ㅎ"]
        for ch in remove_chars:
            text = text.replace(ch, "")

        return text

    # =========================
    # FuzzyWuzzy 유사도 계산
    # =========================
    def fuzzy_score(self, text, key):
        """
        STT 결과와 키워드 사이의 유사도 계산
        """
        text_norm = self.normalize_text(text)
        key_norm = self.normalize_text(key)

        if not text_norm or not key_norm:
            return 0

        # 키워드가 그대로 포함되면 100점
        if key_norm in text_norm:
            return 100

        score1 = fuzz.ratio(text_norm, key_norm)
        score2 = fuzz.partial_ratio(text_norm, key_norm)

        return max(score1, score2)

    # =========================
    # 호출어 인식
    # =========================
    def contains_wake_word(self, text):
        """
        호출어 유사도 기반 인식
        """
        best_wake = None
        best_score = 0

        for wake in self.wake_words:
            score = self.fuzzy_score(text, wake)

            if score > best_score:
                best_score = score
                best_wake = wake

        if best_score >= self.wake_threshold:
            return True, best_wake, best_score

        return False, best_wake, best_score

    # =========================
    # 목적지 찾기
    # =========================
    def find_destination(self, text):
        """
        STT 결과에서 가장 유사한 목적지 찾기
        """
        best_location = None
        best_key = None
        best_score = 0

        top_results = []

        for location, keywords in self.hospital_map.items():
            for key in keywords:
                score = self.fuzzy_score(text, key)
                top_results.append((location, key, score))

                if score > best_score:
                    best_score = score
                    best_location = location
                    best_key = key

        top_results.sort(key=lambda x: x[2], reverse=True)

        self.get_logger().info('--- 유사도 TOP 3 ---')
        for location, key, score in top_results[:3]:
            self.get_logger().info(
                f'{location} / 키워드: "{key}" / 유사도: {score}'
            )

        # 목적지별 기준값 가져오기
        threshold = self.location_threshold.get(
            best_location,
            self.default_threshold
        )

        if best_score >= threshold:
            return best_location, best_key, best_score

        return None, best_key, best_score

    # =========================
    # 인식 스트림 초기화
    # =========================
    def reset_recognition_stream(self):
        self.stream = self.recognizer.create_stream()
        self.last_text = ""

    # =========================
    # 목적지 발행
    # =========================
    def publish_destination(self, location, key, score):
        msg = String()
        msg.data = location
        self.publisher_.publish(msg)

        self.get_logger().info(
            f'🚀 [발송] 목적지: "{location}" / 키워드: "{key}" / 유사도: {score}'
        )

    # =========================
    # 실시간 음성 인식 콜백
    # =========================
    def audio_callback(self, indata, frames, time_info, status):
        if status:
            self.get_logger().warn(f"오디오 상태 경고: {status}")

        samples = indata.flatten()

        self.stream.accept_waveform(16000, samples)

        while self.recognizer.is_ready(self.stream):
            self.recognizer.decode_stream(self.stream)

        result = self.recognizer.get_result(self.stream).text.strip()

        if not result:
            return

        if result == self.last_text:
            return

        self.last_text = result
        self.get_logger().info(f'📝 RAW 인식 결과: [{result}]')

        # =========================
        # 1. 호출어 OFF 테스트 모드
        # =========================
        if not self.use_wake_word:
            location, key, score = self.find_destination(result)

            if location:
                self.publish_destination(location, key, score)
                self.reset_recognition_stream()
                return

            self.get_logger().info(
                f'🤔 목적지를 찾지 못했습니다. 가장 가까운 키워드: "{key}" / 유사도: {score}'
            )
            self.reset_recognition_stream()
            return

        # =========================
        # 2. 호출어 ON 모드
        # =========================
        if not self.is_awake:
            wake_ok, wake_key, wake_score = self.contains_wake_word(result)

            if wake_ok:
                self.is_awake = True
                self.awake_time = time.monotonic()

                self.get_logger().info(
                    f'✅ 호출어 인식됨! 호출어: "{wake_key}" / 유사도: {wake_score}'
                )

                # "로봇아 화장실"처럼 한 문장에 목적지도 같이 있는 경우
                location, key, score = self.find_destination(result)

                if location:
                    self.publish_destination(location, key, score)

                    self.is_awake = False
                    self.reset_recognition_stream()

                    self.get_logger().info('😴 다시 대기 중... "제트야"라고 불러주세요!')
                    return

                self.get_logger().info('이제 목적지를 말씀해주세요.')
                self.get_logger().info('예: "내과", "화장실", "접수처", "배 아파요"')

                self.reset_recognition_stream()

            return

        # =========================
        # 3. 깨어난 상태인데 시간이 너무 오래 지난 경우
        # =========================
        elapsed = time.monotonic() - self.awake_time

        if elapsed > self.awake_timeout:
            self.get_logger().info('⏰ 명령 시간이 초과되어 다시 대기 상태로 돌아갑니다.')
            self.is_awake = False
            self.reset_recognition_stream()
            self.get_logger().info('😴 대기 중... "제트야"라고 불러주세요!')
            return

        # =========================
        # 4. 깨어난 상태에서 목적지 찾기
        # =========================
        location, key, score = self.find_destination(result)

        if location:
            self.publish_destination(location, key, score)

            self.is_awake = False
            self.reset_recognition_stream()

            self.get_logger().info('😴 다시 대기 중... "제트야"라고 불러주세요!')
            return

        self.get_logger().info(
            f'🤔 목적지를 찾지 못했습니다. 가장 가까운 키워드: "{key}" / 유사도: {score}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = VoiceTextPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'sd_stream'):
            node.sd_stream.stop()
            node.sd_stream.close()

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
