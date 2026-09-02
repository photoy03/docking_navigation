import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import sherpa_onnx
import sys
import time
import sounddevice as sd

import queue
import threading

from fuzzywuzzy import fuzz


class VoiceTextPublisher(Node):

    def __init__(self):
        super().__init__('voice_text_publisher')

        # ============================================================
        # ROS2 Publisher
        # ============================================================
        self.publisher_ = self.create_publisher(
            String,
            '/voice_command',
            10
        )

        # ============================================================
        # 실행 모드
        # ============================================================
        # False : 시연용 깔끔한 출력
        # True  : 중간 STT / 유사도 / TOP3 등 상세 출력
        self.debug_mode = False

        # ============================================================
        # 상태 변수
        # ============================================================
        self.is_awake = False

        self.awake_time = 0.0
        self.awake_timeout = 8.0

        self.last_partial_text = ""

        # ============================================================
        # Queue + Thread 설정
        # ============================================================
        # blocksize=3200, 16000Hz이므로
        # Queue 하나 = 약 0.2초 음성
        #
        # maxsize=30
        # → 최대 약 6초 정도까지 버퍼링 가능
        self.audio_queue = queue.Queue(maxsize=30)

        # STT Worker 실행 여부
        self.worker_running = True

        # Queue가 꽉 찬 횟수
        self.queue_drop_count = 0

        # sounddevice callback 경고 횟수
        self.audio_status_count = 0
        self.last_audio_status = ""

        # ============================================================
        # FuzzyWuzzy Threshold
        # ============================================================
        self.wake_threshold = 75

        self.location_threshold = {
            "내과": 60,
            "진료실": 60,
            "화장실": 60,
            "접수처": 60,
            "병실": 60,
        }

        # ============================================================
        # 호출어
        # ============================================================
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

        # ============================================================
        # 목적지별 키워드
        # ============================================================
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

                "어디로 가",
                "어디로가",
                "어디 가",
                "어디가",

                "카드",
                "수납",
                "결제",

                "진료 접수",
                "진료접수",

                "접수 어디",
                "접수어디",

                "접수하고 싶어요",
                "접수하고싶어요",
                "접수하려고요",

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

        # ============================================================
        # STT / Microphone 초기화
        # ============================================================
        try:

            # --------------------------------------------------------
            # Sherpa ONNX Engine
            # --------------------------------------------------------
            _engine = sherpa_onnx.lib._sherpa_onnx

            feat_config = _engine.FeatureExtractorConfig(
                sampling_rate=16000,
                feature_dim=80
            )

            # --------------------------------------------------------
            # Jetson 모델 경로
            # --------------------------------------------------------
            model_path = "/home/jet/software/korean-model"

            model_config = _engine.OnlineModelConfig(

                transducer=_engine.OnlineTransducerModelConfig(

                    encoder=(
                        f"{model_path}/"
                        "encoder-epoch-99-avg-1.onnx"
                    ),

                    decoder=(
                        f"{model_path}/"
                        "decoder-epoch-99-avg-1.onnx"
                    ),

                    joiner=(
                        f"{model_path}/"
                        "joiner-epoch-99-avg-1.onnx"
                    )
                ),

                tokens=f"{model_path}/tokens.txt",

                # ===================================================
                # CPU 추론
                #
                # 기존 1 → 2
                # ===================================================
                num_threads=2,

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

            self.recognizer = _engine.OnlineRecognizer(
                recon_config
            )

            self.stream = self.recognizer.create_stream()

            # ========================================================
            # ReSpeaker 자동 검색
            # ========================================================
            self.mic_device = self.find_respeaker()

            device_info = sd.query_devices(
                self.mic_device
            )

            sd.check_input_settings(
                device=self.mic_device,
                channels=1,
                samplerate=16000,
                dtype='float32'
            )

            # ========================================================
            # ★ STT Worker Thread 시작
            #
            # Sherpa 추론은 여기서부터
            # audio_callback과 완전히 분리됨
            # ========================================================
            self.stt_thread = threading.Thread(
                target=self.stt_worker,
                daemon=True
            )

            self.stt_thread.start()

            # ========================================================
            # Microphone Stream
            # ========================================================
            self.sd_stream = sd.InputStream(

                device=self.mic_device,

                channels=1,

                samplerate=16000,

                dtype='float32',

                blocksize=3200,

                latency='high',

                callback=self.audio_callback
            )

            self.sd_stream.start()

            self.get_logger().info(
                f'🎤 ReSpeaker 연결 완료: '
                f'device={self.mic_device}'
            )

            self.get_logger().info(
                '🧠 STT 추론: CPU / num_threads=2'
            )

            self.get_logger().info(
                '📦 Audio Queue + STT Worker Thread 활성화'
            )

            if self.debug_mode:

                self.get_logger().info(
                    f'🔧 장치 이름: '
                    f'{device_info["name"]}'
                )

                self.get_logger().info(
                    f'🔧 입력 채널: '
                    f'{device_info["max_input_channels"]}'
                )

            self.print_waiting_message()

        except Exception as e:

            self.worker_running = False

            self.get_logger().error(
                f'초기화 실패: {e}'
            )

            sys.exit(1)

    # ================================================================
    # DEBUG LOG
    # ================================================================
    def debug_log(self, message):

        if self.debug_mode:
            self.get_logger().info(message)

    # ================================================================
    # 대기 메시지
    # ================================================================
    def print_waiting_message(self):

        self.get_logger().info(
            '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
        )

        self.get_logger().info(
            '😴 [대기] '
            '"제트야" 또는 "로봇아"라고 불러주세요.'
        )

        self.get_logger().info(
            '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
        )

    # ================================================================
    # ReSpeaker 자동 검색
    # ================================================================
    def find_respeaker(self):

        devices = sd.query_devices()

        for index, device in enumerate(devices):

            name = device["name"]

            input_channels = (
                device["max_input_channels"]
            )

            if (
                "respeaker" in name.lower()
                and input_channels > 0
            ):

                return index

        self.get_logger().error(
            '❌ ReSpeaker 마이크를 찾지 못했습니다.'
        )

        if self.debug_mode:

            for index, device in enumerate(devices):

                if device["max_input_channels"] > 0:

                    self.get_logger().error(
                        f'[{index}] '
                        f'{device["name"]}'
                    )

        raise RuntimeError(
            'ReSpeaker 마이크가 연결되어 있지 않습니다.'
        )

    # ================================================================
    # 텍스트 정규화
    # ================================================================
    def normalize_text(self, text):

        text = text.strip()

        remove_chars = [
            " ",
            ".",
            ",",
            "?",
            "!",
            "~",
            "ㅋ",
            "ㅎ",
        ]

        for ch in remove_chars:

            text = text.replace(
                ch,
                ""
            )

        return text

    # ================================================================
    # FuzzyWuzzy 유사도
    # ================================================================
    def fuzzy_score(self, text, key):

        text_norm = self.normalize_text(text)
        key_norm = self.normalize_text(key)

        if not text_norm or not key_norm:
            return 0

        # ============================================================
        # 한 글자 오인식 방지
        # ============================================================
        if len(text_norm) < 2:
            return 0

        # ============================================================
        # 정확히 포함
        # ============================================================
        if key_norm in text_norm:
            return 100

        # ============================================================
        # 전체 유사도
        # ============================================================
        score_ratio = fuzz.ratio(
            text_norm,
            key_norm
        )

        # ============================================================
        # 짧은 문장에는 partial_ratio 사용 금지
        # ============================================================
        if len(text_norm) <= 2:
            return score_ratio

        # ============================================================
        # 긴 문장에는 partial_ratio
        # ============================================================
        score_partial = fuzz.partial_ratio(
            text_norm,
            key_norm
        )

        return max(
            score_ratio,
            score_partial
        )

    # ================================================================
    # 호출어 검색
    # ================================================================
    def contains_wake_word(self, text):

        best_wake = None
        best_score = 0

        for wake in self.wake_words:

            score = self.fuzzy_score(
                text,
                wake
            )

            if score > best_score:

                best_score = score
                best_wake = wake

        self.debug_log(
            f'🔎 호출어 BEST: '
            f'"{best_wake}" / '
            f'{best_score}'
        )

        detected = (
            best_score >= self.wake_threshold
        )

        return (
            detected,
            best_wake,
            best_score
        )

    # ================================================================
    # 호출어만 말한 건지 확인
    # ================================================================
    def is_wake_only(self, text):

        text_norm = self.normalize_text(
            text
        )

        (
            wake_detected,
            wake_word,
            wake_score
        ) = self.contains_wake_word(
            text
        )

        if not wake_detected:
            return False

        # "로봇", "로봇아", "제트야" 같은 짧은 호출
        if len(text_norm) <= 5:
            return True

        return False

    # ================================================================
    # 목적지 찾기
    # ================================================================
    def find_destination(self, text):

        best_location = None
        best_key = None
        best_score = 0

        top_results = []

        for location, keywords in self.hospital_map.items():

            for key in keywords:

                score = self.fuzzy_score(
                    text,
                    key
                )

                top_results.append(
                    (
                        location,
                        key,
                        score
                    )
                )

                if score > best_score:

                    best_score = score
                    best_location = location
                    best_key = key

        # ============================================================
        # Debug TOP3
        # ============================================================
        if self.debug_mode:

            top_results.sort(
                key=lambda x: x[2],
                reverse=True
            )

            self.get_logger().info(
                '--- 목적지 유사도 TOP 3 ---'
            )

            for (
                location,
                key,
                score
            ) in top_results[:3]:

                self.get_logger().info(
                    f'{location} / '
                    f'"{key}" / '
                    f'{score}'
                )

        threshold = (
            self.location_threshold.get(
                best_location,
                60
            )
        )

        if best_score >= threshold:

            return (
                best_location,
                best_key,
                best_score
            )

        return (
            None,
            best_key,
            best_score
        )

    # ================================================================
    # STT Stream Reset
    # ================================================================
    def reset_recognition_stream(self):

        # 중요:
        # 이 함수는 STT Worker Thread에서만 호출됨.
        #
        # 따라서 sounddevice callback과
        # Sherpa stream이 충돌하지 않음.

        self.stream = (
            self.recognizer.create_stream()
        )

        self.last_partial_text = ""

    # ================================================================
    # ROS2 목적지 Publish
    # ================================================================
    def publish_destination(
        self,
        location,
        key,
        score
    ):

        msg = String()

        msg.data = location

        self.publisher_.publish(msg)

        self.get_logger().info(
            f'🚀 [목적지 결정] {location}'
        )

        self.get_logger().info(
            f'📡 [ROS2 발송] '
            f'voice_command → "{location}"'
        )

        self.debug_log(
            f'🔧 매칭 키워드="{key}", '
            f'유사도={score}'
        )

    # ================================================================
    # 발화가 완전히 끝난 뒤 최종 처리
    # ================================================================
    def process_final_text(self, text):

        text = text.strip()

        if not text:
            return

        normalized = self.normalize_text(
            text
        )

        # ============================================================
        # 한 글자 결과 무시
        # ============================================================
        if len(normalized) < 2:

            self.debug_log(
                f'⏭️ 너무 짧은 최종 문장 무시: '
                f'[{text}]'
            )

            return

        # ============================================================
        # 1. 아직 호출되지 않은 상태
        # ============================================================
        if not self.is_awake:

            (
                wake_detected,
                wake_word,
                wake_score
            ) = self.contains_wake_word(
                text
            )

            # 호출어가 없는 평범한 대화
            if not wake_detected:

                self.debug_log(
                    f'😴 호출어 없음 → 무시: '
                    f'[{text}]'
                )

                return

            # ========================================================
            # 호출어 인식
            # ========================================================
            self.get_logger().info("")

            self.get_logger().info(
                f'🎤 [호출어 인식] {text}'
            )

            self.debug_log(
                f'🔧 호출어 키워드="{wake_word}", '
                f'유사도={wake_score}'
            )

            # ========================================================
            # 호출어 + 목적지를 한 번에 말한 경우
            #
            # 예:
            # "로봇아 화장실로 가줘"
            # ========================================================
            if not self.is_wake_only(text):

                (
                    location,
                    key,
                    score
                ) = self.find_destination(
                    text
                )

                if location:

                    self.get_logger().info(
                        f'🗣️ [최종 음성] {text}'
                    )

                    self.publish_destination(
                        location,
                        key,
                        score
                    )

                    self.is_awake = False

                    self.get_logger().info("")

                    self.print_waiting_message()

                    return

            # ========================================================
            # 호출어만 말한 경우
            # ========================================================
            self.is_awake = True

            self.awake_time = (
                time.monotonic()
            )

            self.get_logger().info(
                '✅ 호출 완료'
            )

            self.get_logger().info(
                '👉 목적지를 말씀해주세요.'
            )

            self.get_logger().info("")

            return

        # ============================================================
        # 2. 이미 호출된 상태
        # ============================================================
        elapsed = (
            time.monotonic()
            - self.awake_time
        )

        # ============================================================
        # Timeout
        # ============================================================
        if elapsed > self.awake_timeout:

            self.is_awake = False

            self.get_logger().info(
                '⏰ [시간 초과] '
                '다시 호출해주세요.'
            )

            self.print_waiting_message()

            return

        # ============================================================
        # 이미 깨어있는데 호출어가 또 들어온 경우
        #
        # "로봇아"
        # ↓
        # "로봇"
        #
        # 이런 경우 목적지 판정 X
        # ============================================================
        if self.is_wake_only(text):

            self.debug_log(
                f'🔁 호출어 반복: '
                f'[{text}] → 목적지 판정 안 함'
            )

            self.awake_time = (
                time.monotonic()
            )

            return

        # ============================================================
        # ★ 사용자가 말을 다 끝낸 뒤 목적지 판단
        # ============================================================
        self.get_logger().info(
            f'🗣️ [최종 음성] {text}'
        )

        (
            location,
            key,
            score
        ) = self.find_destination(
            text
        )

        # ============================================================
        # 등록된 목적지
        # ============================================================
        if location:

            self.publish_destination(
                location,
                key,
                score
            )

            self.is_awake = False

            self.get_logger().info("")

            self.print_waiting_message()

            return

        # ============================================================
        # 등록되지 않은 목적지
        # ============================================================
        self.get_logger().info(
            '❌ 찾으시는 목적지가 없습니다.'
        )

        # 사용자가 다시 말할 수 있도록 호출 상태 유지
        self.is_awake = True

        self.awake_time = (
            time.monotonic()
        )

    # ================================================================
    # ★ Audio Callback
    #
    # 여기서는 절대 Sherpa 추론을 하지 않음.
    #
    # 하는 일:
    #
    # ReSpeaker 음성
    #      ↓
    # Queue에 넣기
    #      ↓
    # 바로 종료
    #
    # ================================================================
    def audio_callback(
        self,
        indata,
        frames,
        time_info,
        status
    ):

        # ============================================================
        # callback 안에서는 logger도 최대한 사용하지 않음
        # ============================================================
        if status:

            self.audio_status_count += 1
            self.last_audio_status = str(status)

        # ============================================================
        # 음성 데이터 복사
        #
        # indata는 callback이 끝나면 다시 사용될 수 있기 때문에
        # 반드시 copy()
        # ============================================================
        audio_block = indata.copy()

        try:

            # ========================================================
            # 기다리지 않고 Queue에 즉시 저장
            #
            # put()이 아니라 put_nowait()
            #
            # → callback이 Queue 때문에 멈추는 것도 방지
            # ========================================================
            self.audio_queue.put_nowait(
                audio_block
            )

        except queue.Full:

            # ========================================================
            # Queue가 꽉 차면 callback을 멈추지 않고
            # 해당 block은 버림.
            #
            # 정상 상태에서는 여기에 거의 들어오면 안 됨.
            # ========================================================
            self.queue_drop_count += 1

    # ================================================================
    # ★ STT Worker Thread
    #
    # 무거운 작업은 전부 여기서 수행
    # ================================================================
    def stt_worker(self):

        self.debug_log(
            '🧵 STT Worker Thread 시작'
        )

        while self.worker_running:

            try:

                # ====================================================
                # Queue에서 음성 가져오기
                #
                # timeout을 주는 이유:
                # 종료할 때 thread가 영원히 대기하지 않도록
                # ====================================================
                audio_block = self.audio_queue.get(
                    timeout=0.5
                )

            except queue.Empty:

                # ====================================================
                # 호출 상태에서 사용자가 아무 말도 안 한 경우
                # Timeout 체크
                # ====================================================
                self.check_awake_timeout()

                continue

            # ========================================================
            # 종료용 Sentinel
            # ========================================================
            if audio_block is None:

                break

            try:

                # ====================================================
                # callback에서 넘어온 음성
                # ====================================================
                samples = (
                    audio_block.flatten()
                )

                # ====================================================
                # Sherpa에 전달
                # ====================================================
                self.stream.accept_waveform(
                    16000,
                    samples
                )

                # ====================================================
                # CPU 추론
                # ====================================================
                while self.recognizer.is_ready(
                    self.stream
                ):

                    self.recognizer.decode_stream(
                        self.stream
                    )

                # ====================================================
                # 현재 STT 결과
                # ====================================================
                result = (
                    self.recognizer
                    .get_result(
                        self.stream
                    )
                    .text
                    .strip()
                )

                # ====================================================
                # Debug 모드에서만 중간 STT
                # ====================================================
                if (
                    self.debug_mode
                    and result
                    and result
                    != self.last_partial_text
                ):

                    self.last_partial_text = (
                        result
                    )

                    self.get_logger().info(
                        f'📝 [중간 STT] '
                        f'{result}'
                    )

                # ====================================================
                # ★ 말 끝났는지 확인
                # ====================================================
                is_endpoint = (
                    self.recognizer
                    .is_endpoint(
                        self.stream
                    )
                )

                # ====================================================
                # 아직 말하는 중
                # ====================================================
                if not is_endpoint:

                    self.check_awake_timeout()

                    self.check_audio_queue_status()

                    continue

                # ====================================================
                # ★ 발화 종료
                # ====================================================
                final_text = result

                # 다음 발화를 위한 stream 초기화
                self.reset_recognition_stream()

                # ====================================================
                # 빈 발화
                # ====================================================
                if not final_text:

                    self.check_audio_queue_status()

                    continue

                # ====================================================
                # ★ 최종 문장 처리
                # ====================================================
                self.process_final_text(
                    final_text
                )

                # ====================================================
                # Queue 상태 확인
                # ====================================================
                self.check_audio_queue_status()

            except Exception as e:

                self.get_logger().error(
                    f'❌ STT Worker 오류: {e}'
                )

            finally:

                self.audio_queue.task_done()

        self.debug_log(
            '🧵 STT Worker Thread 종료'
        )

    # ================================================================
    # 호출 후 8초 Timeout 확인
    # ================================================================
    def check_awake_timeout(self):

        if not self.is_awake:
            return

        elapsed = (
            time.monotonic()
            - self.awake_time
        )

        if elapsed <= self.awake_timeout:
            return

        self.is_awake = False

        self.reset_recognition_stream()

        self.get_logger().info(
            '⏰ [시간 초과] '
            '목적지를 듣지 못했습니다.'
        )

        self.print_waiting_message()

    # ================================================================
    # Queue / Sounddevice 상태 확인
    # ================================================================
    def check_audio_queue_status(self):

        # ============================================================
        # Debug 모드일 때 Queue 크기 표시
        # ============================================================
        if self.debug_mode:

            queue_size = (
                self.audio_queue.qsize()
            )

            if queue_size > 0:

                self.get_logger().info(
                    f'📦 Audio Queue: '
                    f'{queue_size}/'
                    f'{self.audio_queue.maxsize}'
                )

        # ============================================================
        # Queue가 꽉 차서 음성을 버린 적이 있으면 알림
        # ============================================================
        if self.queue_drop_count > 0:

            count = self.queue_drop_count

            self.queue_drop_count = 0

            self.get_logger().warn(
                f'⚠️ STT 처리가 입력보다 느립니다. '
                f'Audio Queue에서 '
                f'{count}개 블록이 누락되었습니다.'
            )

        # ============================================================
        # PortAudio 상태 경고
        # ============================================================
        if self.audio_status_count > 0:

            count = self.audio_status_count
            status = self.last_audio_status

            self.audio_status_count = 0
            self.last_audio_status = ""

            self.get_logger().warn(
                f'⚠️ 오디오 입력 경고 '
                f'({count}회): {status}'
            )

    # ================================================================
    # 안전 종료
    # ================================================================
    def shutdown_audio(self):

        # ============================================================
        # 먼저 마이크 입력 정지
        # ============================================================
        if hasattr(
            self,
            'sd_stream'
        ):

            try:

                self.sd_stream.stop()
                self.sd_stream.close()

            except Exception:
                pass

        # ============================================================
        # Worker 종료
        # ============================================================
        self.worker_running = False

        # Queue에서 대기하고 있을 수 있으므로 깨워줌
        try:

            self.audio_queue.put_nowait(
                None
            )

        except queue.Full:

            # Queue가 꽉 찬 경우 하나 제거 후 종료 신호
            try:

                self.audio_queue.get_nowait()
                self.audio_queue.task_done()

            except queue.Empty:
                pass

            try:

                self.audio_queue.put_nowait(
                    None
                )

            except queue.Full:
                pass

        # Thread 종료 기다리기
        if hasattr(
            self,
            'stt_thread'
        ):

            self.stt_thread.join(
                timeout=2.0
            )


# ====================================================================
# MAIN
# ====================================================================
def main(args=None):

    rclpy.init(
        args=args
    )

    node = VoiceTextPublisher()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.shutdown_audio()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':

    main()