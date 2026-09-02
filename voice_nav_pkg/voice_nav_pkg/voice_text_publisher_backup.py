import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sherpa_onnx
import sys
import numpy as np
import sounddevice as sd

class VoiceTextPublisher(Node):
    def __init__(self):
        super().__init__('voice_text_publisher')
        self.publisher_ = self.create_publisher(String, 'voice_command', 10)
        
        try:
            _engine = sherpa_onnx.lib._sherpa_onnx
            feat_config = _engine.FeatureExtractorConfig(sampling_rate=16000, feature_dim=80)
            
            # 모델 경로는 서영이 설정 그대로 유지
            model_path = "/home/jet/software/korean-model"
            
            model_config = _engine.OnlineModelConfig(
                transducer=_engine.OnlineTransducerModelConfig(
                    encoder=f"{model_path}/encoder-epoch-99-avg-1.onnx",
                    decoder=f"{model_path}/decoder-epoch-99-avg-1.onnx",
                    joiner=f"{model_path}/joiner-epoch-99-avg-1.onnx"
                ),
                tokens=f"{model_path}/tokens.txt",
                num_threads=1, debug=False, model_type='zipformer'
            )
            
            recon_config = _engine.OnlineRecognizerConfig(
                feat_config=feat_config, model_config=model_config,
                enable_endpoint=True, decoding_method='modified_beam_search', max_active_paths=4
            )

            self.recognizer = _engine.OnlineRecognizer(recon_config)
            self.stream = self.recognizer.create_stream()
            
            self.last_text = ""
            self.sd_stream = sd.InputStream(
                channels=1, samplerate=16000, dtype='float32',
                callback=self.audio_callback
            )
            self.sd_stream.start()
            
            self.get_logger().info(' 인식기 준비 완료! 어르신 말씀을 기다리는 중입니다.')

        except Exception as e:
            self.get_logger().error(f'초기화 실패: {e}')
            sys.exit(1)

    def audio_callback(self, indata, frames, time, status):
        """실시간 음성 인식 및 목적지 매핑 로직"""
        samples = indata.flatten()
        self.stream.accept_waveform(16000, samples)
        
        while self.recognizer.is_ready(self.stream):
            self.recognizer.decode_stream(self.stream)
        
        result = self.recognizer.get_result(self.stream).text.strip()
        
        if result and result != self.last_text:
            self.last_text = result
            self.get_logger().info(f' 인식 결과: {result}')
            
            # 목적지별 키워드 맵 (추상적 표현 대응)
            hospital_map = {
                "내과": ["내과", "배 아파", "속 안 좋", "감기", "기침", "으슬으슬", "열나", "소화"],
                "진료실": ["진료실", "선생님", "검사", "치료", "진찰"],
                "화장실": ["화장실", "급해", "물 내리는", "소변", "대변", "손 씻", "뒤간"],
                "접수처": ["접수", "등록", "처음", "번호표", "돈 내", "어디로 가", "카드", "수납"],
                "병실": ["병실", "입원", "면회", "누워", "병문안"]
            }
            
            found = False
            for location, keywords in hospital_map.items():
                for key in keywords:
                    if key in result:
                        msg = String()
                        msg.data = location
                        self.publisher_.publish(msg)
                        self.get_logger().info(f'[발송] 목적지: "{location}" (키워드: {key})')
                        
                        # 인식 성공 시 스트림을 새로 생성하여 뇌를 비워줌
                        self.stream = self.recognizer.create_stream()
                        self.last_text = ""
                        found = True
                        break
                if found: break

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
        rclpy.shutdown()

if __name__ == '__main__':
    main()
