import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Header
from <PACKAGE_NAME>.msg import AudioDataWithDOA  # 패키지명 변경

import pyaudio, numpy as np
import usb.core, usb.util
from tuning import Tuning  # ReSpeaker의 Tuning 클래스 (usb_4_mic_array 패키지 제공)

class AudioWithDOAPublisher(Node):
    def __init__(self):
        super().__init__('audio_with_doa_publisher')
        # 퍼블리셔 생성
        self.pub = self.create_publisher(AudioDataWithDOA, 'audio_with_doa', 10)
        # PyAudio 설정
        self.audio = pyaudio.PyAudio()
        self.rate = 16000
        self.channels = 6
        self.chunk = 1024
        # ReSpeaker 장치 인덱스 탐색 (이름으로 검색)
        self.device_index = None
        for i in range(self.audio.get_device_count()):
            dev_info = self.audio.get_device_info_by_index(i)
            name = dev_info.get('name','').lower()
            if 'respeaker' in name:
                self.device_index = i
                break
        if self.device_index is None:
            self.get_logger().error('ReSpeaker 마이크 장치를 찾을 수 없습니다.')
            raise RuntimeError('ReSpeaker device not found')
        # 오디오 스트림 열기
        self.stream = self.audio.open(
            rate=self.rate,
            format=pyaudio.paInt16,
            channels=self.channels,
            input=True,
            input_device_index=self.device_index,
            frames_per_buffer=self.chunk
        )
        self.get_logger().info(f'ReSpeaker 스트림 시작 (device index={self.device_index}, rate={self.rate}Hz)...')
        # USB 장치 초기화 (DOA용)
        dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
        if not dev:
            self.get_logger().error('ReSpeaker USB Mic Array 장치를 찾을 수 없습니다 (DOA 불가).')
            # DOA 사용 불가이므로 dev 없이 진행 (필요시 return이나 예외 처리)
        else:
            self.mic_tuning = Tuning(dev)
        # 타이머 설정: 주기적으로 _publish_audio 호출
        timer_period =  float(self.chunk) / float(self.rate)  # 청크 당 소요 시간(초)
        self.timer = self.create_timer(timer_period, self._publish_audio)

    def _publish_audio(self):
        try:
            data = self.stream.read(self.chunk, exception_on_overflow=False)
        except Exception as e:
            self.get_logger().error(f'오디오 스트림 읽기 오류: {e}')
            return
        # 6채널 데이터를 numpy로 변환하여 channel 0 추출
        pcm_data = np.frombuffer(data, dtype=np.int16)
        mono_data = pcm_data[0::self.channels]  # channel 0 데이터 추출:contentReference[oaicite:9]{index=9}
        # DOA 값 읽기
        doa_value = None
        if hasattr(self, 'mic_tuning'):
            try:
                doa_value = int(self.mic_tuning.direction)  # Int32로 변환
            except Exception as e:
                self.get_logger().warn(f'DOA 읽기 실패: {e}')
                doa_value = 0
        else:
            doa_value = 0  # 장치 미발견시 0으로 처리 또는 이전 값 유지
        # 메시지 구성 및 퍼블리시
        msg = AudioDataWithDOA()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'respeaker_base'
        msg.audio = mono_data.tolist()  # numpy array -> list[int]
        msg.doa = Int32(data=doa_value if doa_value is not None else 0)
        self.pub.publish(msg)
        # (옵션) 로그 출력
        self.get_logger().debug(f'Published audio chunk with {len(msg.audio)} samples, DOA={msg.doa.data}°')

    def destroy(self):
        # 리소스 정리
        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()
        self.get_logger().info('오디오 스트림을 종료하였습니다.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AudioWithDOAPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('노드 종료 신호 수신 (Ctrl+C)')
    finally:
        # 노드 및 리소스 정리
        node.destroy()
        rclpy.shutdown()
