#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header  # Int32 메시지는 AudioDataWithDOA 정의에 사용됨
from respeaker_ros2.msg import AudioDataWithDOA

import pyaudio, numpy as np
import usb.core, usb.util
import struct
import threading

# ReSpeaker USB Mic Array의 DSP 파라미터 목록 (tuning.py의 PARAMETERS 정의 통합)
PARAMETERS = {
    'AECFREEZEONOFF': (18, 7, 'int', 1, 0, 'rw', 'Adaptive Echo Canceler updates inhibit.', 
                       '0 = Adaptation enabled', '1 = Freeze adaptation, filter only'),
    'AECNORM': (18, 19, 'float', 16, 0.25, 'rw', 'Limit on norm of AEC filter coefficients'),
    'AECPATHCHANGE': (18, 25, 'int', 1, 0, 'ro', 'AEC Path Change Detection.', 
                      '0 = false (no path change detected)', '1 = true (path change detected)'),
    'RT60': (18, 26, 'float', 0.9, 0.25, 'ro', 'Current RT60 estimate in seconds'),
    'HPFONOFF': (18, 27, 'int', 3, 0, 'rw', 'High-pass Filter on microphone signals.', 
                 '0 = OFF', '1 = ON - 70 Hz cut-off', '2 = ON - 125 Hz cut-off', '3 = ON - 180 Hz cut-off'),
    'RT60ONOFF': (18, 28, 'int', 1, 0, 'rw', 'RT60 Estimation for AEC. 0 = OFF, 1 = ON'),
    'AECSILENCELEVEL': (18, 30, 'float', 1, 1e-9, 'rw', 'Threshold for signal detection in AEC [-inf .. 0] dBov (default: -80dBov)'),
    'AECSILENCEMODE': (18, 31, 'int', 1, 0, 'ro', 'AEC far-end silence detection status.', 
                       '0 = false (signal detected)', '1 = true (silence detected)'),
    'AGCONOFF': (19, 0, 'int', 1, 0, 'rw', 'Automatic Gain Control.', '0 = OFF', '1 = ON'),
    'AGCMAXGAIN': (19, 1, 'float', 1000, 1, 'rw', 'Maximum AGC gain factor.', '[0 .. 60] dB (default: 30dB)'),
    'AGCDESIREDLEVEL': (19, 2, 'float', 0.99, 1e-8, 'rw', 'Target output signal level.', 
                        '[-inf .. 0] dBov (default: -23dBov)'),
    'AGCGAIN': (19, 3, 'float', 1000, 1, 'rw', 'Current AGC gain factor.', '[0 .. 60] dB (default: 0.0dB)'),
    'AGCTIME': (19, 4, 'float', 1, 0.1, 'rw', 'AGC ramp-up/down time constant (seconds).'),
    'CNIONOFF': (19, 5, 'int', 1, 0, 'rw', 'Comfort Noise Insertion.', '0 = OFF', '1 = ON'),
    'FREEZEONOFF': (19, 6, 'int', 1, 0, 'rw', 'Adaptive beamformer updates.', 
                    '0 = Adaptation enabled', '1 = Freeze adaptation (filter only)'),
    'STATNOISEONOFF': (19, 8, 'int', 1, 0, 'rw', 'Stationary noise suppression.', '0 = OFF', '1 = ON'),
    'GAMMA_NS': (19, 9, 'float', 3, 0, 'rw', 'Over-subtraction factor for stationary noise.'),
    'MIN_NS': (19, 10, 'float', 1, 0, 'rw', 'Gain-floor for stationary noise suppression.', 
               '[-inf .. 0] dB (default: -16dB)'),
    'NONSTATNOISEONOFF': (19, 11, 'int', 1, 0, 'rw', 'Non-stationary noise suppression.', '0 = OFF', '1 = ON'),
    'GAMMA_NN': (19, 12, 'float', 3, 0, 'rw', 'Over-subtraction factor for non-stationary noise.'),
    'MIN_NN': (19, 13, 'float', 1, 0, 'rw', 'Gain-floor for non-stationary noise suppression.', 
               '[-inf .. 0] dB (default: -10dB)'),
    'ECHOONOFF': (19, 14, 'int', 1, 0, 'rw', 'Echo suppression (AEC tail).', '0 = OFF', '1 = ON'),
    'GAMMA_E': (19, 15, 'float', 3, 0, 'rw', 'Over-subtraction factor of echo (direct/early components).'),
    'GAMMA_ETAIL': (19, 16, 'float', 3, 0, 'rw', 'Over-subtraction factor of echo (tail components).'),
    'GAMMA_ENL': (19, 17, 'float', 5, 0, 'rw', 'Over-subtraction factor of non-linear echo.'),
    'NLATTENONOFF': (19, 18, 'int', 1, 0, 'rw', 'Non-linear echo attenuation (NLA).', '0 = OFF', '1 = ON'),
    'NLAEC_MODE': (19, 20, 'int', 2, 0, 'rw', 'Non-linear AEC training mode.', 
                   '0 = OFF', '1 = ON - phase 1', '2 = ON - phase 2'),
    'SPEECHDETECTED': (19, 22, 'int', 1, 0, 'ro', 'Speech detection status.', 
                       '0 = false (no speech detected)', '1 = true (speech detected)'),
    'FSBUPDATED': (19, 23, 'int', 1, 0, 'ro', 'FSB (beamformer) update status.', 
                   '0 = false (not updated)', '1 = true (updated)'),
    'FSBPATHCHANGE': (19, 24, 'int', 1, 0, 'ro', 'FSB path change detected.', 
                      '0 = false', '1 = true'),
    'TRANSIENTONOFF': (19, 29, 'int', 1, 0, 'rw', 'Transient echo suppression.', '0 = OFF', '1 = ON'),
    'VOICEACTIVITY': (19, 32, 'int', 1, 0, 'ro', 'VAD voice activity status.', 
                      '0 = false (no voice)', '1 = true (voice detected)'),
    'STATNOISEONOFF_SR': (19, 33, 'int', 1, 0, 'rw', 'Stationary noise suppression (ASR output).', '0 = OFF', '1 = ON'),
    'NONSTATNOISEONOFF_SR': (19, 34, 'int', 1, 0, 'rw', 'Non-stationary noise suppression (ASR output).', '0 = OFF', '1 = ON'),
    'GAMMA_NS_SR': (19, 35, 'float', 3, 0, 'rw', 'Over-subtraction factor of stationary noise (ASR output).', 
                    '[0.0 .. 3.0] (default: 1.0)'),
    'GAMMA_NN_SR': (19, 36, 'float', 3, 0, 'rw', 'Over-subtraction factor of non-stationary noise (ASR output).', 
                    '[0.0 .. 3.0] (default: 1.1)'),
    'MIN_NS_SR': (19, 37, 'float', 1, 0, 'rw', 'Gain-floor for stationary noise suppression (ASR).', 
                  '[-inf .. 0] dB (default: -16dB)'),
    'MIN_NN_SR': (19, 38, 'float', 1, 0, 'rw', 'Gain-floor for non-stationary noise suppression (ASR).', 
                  '[-inf .. 0] dB (default: -10dB)'),
    'GAMMAVAD_SR': (19, 39, 'float', 1000, 0, 'rw', 'VAD threshold (for voice activity detection).', 
                    '[-inf .. 60] dB (default: 3.5dB)'),
    'DOAANGLE': (21, 0, 'int', 359, 0, 'ro', 'DOA angle (0-359°). Orientation depends on firmware configuration.')
}

class Tuning:
    """ReSpeaker USB Mic Array Tuning Interface - DSP parameter control"""
    TIMEOUT = 100000  # USB control transfer timeout (ms)
    def __init__(self, dev):
        self.dev = dev  # usb.core.Device instance
    
    def write(self, name, value):
        """Write a value to a DSP parameter by name."""
        try:
            data = PARAMETERS[name]
        except KeyError:
            return None  # Unknown parameter name
        if data[5] == 'ro':
            raise ValueError(f"{name} is read-only")
        # Prepare payload: 4 bytes offset, 4 bytes value, 4 bytes type flag (1=int, 0=float)
        if data[2] == 'int':
            payload = struct.pack('iii', data[1], int(value), 1)
        else:
            payload = struct.pack('ifi', data[1], float(value), 0)
        # Send control transfer (host-to-device)
        self.dev.ctrl_transfer(
            usb.util.CTRL_OUT | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, 0, data[0], payload, self.TIMEOUT
        )
        return True

    def read(self, name):
        """Read the current value of a DSP parameter by name."""
        try:
            data = PARAMETERS[name]
        except KeyError:
            return None  # Unknown parameter name
        param_id = data[0]
        cmd = 0x80 | data[1]               # Read command (bit7 set)
        if data[2] == 'int':
            cmd |= 0x40                    # int type flag (bit6 set)
        length = 8                         # expecting 8 bytes response (two 32-bit integers)
        # Send control transfer (device-to-host) to read parameter
        response = self.dev.ctrl_transfer(
            usb.util.CTRL_IN | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, cmd, param_id, length, self.TIMEOUT
        )
        # Unpack two 32-bit integers from response
        try:
            value, exponent = struct.unpack('ii', response.tobytes())
        except AttributeError:
            # PyUSB on Py3: if response is array('B') or bytes, convert to bytes
            value, exponent = struct.unpack('ii', bytes(response))
        # Interpret result based on type
        if data[2] == 'int':
            return value
        else:
            # float value = value * (2^exponent)
            return value * (2.0 ** exponent)

    def is_voice(self):
        """Check voice activity (VAD). Returns 1 if voice detected, else 0."""
        return self.read('VOICEACTIVITY')

    @property
    def direction(self):
        """Current DOA (direction of arrival) angle in degrees (0-359)."""
        return self.read('DOAANGLE')

    @property
    def version(self):
        """Firmware version byte (if available)."""
        resp = self.dev.ctrl_transfer(
            usb.util.CTRL_IN | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, 0x80, 0, 1, self.TIMEOUT
        )
        return resp[0] if resp else None

    def close(self):
        """Release the USB device resource."""
        usb.util.dispose_resources(self.dev)

def find_respeaker_device(vid=0x2886, pid=0x0018):
    """Find the ReSpeaker USB Mic Array device and return a Tuning interface object."""
    dev = usb.core.find(idVendor=vid, idProduct=pid)
    if not dev:
        return None
    return Tuning(dev)

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
            # 장치 연결 확인 <-- 수정된 부분
            if not usb.core.find(idVendor=0x2886, idProduct=0x0018):  # 장치 연결 해제 시 확인
                raise RuntimeError('ReSpeaker device disconnected')  # 장치 연결이 끊어졌을 때 예외 발생
       
            data = self.stream.read(self.chunk, exception_on_overflow=False)
        except Exception as e:
            self.get_logger().error(f'장치가 인식 안됨: {e}')
            self.destroy()  # <-- 연결이 끊어지면 리소스 정리 후 종료
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
