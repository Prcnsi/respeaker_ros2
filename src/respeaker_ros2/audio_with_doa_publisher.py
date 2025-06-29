#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, Int32  # Int32 메시지는 AudioDataWithDOA 정의에 사용됨
from respeaker_ros2.msg import AudioDataWithDOA

import pyaudio, numpy as np
import usb.core, usb.util
import struct
import threading
import queue
import re
import subprocess
import time
from statistics import median
import os

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
        self.get_logger().info("init called")
        self.pub = self.create_publisher(AudioDataWithDOA, 'audio_with_doa', 10)

        self.audio = pyaudio.PyAudio()
        self.rate = 16000
        self.channels = 6
        self.chunk = 1024
        self.device_index = None

        output = subprocess.check_output(['arecord', '-l']).decode()
        for i in range(self.p.get_device_count()):
            info = self.p.get_device_info_by_index(i)
            if 'ReSpeaker' in info['name']:
                self.device_index = i
                break
        if self.device_index is None:
            raise RuntimeError('ReSpeaker device not found')
        
        self.stream = self.audio.open(
            rate=self.rate,
            format=pyaudio.paInt16,
            channels=self.channels,
            input=True,
            input_device_index=self.device_index,
            frames_per_buffer=self.chunk
        )

        dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
        self.mic_tuning = Tuning(dev) if dev else None

        self.msg_queue = queue.Queue()
        self.doa_buffer = []  # (timestamp, doa) 리스트
        self.shutdown_flag = threading.Event()

        self.capture_thread = threading.Thread(target=self.read_audio_loop, daemon=True)
        self.publish_thread = threading.Thread(target=self.publish_loop, daemon=True)
        self.doa_thread = threading.Thread(target=self.read_doa_loop, daemon=True)

        self.capture_thread.start()
        self.publish_thread.start()
        self.doa_thread.start()

    def read_audio_loop(self):
        while not self.shutdown_flag.is_set():
            try:
                t_start = time.time()
                data = self.stream.read(self.chunk, exception_on_overflow=False)
                t_end = time.time()

                stamp_ros = self.get_clock().now().to_msg()
                pcm_data = np.frombuffer(data, dtype=np.int16)
                mono_data = pcm_data[0::self.channels]

                self.msg_queue.put((stamp_ros, t_start, t_end, mono_data))
            except Exception as e:
                self.get_logger().error(f'Read error: {e}')
                self.shutdown_flag.set()

    def read_doa_loop(self):
        while not self.shutdown_flag.is_set():
            try:
                t_now = time.time()
                doa = self.mic_tuning.direction if self.mic_tuning else 0
                self.doa_buffer.append((t_now, doa))
                # 오래된 값 제거 (10초 이상된 값 제거)
                self.doa_buffer = [(t, v) for t, v in self.doa_buffer if t_now - t <= 10.0]
                time.sleep(0.01)  # 100Hz 주기로 샘플링
            except Exception as e:
                self.get_logger().warn(f'DOA read error: {e}')

    def find_avg_doa(self, t_start, t_end):
        window = [v for t, v in self.doa_buffer if t_start <= t <= t_end]
        if not window:
            return 0
        return int(median(window))  # 중앙값 사용으로 더 안정적인 대표값 추정

    def publish_loop(self):
        while not self.shutdown_flag.is_set():
            try:
                stamp_ros, t_start, t_end, audio_data = self.msg_queue.get(timeout=1)
                doa_value = self.find_avg_doa(t_start, t_end)

                msg = AudioDataWithDOA()
                msg.header = Header()
                msg.header.stamp = stamp_ros
                msg.header.frame_id = 'respeaker_base'
                msg.audio = audio_data.tolist()
                msg.doa = Int32(data=doa_value)

                self.get_logger().info(f"[PUB] Publishing audio len={len(msg.audio)}, doa={doa_value}")
                self.pub.publish(msg)
            except queue.Empty:
                continue

    def destroy(self):
        self.shutdown_flag.set()
        self.capture_thread.join()
        self.publish_thread.join()
        self.doa_thread.join()

        if self.stream.is_active():
            self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()

        if self.mic_tuning:
            self.mic_tuning.close()

        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AudioWithDOAPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    p=pyaudio.PyAudio()
    dev_info=p.get_device_info_by_index(0)
    print(dev_info['name'], 'max_input_channel',dev_info['maxInputChannels'])
    main()
