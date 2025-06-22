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
        # ROS2 Publisher for audio data with DOA
        self.publisher_ = self.create_publisher(AudioDataWithDOA, 'audio_with_doa', 10)
        # PyAudio initialization
        self.pyaudio = pyaudio.PyAudio()
        self.rate = 16000        # Sampling rate
        self.channels = 6        # Expect 6 channels (ReSpeaker with 6-channel firmware)
        self.chunk = 1024        # Frames per audio buffer
        # Determine input device (prefer ReSpeaker device)
        self.device_index = None
        for i in range(self.pyaudio.get_device_count()):
            try:
                info = self.pyaudio.get_device_info_by_index(i)
            except Exception:
                continue
            name = info.get('name', '').lower()
            if 'respeaker' in name:
                self.device_index = i
                # Use the device's channel count if available
                max_ch = info.get('maxInputChannels')
                if max_ch:
                    self.channels = int(max_ch)
                break
        if self.device_index is None:
            # Fallback to default input device
            try:
                default_info = self.pyaudio.get_default_input_device_info()
                self.device_index = int(default_info.get('index', 0))
                max_ch = default_info.get('maxInputChannels')
                if max_ch:
                    self.channels = int(max_ch)
                self.get_logger().warn('ReSpeaker device not found by name. Using default input device.')
            except Exception as e:
                self.get_logger().error(f'No audio input device available: {e}')
                raise RuntimeError("Audio input not available")
        # Open audio input stream
        try:
            self.stream = self.pyaudio.open(
                rate=self.rate,
                channels=self.channels,
                format=self.pyaudio.get_format_from_width(2),  # 16-bit audio
                input=True,
                frames_per_buffer=self.chunk,
                input_device_index=self.device_index
            )
        except Exception as e:
            self.get_logger().error(f'Failed to open audio stream: {e}')
            raise
        # Initialize Tuning for DOA
        try:
            self.tuning = find_respeaker_device()
        except Exception as e:
            self.get_logger().error(f'Error initializing Tuning interface: {e}')
            self.tuning = None
        if self.tuning is None:
            self.get_logger().warn('ReSpeaker device not connected or tuning unavailable. DOA will be set to -1.')
        # Start background thread for audio capture and publishing
        self._audio_thread = threading.Thread(target=self._audio_capture_loop, daemon=True)
        self._audio_thread.start()

    def _audio_capture_loop(self):
        """Continuously capture audio and publish AudioDataWithDOA messages."""
        while rclpy.ok():
            try:
                # Read a chunk of audio data
                data = self.stream.read(self.chunk, exception_on_overflow=False)
            except Exception as e:
                self.get_logger().error(f'Audio stream read error: {e}')
                break
            # Create and populate the AudioDataWithDOA message
            msg = AudioDataWithDOA()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'respeaker'
            # Get DOA value (direction of arrival)
            doa_value = -1
            if self.tuning is not None:
                try:
                    doa_value = int(self.tuning.direction)
                catch Exception as e:
                    self.get_logger().error(f'DOA read failed: {e}')
                    doa_value = -1
            msg.doa = doa_value
            # Attach raw audio data (as bytes)
            msg.data = bytes(data)
            # Publish the combined message
            self.publisher_.publish(msg)
        # If loop exits, clean up the audio stream
        try:
            self.stream.stop_stream()
            self.stream.close()
        except Exception:
            pass
        self.pyaudio.terminate()

def main(args=None):
    rclpy.init(args=args)
    node = AudioWithDOAPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure resources are cleaned up
        if node.tuning:
            node.tuning.close()
        node.destroy_node()
        rclpy.shutdown()
