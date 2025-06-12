import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, ByteMultiArray
import pyaudio
import numpy as np

# ReSpeaker usb_4_mic_array 패키지의 tuning.py에 포함된 함수
from tuning import find as find_respeaker


class AudioDoaPublisher(Node):
    def __init__(self):
        super().__init__('audio_doa_publisher')

        # ROS 2 publishers
        self.pub_audio = self.create_publisher(ByteMultiArray, 'respeaker/audio0', 10)
        self.pub_doa = self.create_publisher(Int32, 'respeaker/doa', 10)

        # PyAudio setup
        self.rate = 16000
        self.channels = 6
        self.width = 2  # 16-bit PCM
        self.chunk = 1024
        self.device_index = 2  # ← get from get_index.py

        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(
            rate=self.rate,
            format=self.p.get_format_from_width(self.width),
            channels=self.channels,
            input=True,
            input_device_index=self.device_index
        )

        # ReSpeaker tuning interface (for DOA)
        self.tuner = find_respeaker()

        # Create timer for periodic callback
        self.timer = self.create_timer(self.chunk / self.rate, self.timer_callback)

    def timer_callback(self):
        # Read audio chunk
        data = self.stream.read(self.chunk, exception_on_overflow=False)
        arr = np.frombuffer(data, dtype=np.int16)

        # Extract channel 0 data (1 of 6)
        ch0 = arr[0::self.channels]

        # Publish channel 0 audio
        audio_msg = ByteMultiArray()
        audio_msg.data = ch0.tobytes()
        self.pub_audio.publish(audio_msg)

        # Publish DOA (direction)
        if self.tuner:
            doa_msg = Int32()
            doa_msg.data = int(self.tuner.direction)
            self.pub_doa.publish(doa_msg)

    def destroy_node(self):
        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()
        super().destroy_node()


def main():
    rclpy.init()
    node = AudioDoaPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
