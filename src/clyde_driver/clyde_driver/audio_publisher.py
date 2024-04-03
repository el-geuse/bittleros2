import rclpy
from rclpy.node import Node
import sounddevice as sd
import wave
import tempfile
from clyde_msgs.msg import WavFile


class AudioPublisher(Node):
    def __init__(self):
        super().__init__('audio_publisher')
        self.publisher_ = self.create_publisher(WavFile, '/audio_files', 10)
        self.duration = 1
        self.timer = self.create_timer(self.duration, self.capture_and_publish)
        self.sample_rate = 44100

    def capture_and_publish(self):
        self.get_logger().info('Capturing audio...')
        audio_data = sd.rec(int(self.duration * self.sample_rate), samplerate=self.sample_rate, channels=1, dtype='int16')
        sd.wait()  # wait until recording is finished

        # temporary storage to avoid writing to disk
        with tempfile.NamedTemporaryFile(delete=False) as tmpfile:
            wf = wave.open(tmpfile.name, 'wb')
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(self.sample_rate)
            wf.writeframes(audio_data)
            wf.close()

            # read the temp file and prepare the message
            with open(tmpfile.name, 'rb') as f:
                wav_data = f.read()

        msg = WavFile()
        msg.sample_rate = self.sample_rate
        msg.num_channels = 1
        msg.data = wav_data
        self.publisher_.publish(msg)
        self.get_logger().info('Audio published.')


def main(args=None):
    rclpy.init(args=args)
    audio_publisher = AudioPublisher()
    rclpy.spin(audio_publisher)
    audio_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
