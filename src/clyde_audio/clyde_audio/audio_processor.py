import rclpy
from rclpy.node import Node
import speech_recognition as sr
import tempfile
import wave
from clyde_msgs.msg import WavFile  # Assuming this is your custom message for audio files


class AudioProcessor(Node):
    def __init__(self):
        super().__init__('audio_processor')
        self.subscription = self.create_subscription(
            WavFile,
            '/audio_files',
            self.process_audio,
            10)
        self.subscription  # prevent unused variable warning

    def process_audio(self, msg):
        self.get_logger().info('Received audio file.')

        # Save the received audio data to a temporary file
        with tempfile.NamedTemporaryFile(delete=False, suffix='.wav') as tmpfile:
            wf = wave.open(tmpfile.name, 'wb')
            wf.setnchannels(msg.num_channels)
            wf.setsampwidth(2)  # Assuming 16-bit samples
            wf.setframerate(msg.sample_rate)
            wf.writeframes(msg.data)
            wf.close()

            # Now you can use the file as needed for processing
            self.recognize_speech_from_wav(tmpfile.name)

    def recognize_speech_from_wav(self, wav_filename):
        recognizer = sr.Recognizer()
        with sr.AudioFile(wav_filename) as source:
            audio_data = recognizer.record(source)
            try:
                # Use Google's speech recognition
                text = recognizer.recognize_google(audio_data)
                self.get_logger().info(f'Recognized Text: {text}')
                # Here you can add more processing (e.g., wake word detection, commands, etc.)
            except sr.UnknownValueError:
                self.get_logger().error('Google Speech Recognition could not understand audio')
            except sr.RequestError as e:
                self.get_logger().error(f'Could not request results from Google Speech Recognition service; {e}')


def main(args=None):
    rclpy.init(args=args)
    audio_processor = AudioProcessor()
    rclpy.spin(audio_processor)
    audio_processor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
