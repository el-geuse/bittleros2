import os
import struct
import wave
from datetime import datetime
import speech_recognition as sr
import pvporcupine
from pvrecorder import PvRecorder
sr.__version__
'3.8.1'


class Audio_Detector:
    def __init__(self, access_key, keyword_paths, audio_device_index=-1, output_path=None):
        self.access_key = access_key
        self.keyword_paths = keyword_paths
        self.audio_device_index = audio_device_index
        self.output_path = output_path
        self.RATE = 44100
        self.CHUNK = 1024
        self.porcupine = None
        self.PvRecorder = None
        self.wav_file = None

    def porcupine_init(self, library_path=None, model_path=None, Sensitivities=[0.7]):
        try:
            self.porcupine = pvporcupine.create(
                                                access_key=self.access_key,
                                                library_path=library_path,
                                                model_path=model_path,
                                                keyword_paths=self.keyword_paths,
                                                sensitivities=Sensitivities)
        except pvporcupine.PorcupineInvalidArgumentError as e:
            print("One or more arguments provided to Porcupine is invalid")
            print(e)
            raise e
        except pvporcupine.PorcupineActivationError as e:
            print("AccessKey activation error")
            raise e
        except pvporcupine.PorcupineActivationLimitError as e:
            print("AccessKey '%s' has reached it's temporary device limit")
            raise e
        except pvporcupine.PorcupineActivationRefusedError as e:
            print("AccessKey '%s' refused")
            raise e
        except pvporcupine.PorcupineActivationThrottledError as e:
            print("AccessKey '%s' has been throttled")
            raise e
        except pvporcupine.PorcupineError as e:
            print("Failed to initialize Porcupine")
            raise e

    def PvRecorder_init(self):
        self.recorder = PvRecorder(
            frame_length=self.porcupine.frame_length,  # 512 default
            device_index=self.audio_device_index)
        self.recorder.start()

    def wavefile(self):
        if self.output_path is not None:
            self.wav_file = wave.open(self.output_path, "w")
            self.wav_file.setnchannels(1)
            self.wav_file.setsampwidth(2)
            self.wav_file.setframerate(20000)

    def wakeword(self):
        keywords = [os.path.basename(x).replace('.ppn', '').split('_')[0] for x in self.keyword_paths]

        print('Listening ... (press Ctrl+C to exit)')

        try:
            self.porcupine_init()
            self.PvRecorder_init()
            self.wavefile()

            while True:
                pcm = self.recorder.read()
                result = self.porcupine.process(pcm)

                if self.wav_file is not None:
                    self.wav_file.writeframes(struct.pack("h" * len(pcm), *pcm))

                if result >= 0:
                    print('[%s] Detected %s' % (str(datetime.now()), keywords[result]))
                    self.recorder.stop()
                    self.speech_to_text()

        except KeyboardInterrupt:
            print('Stopping ...')

    def speech_to_text(self):

       a = []
       seconds = 2

       self.recorder.start()
       print("start recording...")

       for i in range(0, int(self.RATE/self.CHUNK*seconds)):
           frame = self.recorder.read()
           a.extend(frame)

       self.recorder.stop()
       with wave.open("output.wav", 'w') as f:
           f.setparams((1, 2, 16000, 512, "NONE", "NONE"))
           f.writeframes(struct.pack("h" * len(a), *a))
           self.recorder.delete()

       print("recording stopped")

       r = sr.Recognizer()

       output_file = sr.AudioFile("output.wav")
       with output_file as source:
           r.adjust_for_ambient_noise(source, duration=0.5)
           audio = r.record(source)

       type(audio)

       audio = r.recognize_google(audio)

       try:
          if audio == "follow":
              print("now following")
              exit()
          else:
              print("unrecognised prompt")
              self.wakeword()
       except sr.UnknownValueError:
           print("Audio not recognised")
           self.wakeword()


if __name__ == '__main__':

    access_key = "C2xFc9n2QGICQXRJuWvfCZUMZW+uru9hd18xtfD1PWCAXc5LbqfTdQ=="
    keyword_paths = ["/workspaces/clyderos2/src/clyde_audio/resource/Hello-Clyde_en_linux_v3_0_0.ppn"]

    audio = Audio_Detector(access_key=access_key, keyword_paths=keyword_paths)
    audio.wakeword()
