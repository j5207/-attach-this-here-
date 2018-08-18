#!/usr/bin/env python

from __future__ import division
import re
import sys
import os
os.environ["GOOGLE_APPLICATION_CREDENTIALS"]="/home/lize/Downloads/Speech to Text-key.json"
from google.cloud import speech
from google.cloud.speech import enums
from google.cloud.speech import types
import pyaudio
from six.moves import queue
import datetime
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String

# [END import_libraries]

# Audio recording parameters
RATE = 16000
CHUNK = int(RATE / 10)  # 100ms


class MicrophoneStream(object):
    """Opens a recording stream as a generator yielding the audio chunks."""
    def __init__(self, rate, chunk):
        self._rate = rate
        self._chunk = chunk

        # Create a thread-safe buffer of audio data
        self._buff = queue.Queue()
        self.closed = True

    def __enter__(self):
        self._audio_interface = pyaudio.PyAudio()
        self._audio_stream = self._audio_interface.open(
            format=pyaudio.paInt16,
            # The API currently only supports 1-channel (mono) audio
            # https://goo.gl/z757pE
            channels=1, rate=self._rate,
            input=True, frames_per_buffer=self._chunk,
            # Run the audio stream asynchronously to fill the buffer object.
            # This is necessary so that the input device's buffer doesn't
            # overflow while the calling thread makes network requests, etc.
            stream_callback=self._fill_buffer,
        )

        self.closed = False

        return self

    def __exit__(self, type, value, traceback):
        self._audio_stream.stop_stream()
        self._audio_stream.close()
        self.closed = True
        # Signal the generator to terminate so that the client's
        # streaming_recognize method will not block the process termination.
        self._buff.put(None)
        self._audio_interface.terminate()

    def _fill_buffer(self, in_data, frame_count, time_info, status_flags):
        """Continuously collect data from the audio stream, into the buffer."""
        self._buff.put(in_data)
        return None, pyaudio.paContinue

    def generator(self):
        while not self.closed:
            # Use a blocking get() to ensure there's at least one chunk of
            # data, and stop iteration if the chunk is None, indicating the
            # end of the audio stream.
            chunk = self._buff.get()
            if chunk is None:
                return
            data = [chunk]

            # Now consume whatever other data's still buffered.
            while True:
                try:
                    chunk = self._buff.get(block=False)
                    if chunk is None:
                        return
                    data.append(chunk)
                except queue.Empty:
                    break

            yield b''.join(data)
# [END audio_stream]


def listen_print_loop(responses):
    num_chars_printed = 0
    pub_select = rospy.Publisher('/voice_command', Int32, queue_size=1)
    pub_color = rospy.Publisher('/item_color', String, queue_size=20)
    start_time = datetime.datetime.now()

    for response in responses:
        current_time = datetime.datetime.now()
        if (current_time - start_time).seconds > 55:
            return True
        if not response.results:
            continue
        result = response.results[0]
        if not result.alternatives:
            continue
        transcript = result.alternatives[0].transcript
        overwrite_chars = ' ' * (num_chars_printed - len(transcript))

        if not result.is_final:
            sys.stdout.write(transcript + overwrite_chars + '\r')
            sys.stdout.flush()

            num_chars_printed = len(transcript)

        else:
            print(transcript + overwrite_chars)
            if re.search(r'\b(exit|quit)\b', transcript, re.I):
                print('Exiting..')
                pub_select.publish(-1)
                return False
            elif re.search(r'\b(attach|put|move|make|moves|who|attached|attack|attacked|moved|get)\b', transcript, re.I):
                print(1)
                pub_select.publish(1)
                if re.search(r'\b(blue)\b', transcript, re.I):
                    print("blue")
                    pub_color.publish("blue")
                elif re.search(r'\b(yellow)\b', transcript, re.I):
                    print("yellow")
                    pub_color.publish("yellow")
                elif re.search(r'\b(green)\b', transcript, re.I):
                    print("green")
                    pub_color.publish("green")

            elif re.search(r'\b(here|there|shear|Kia|cheer|this|that)\b', transcript, re.I):
                print(2)
                pub_select.publish(2)
                return True

            num_chars_printed = 0


def main():
    rospy.init_node('Speech_node')
    language_code = 'en-US'
    if_restart = True
    client = speech.SpeechClient()
    config = types.RecognitionConfig(
        encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
        sample_rate_hertz=RATE,
        language_code=language_code)
    streaming_config = types.StreamingRecognitionConfig(
        config=config,
        interim_results=True)
    while if_restart:
        with MicrophoneStream(RATE, CHUNK) as stream:
            audio_generator = stream.generator()
            requests = (types.StreamingRecognizeRequest(audio_content=content)
                        for content in audio_generator)

            responses = client.streaming_recognize(streaming_config, requests)
            if_restart = listen_print_loop(responses)




if __name__ == '__main__':
    main()
