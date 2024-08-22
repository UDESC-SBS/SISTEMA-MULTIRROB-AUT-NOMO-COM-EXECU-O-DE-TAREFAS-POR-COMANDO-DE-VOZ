#!/usr/bin/env python3

from faster_whisper import WhisperModel
import threading
import pyaudio as pa
import numpy as np
import pygame
import rospy
from std_msgs.msg import String

"""
def talker():
    pub = rospy.Publisher('LLamaTalker', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if StringList:
            #rospy.loginfo(hello_str)
            pub.publish(StringList[0])
            del StringList[0]
        rate.sleep()
"""

class Core:
    __RecAudio = False
    __running = False
    __Stream = None
    __Mic = None
    __Display  = None
    __AudioData = []
    __Whisper = None
    __AudioQueue = []

    def __init__(self):
        print("[*] Loading WhisperModel...")
        self.__Responses = []
        self.__Whisper = WhisperModel("medium",
                                      device="cpu",
                                      compute_type="int8",
                                      num_workers=10,
                                      cpu_threads=8)
        print("[*] WhisperModel Loaded")
        
        self.__Mic = pa.PyAudio()
        
        self.__Stream = self.__Mic.open(format=pa.paInt16, 
                                        channels=1, 
                                        rate=16000,
                                        input=True,
                                        frames_per_buffer=1024)
        self.__Display = pygame.display.set_mode((400, 320))
        self.__TalkerThread = threading.Thread(target=self.Talker)
        self.__ConsumerThread = threading.Thread(target=self.Consumer)
        self.__ProducerThread = threading.Thread(target=self.Producer)
        pass
        
    def Main(self):
        rospy.init_node('AIWrapper', anonymous=True)
        self.__running = True
        self.__TalkerThread.start()
        self.__ConsumerThread.start()
        self.__ProducerThread.start()
        while self.__running:
            for Event in pygame.event.get():
                if Event.type == pygame.QUIT:
                    self.__running = False
                    self.__ProducerThread.join()
                    self.__ConsumerThread.join()
                    self.__Stream.stop_stream()
                    self.__Stream.close()
                    self.__Mic.terminate()


                    pygame.quit()
                    return
                    #sys.exit()

                if Event.type == pygame.KEYDOWN:
                    if Event.key == pygame.K_ESCAPE:
                        self.__running = False
                        self.__ProducerThread.join()
                        self.__ConsumerThread.join()
                        self.__Stream.stop_stream()
                        self.__Stream.close()
                        self.__Mic.terminate()
                        pygame.quit()
                        return
                        #sys.exit()

                    if Event.key == pygame.K_SPACE:
                        self.__RecAudio = True
                        pass
                    pass

                if Event.type == pygame.KEYUP:
                    if Event.key == pygame.K_SPACE:
                        self.__RecAudio = False
                        pass
                    pass
                pass
            pass

    def Transcribe(self, audio_data) -> str:
        audio_data = b''.join(audio_data)
        audio_data_array: np.ndarray = np.frombuffer(audio_data, np.int16).astype(np.float32) / 255.0
        segments, _ = self.__Whisper.transcribe(audio_data_array,
                                    language="pt",
                                    beam_size=5,
                                    vad_filter=True,
                                    vad_parameters=dict(min_silence_duration_ms=1000))

        segments = [s.text for s in segments]
        transcription = " ".join(segments)
        transcription = transcription.strip()
        print("[*] TranscribeBlocking Executed!")
        return transcription

    def Talker(self):
        pub = rospy.Publisher('Whisper', String, queue_size=10)
        rate = rospy.Rate(10)
        while self.__running:
            if self.__Responses:
                pub.publish(self.__Responses[0])
                del self.__Responses[0]
        rate .sleep()

    def Producer(self):
        while self.__running:
            chunk = self.__Stream.read(16000)
            if self.__RecAudio == False and self.__AudioData:
                self.__AudioQueue.append(self.__AudioData)
                self.__AudioData = []

            if self.__RecAudio == True:
                self.__AudioData.append(chunk)
                #self.__AudioQueue.append(self.__AudioData)

    def Consumer(self):
        while self.__running:
            if self.__AudioQueue:
                print("if self.__AudioQueue")
                TranscribeResult = self.Transcribe(self.__AudioQueue[0])
                self.__Responses.append(TranscribeResult)
                print(f"TranscribeResult: {TranscribeResult}")

                del self.__AudioQueue[0]

c = Core()
c.Main()
