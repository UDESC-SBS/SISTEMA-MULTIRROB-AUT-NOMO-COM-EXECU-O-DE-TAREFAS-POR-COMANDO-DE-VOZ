#! /usr/bin/env python3

import sys
import os
import threading
#import json
#import wave
import pyaudio as pa
import queue
import re
import numpy as np
import pygame

#from ctransformers import AutoModelForCausalLM
from faster_whisper import WhisperModel

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


#########################################################################

"""
class Place:
    __Name : str
    __Position : (float, float)

    def __init__(self, Name : str, Position : (float, float)):
        self.__Name = Name
        self.__Position = Position
        return

    def UpdateName(self, Name : str):
        self.__Name = Name
        return
    
    def UpdatePos(self, Position : (float, float)):
        self.__Position = Position
        return
    
    def GetName(self) -> str:
        return self.__Name

    def GetPosition(self) -> (float, float):
        return self.__Position

    def __del__(self):
        return

#########################################################################

class Bot:
    __ID = None     # Robot ID
    __Node = None   # Ros node for action server
    
    def __init__(self, NodeName : str, ID : int):
        self.__Node = actionlib.SimpleActionClient(NodeName, MoveBaseAction)
        self.__ID = ID
        pass

    def MoveBaseGoal(self, Pos : (float, float)):
        self.__Node.wait_for_server()
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = Pos[0]
        goal.target_pose.pose.position.y = Pos[1]
        goal.target_pose.pose.orientation.w = 1.0

        self.__Node.send_goal(goal)
        pass

    def GetId(self):
        return self.__ID

    def GetNode(self):
        return self.__Node

    pass

########################################################################

"""
    
class Core:
    __RecAudio = False
    __Stream = None
    __Mic = None
    __Display  = None

    __Places : [Place]
    __AudioData = []
    __Bots = []

    __Whisper = None

    __AudioQueue = []

    def __init__(self):
        print("[*] Loading WhisperModel...")
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
        print("[*] DisplayCreated")
        pass

    # EventManager thread, it will be running in a loop, it is responsible
    # for treating the Keystrokes and the pygame window.
    def Main(self):
        print("[!] Main")
        while True:
            for Event in pygame.event.get():
                if Event.type == pygame.QUIT:
                    self.__Stream.stop_stream()
                    self.__Stream.close()
                    self.__Mic.terminate()

                    pygame.quit()
                    sys.exit()
                    pass

                if Event.type == pygame.KEYDOWN:
                    if Event.key == pygame.K_ESCAPE:

                        self.__Stream.stop_stream()
                        self.__Stream.close()
                        self.__Mic.terminate()
                        pygame.quit()
                        sys.exit()
                        pass

                    if Event.key == pygame.K_SPACE:
                        print("[!] Recording Audio...")
                        self.__RecAudio = True
                        pass
                    pass

                if Event.type == pygame.KEYUP:
                    if Event.key == pygame.K_SPACE:
                        print("[!] Doing Nothing...")
                        self.__RecAudio = False
                        pass
                    pass
                pass
            pass

    def UpdateBotList(self, Bot : []):
        self.__Bots = Bot
        return True

    def GetBots(self):
        if not self.__Bots:
            print("[!] Bot list empty!")
            return
        else:
            return self.__Bots

    def AddBot(self, Bot):
        self.__Bots.append(Bot)
        pass

    def UpdateLocations(self, L : [Place]):
        self.__Places = L
        pass

    def AddLocation(self, Name, Pos : (float, float)):
        self.__Places.append(Place(Name, Pos))
        return

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

    def Producer(self):
        while True:
            chunk = self.__Stream.read(16000)
            if self.__RecAudio == False and self.__AudioData:
                self.__AudioQueue.append(self.__AudioData)
                self.__AudioData = []

            if self.__RecAudio == True:
                self.__AudioData.append(chunk)
                #self.__AudioQueue.append(self.__AudioData)

    def Consumer(self):
        while True:
            if self.__AudioQueue:
                print("if self.__AudioQueue")
                TranscribeResult = self.Transcribe(self.__AudioQueue[0])
                print(f"TranscribeResult: {TranscribeResult}")

                Numbers = re.findall("\d+", TranscribeResult)
                Positions : [(float, float)]= []
                for i in self.__Places:
                    if i.GetName().upper() in TranscribeResult.upper():
                        Positions.append(i.GetPosition())
                        print(f"{i.GetPosition()}:{i.GetName().upper()}")

                print(f"Numbers: {Numbers} | Positions: {Positions}")

                for i in range(len(Positions)): 
                    if i < len(self.__Bots):
                        for j in self.__Bots:
                            if j.GetId() == int(Numbers[i]):
                                print(f"{i}:BotID '{Numbers[i]}' goes to '{Positions[i]}'")
                                j.MoveBaseGoal(Positions[i])

                del self.__AudioQueue[0]
