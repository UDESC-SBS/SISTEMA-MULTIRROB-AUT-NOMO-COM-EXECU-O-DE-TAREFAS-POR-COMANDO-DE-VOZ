#! /usr/bin/env python3
import threading
#import whisper

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction,  MoveBaseGoal

import Core

global core
core = None

if __name__ == "__main__":
    core = Core.Core()

    try:
        rospy.init_node('movebase_core_client')
        Bots = [
                Core.Bot("/robot1/move_base", 1),
                Core.Bot("/robot2/move_base", 2)
                ]
        core.UpdateBotList(Bots)
        core.AddBot(Core.Bot("/robot3/move_base", 3))

        Places = [
                Core.Place("Banheiro", (-6.5, 3.0)),
                Core.Place("Garagem", (-4.9, 3.7)),
                Core.Place("Quarto A", (-6.4, -1.6)),
                Core.Place("Cozinha", (6.4, -4.4)),
                Core.Place("Cômodo A", (4.8, 1.6)),
                Core.Place("Cômodo B", (-3.0, 1.0)),
                Core.Place("Quarto B", (-2.8, 1.0))
                ]
        core.UpdateLocations(Places)
        core.AddLocation("Sala De Estar", (-1.5, 3.2))

        print("[*] Calling Main")
        t1 = threading.Thread(target=core.Main)
        t1.start()

        print("[*] Producer")
        Producer = threading.Thread(target=core.Producer, args=())
        Producer.start()
        
        print("[*] Consumer")
        Consumer = threading.Thread(target=core.Consumer, args=())
        Consumer.start()

        Producer.join()
        Consumer.join()

    except Exception as e:
        print(f"[!] Error in the initialization: {e}.")
