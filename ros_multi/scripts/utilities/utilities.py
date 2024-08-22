#! /usr/bin/env python3

import rospy
import actionlib

from threading import Thread
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan, GetPlanRequest, GetPlanResponse
from nav_msgs.msg import Path
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class CoreMessages:
    MOVE_ROBOT = 1
    MOVE_NEAREST_ROBOT = 2
    ADD_ROBOT = 3
    REMOVE_ROBOT = 4

class Robot:
    __Position : PoseStamped = None
    __ServiceProxy : rospy.ServiceProxy = None
    ActionBaseNode : actionlib.SimpleActionClient = None

    def __init__(self, Namespace):
        self.__ServiceProxy = rospy.ServiceProxy(f'{Namespace}/move_base/make_plan', GetPlan)
        self.ActionBaseNode = actionlib.SimpleActionClient(f"{Namesapce}/move_base", MoveBaseAction)

    def Proxy(self, Request):
        return self.__ServiceProxy(Request)

    def GetPosition(self):
        return self.__Position


class RobotHandler:
    __Robots : [Robot] = []

    def __init__(self):
        self.__Robots = []
        pass
    
    def AddRobot(self, robot : Robot):
        self.__Robots.append(robot)

    def RemoveRobot(self, RobotIndex : int):
        self.__Robots.pop(RobotIndex)

    def MoveRobot(self, RobotIndex : int, Goal : MoveBaseGoal):
        try:
            self.__Robots[RobotIndex].ActionBaseNode.send_goal(Goal)
            return True
        except:
            return False

    def MakePlan(self, RobotIndex : int, Goal : PoseStamped) -> GetPlanResponse:
        if self.__Robots[RobotIndex].ActionBaseNode.get_state() is actionlib.GoalStatus.ACTIVE:
            rospy.logerr("Robot of index %d is currently active" % RobotIndex)
            return None

        Request = GetPlanRequest()
        Request.start = self.__Robots[RobotIndex].GetPosition()
        Request.goal = Goal
        Request.tolerance = 0.0

        try:
            Response = self.__Robots[RobotIndex].Proxy(Request)
            return Response.plan
        except:
            rospy.logerr("Service Call Failed: %s" % e)
            return None

class Core:
    def __init__(self, Robots : [Robot]):
        self.RosNode = rospy.init_node('multi_robot_client')
        self.RobotHandler = RobotHandler()
        for i in Robots:
            self.RobotHandler.AddRobot(i)

    pass

