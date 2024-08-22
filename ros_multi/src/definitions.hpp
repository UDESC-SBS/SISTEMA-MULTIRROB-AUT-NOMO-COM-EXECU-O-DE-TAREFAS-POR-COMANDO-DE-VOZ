#pragma once

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef std_msgs::Header Header;
typedef nav_msgs::Odometry::ConstPtr OdometryMsg; 
typedef geometry_msgs::PoseStamped PoseStamped;
typedef geometry_msgs::Pose Pose;
typedef move_base_msgs::MoveBaseGoal MoveBaseGoal;
typedef nav_msgs::OccupancyGrid OccupancyGrid;
typedef nav_msgs::Path Path;
typedef nav_msgs::GetPlan GetPlan;



class _Room;
class _Map;
class _Robot;
class RobotHandler;

template <typename T>
class Msg;
class EventHandler;



#include "utilities.hpp"