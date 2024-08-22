#pragma once 
#include <string>
#include <vector>
#include <atomic>
#include <thread>
#include <cstdio>
#include <stdexcept>
#include <unistd.h>
#include <regex>
#include <bits/stdc++.h>

#include <actionlib/client/simple_action_client.h>

#include <geometry_msgs/PoseStamped.h>

#include <move_base_msgs/MoveBaseAction.h>

#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <memory>

#include "definitions.hpp"

using namespace std;

const char* FunctionList[] = {
	"RobotHandler::MoveToGoal",
	"MapHandler::CreateMap"
};

#pragma pack(1)
enum EventTypes{
	SCAN_MAP,
	SAVE_MAP,
	ADD_ROBOT,
	GET_ROBOT_PATH,
	MOVE_ROBOT,
	CALLBACK_RESULT,
	NOTHING
};

typedef struct _V2 {
	float x = 0, y = 0;
} Vector2, *pVector2;

typedef struct _Event {
	_Event() {}
	_Event(void* ObjPointer, void* Data, size_t Size, EventTypes Type){
		this->ObjPointer = ObjPointer;
		this->Type = Type;
		this->Size = Size;
		this->Data = Data;
	}
	_Event(void* ObjPointer, void* pEventHandler, void* Data, size_t Size, EventTypes Type){
		this->ObjPointer = ObjPointer;
		this->pEventHandler = pEventHandler;
		this->Type = Type;
		this->Size = Size;
		this->Data = Data;
	}
	void AllocData(void* Src){
		Data = malloc(Size);
		memcpy(Data, Src, Size);	
		return;
	}
	void* ObjPointer;
	void* Data;
	void* pEventHandler;
	size_t Size;
	EventTypes Type;
} stEvent, *pstEvent;

typedef class _Room{
	private:
		const char* Name;
		PoseStamped Position;

	public:
		_Room() {}
		_Room(const char* name) : Name(name) {}
		_Room(const char* name, Vector2 Pos);

		const char* GetName(){return Name;}
		void UpdateName(const char* name) {Name = name;}

		PoseStamped GetPos(){return Position;}
		void UpdatePos(Vector2 Pos);

} Room, *pRoom;

typedef class _MapHandler{
	public:
		_MapHandler();
		_MapHandler(bool CreateNew);
		void CreateMap();

		void AddRoom(pRoom room);
		vector<pRoom> GetRooms();

	private:
		OccupancyGrid GlobalMap;
		vector<pRoom> Rooms;


} MapHandler, *pMapHandler;

typedef class _Robot{
	public:
		_Robot(std::string Name, ros::NodeHandle* hNode);
		~_Robot();
		bool MoveToGoal(MoveBaseGoal *goal);
		bool GetPosition(PoseStamped* Position);
		bool GetPath(Path *plan, PoseStamped *goal);
		const char* GetFrameID();

	protected:
		void PoseCallback(const OdometryMsg& msg);
		
	private:
		ros::Subscriber PoseSubs;
		ros::ServiceClient MakePlanClient;
		string frame_id;
		PoseStamped Pose;
		MoveBaseClient* pMoveBase;

} Robot, *pRobot;

// Scan map
// Separate map into rooms
class RobotHandler{
	public:
		pMapHandler pMap;
		typedef struct _MoveTo{
			int index = 0;
			const char* goal;
		} MoveTo, *pMoveTo;

		RobotHandler();
		~RobotHandler();
		bool MoveNearest(PoseStamped *Goal);
		bool MoveToGoal(pMoveTo Handle);
		bool AddRobot(pRobot robot);
		ros::NodeHandle* GetNodeHandle();
		void FetchFunction(pstEvent event);

	private:
		ros::NodeHandle hNode;
		vector<pRobot> SleepingRobots;
		vector<pRobot> ActiveRobots;
};


class EventHandler {
	public:
		EventHandler();
		~EventHandler();
		bool Run();
		bool Stop();
		bool IsRunning();
		void PushMessage(pstEvent event);

	private:
		void RunLoop();
		void FinishExecution();

		atomic_bool ThreadState;
		atomic_bool IsFinished;
		thread LoopThread;
		queue <pstEvent> EventQueue;
};
#pragma pack(1)

#include "Map.cpp"
#include "Robot.cpp"
#include "RobotHandler.cpp"
#include "EventHandler.cpp"