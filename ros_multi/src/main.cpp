#include <cstdio>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <regex>
#include <cstdint>

#include "utilities.hpp"

EventHandler* Loop = new EventHandler();
RobotHandler* hRobots = nullptr;

void AIWrapperCallback(const std_msgs::String::ConstPtr& msg){
	//printf("MessageReceived!\n");
	pstEvent e = new stEvent(hRobots, nullptr, msg->data.size(), CALLBACK_RESULT);
	e->AllocData((void*)msg->data.c_str());
	Loop->PushMessage(e);
}


int main(int argc, char** argv){
	ros::init(argc, argv, "multi_robot_navigation_node");
	
	//PoseStamped x((0, ros::Time(0), "map"), Pose{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}});

	hRobots = new RobotHandler;
	hRobots->pMap = new MapHandler();
	hRobots->pMap->AddRoom(new Room("cozinha", {-4, 4}));
	hRobots->pMap->AddRoom(new Room("banheiro", {1, 3.7}));
	hRobots->pMap->AddRoom(new Room("sala de estar", {6.7, 4}));
	hRobots->pMap->AddRoom(new Room("sala de jogos", {-6.6, 3.2}));
	hRobots->pMap->AddRoom(new Room("quarto", {-6, -3}));
	hRobots->pMap->AddRoom(new Room("garagem", {6.7, -4.5}));

	Loop->PushMessage(new stEvent(hRobots, new Robot("robot1", hRobots->GetNodeHandle()), sizeof(void*), ADD_ROBOT));
	Loop->PushMessage(new stEvent(hRobots, new Robot("robot2", hRobots->GetNodeHandle()), sizeof(void*), ADD_ROBOT));
	Loop->PushMessage(new stEvent(hRobots, new Robot("robot3", hRobots->GetNodeHandle()), sizeof(void*), ADD_ROBOT));

	Loop->Run();

	ros::Publisher pub = hRobots->GetNodeHandle()->advertise<std_msgs::String>("RobotHandler", 1000);
	ros::Subscriber sub = hRobots->GetNodeHandle()->subscribe("LLama", 1000, AIWrapperCallback);

	while(true){
		/*
		printf("Type a position to move: ");
		PoseStamped x;
		scanf("%lf %lf", &x.pose.position.x, &x.pose.position.y);
		if (x.pose.position.x == -1) break;
		x.pose.orientation.w = 1.0;
		for(int i = 0; i < 1; i++){
			hRobots.MoveNearest(&x);
		}*/

		std_msgs::String msg;
		std::string ss;
		std::getline(std::cin, ss);
		auto n = ss.find( '\x18' );
		if (n != ss.npos) ss = ss.substr( 0, n );
		if (n != ss.npos) break;
		msg.data = ss.c_str();
		pub.publish(msg);
		//Loop->PushMessage(new stEvent(&hRobots, &x, sizeof(PoseStamped), MOVE_ROBOT));
	}



	Loop->Stop();
	
	delete Loop;

	return 0;
}
