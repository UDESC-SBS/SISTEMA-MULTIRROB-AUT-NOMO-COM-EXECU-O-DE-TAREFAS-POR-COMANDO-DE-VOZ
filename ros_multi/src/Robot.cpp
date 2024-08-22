_Robot::_Robot(std::string Name, ros::NodeHandle* hNode){
	frame_id = Name;
	Pose.header.frame_id = Name.c_str();
	PoseSubs = hNode->subscribe("/" + Name + "/odom", 10, &Robot::PoseCallback, this);
	MakePlanClient = hNode->serviceClient<nav_msgs::GetPlan>("/" + Name + "/move_base_node_" + Name + "/make_plan");
	pMoveBase = new MoveBaseClient(Name + "/move_base", true);
}

_Robot::~_Robot(){

}

bool _Robot::MoveToGoal(MoveBaseGoal *goal){
	pMoveBase->sendGoal(*goal);
	return true;
}

bool _Robot::GetPosition(PoseStamped* Position){
	*Position = Pose;
	return true;
}

bool _Robot::GetPath(Path *plan, PoseStamped *Goal){
	GetPlan srv;

	this->GetPosition(&srv.request.start);
	srv.request.start.header.frame_id = "map";
	Goal->header.frame_id = "map";
	srv.request.goal = *Goal;
	srv.request.tolerance = 0.5;
	if(MakePlanClient.call(srv)){
		*plan = srv.response.plan;
		return true;
	}

	return false;
}

const char* _Robot::GetFrameID(){
	return frame_id.c_str();
};

void _Robot::PoseCallback(const OdometryMsg& msg){
	Pose.pose = msg->pose.pose;
}