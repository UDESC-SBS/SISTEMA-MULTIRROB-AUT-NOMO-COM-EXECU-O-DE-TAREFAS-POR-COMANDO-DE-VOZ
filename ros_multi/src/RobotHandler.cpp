
std::vector<std::string> extractJSONArguments(const std::string& arguments) {
    std::vector<std::string> extractedArgs;
    std::string args = arguments;

    // Remove surrounding curly braces if present
    if (args.front() == '{' && args.back() == '}') {
        args = args.substr(1, args.size() - 2);
    }

    // Regular expression to extract key-value pairs
    std::regex keyValuePattern(R"((?:"[^"]*":\s*"([^"]*)\"|\s*(\d+))\s*)");
    std::sregex_iterator keyValueIter(args.begin(), args.end(), keyValuePattern);
    std::sregex_iterator end;

    for (; keyValueIter != end; ++keyValueIter) {
        if ((*keyValueIter)[1].length() > 0) {
            extractedArgs.push_back((*keyValueIter)[1]);
        } else if ((*keyValueIter)[2].length() > 0) {
            extractedArgs.push_back((*keyValueIter)[2]);
        }
    }

    return extractedArgs;
}

std::vector<std::string> extractSimpleArguments(const std::string& arguments) {
    std::vector<std::string> extractedArgs;
    std::string args = arguments;

    // Regular expression to extract simple comma-separated values
    std::regex simplePattern(R"((?:"([^"]*)\"|\s*(\d+))\s*)");
    std::sregex_iterator simpleIter(args.begin(), args.end(), simplePattern);
    std::sregex_iterator end;

    for (; simpleIter != end; ++simpleIter) {
        if ((*simpleIter)[1].length() > 0) {
            extractedArgs.push_back((*simpleIter)[1]);
        } else if ((*simpleIter)[2].length() > 0) {
            extractedArgs.push_back((*simpleIter)[2]);
        }
    }

    return extractedArgs;
}


std::vector<std::vector<std::string>> extractFunctions(const std::string& input) {
    std::vector<std::vector<std::string>> result;

    // Regular expression to match functions and their arguments
    std::regex functionPattern(R"((\w+::\w+)\s*\((.*?)\)\s*)");
    std::smatch matches;

    std::string::const_iterator searchStart(input.cbegin());
    while (std::regex_search(searchStart, input.cend(), matches, functionPattern)) {
        std::string functionName = matches[1];  // Capture the function name
        std::string arguments = matches[2];     // Capture the arguments

        std::vector<std::string> functionInfo;
        functionInfo.push_back(functionName);   // First element is the function name

        // Determine if the arguments are in JSON-like format or simple format
        if (arguments.find('{') != std::string::npos) {
            // JSON-like format
            auto args = extractJSONArguments(arguments);
            functionInfo.insert(functionInfo.end(), args.begin(), args.end());
        } else {
            // Simple comma-separated format
            auto args = extractSimpleArguments(arguments);
            functionInfo.insert(functionInfo.end(), args.begin(), args.end());
        }

        result.push_back(functionInfo);
        searchStart = matches.suffix().first; // Move to the next part of the string
    }

    return result;
}


RobotHandler::RobotHandler(){
}

RobotHandler::~RobotHandler(){

}

bool RobotHandler::MoveNearest(PoseStamped *Goal){
	struct PathRobotPair {
		pRobot robot;
		Path plan;
	};

	PathRobotPair x;
	Path Plan;
	x.robot = SleepingRobots[0];
	x.robot->GetPath(&x.plan, Goal);

	for(pRobot R : SleepingRobots){
		R->GetPath(&Plan, Goal);
		if(Plan.poses.size() < x.plan.poses.size())
			x.robot = R, x.plan = Plan;
	}

	MoveBaseGoal G;
	G.target_pose = *Goal;
	G.target_pose.pose.orientation.w = 1.0;
	x.robot->MoveToGoal(&G);

	return true;
}

bool RobotHandler::MoveToGoal(pMoveTo Handle){
	MoveBaseGoal G;
    printf("%d : %s\n", Handle->index, Handle->goal);

    for(const auto& r : pMap->GetRooms()){
        if(strncmp(r->GetName(), Handle->goal, strlen(Handle->goal)) == 0){
            printf("Move robot to room %s\n", Handle->goal);
            G.target_pose = r->GetPos();
            printf("X%f Y%f\n", G.target_pose.pose.position.x, G.target_pose.pose.position.y);
            SleepingRobots[Handle->index]->MoveToGoal(&G);
            break;
        }
    }    
	return true;
}

bool RobotHandler::AddRobot(pRobot robot){
	printf("Adding Robot: %s\n", robot->GetFrameID());
	try{
		this->SleepingRobots.push_back(robot);
		return true;
	} catch (...) {
		return false;
	}
}

ros::NodeHandle* RobotHandler::GetNodeHandle(){
	return &hNode;
}

void RobotHandler::FetchFunction(pstEvent event){
    int FunctionCount =  atoi(((const char*)event->Data));
    std::vector<std::vector<std::string>> Response = extractFunctions((const char*)event->Data);
    

    for(int i = 0; i < FunctionCount; i++){
        pstEvent e = new stEvent(this, nullptr, 0, NOTHING);
        printf("%s\n", Response[i][0].c_str());
        if (strncmp(FunctionList[0], Response[i][0].c_str(), strlen(FunctionList[0])) == 0) {			
            e->Size = sizeof(MoveTo);
            e->Data = (void*) new MoveTo{atoi(Response[i][2].c_str()), Response[i][1].c_str()};
            e->Type = MOVE_ROBOT;
        } else if (strncmp(FunctionList[1], Response[i][0].c_str(), strlen(FunctionList[1])) == 0) {
            e->Type = SCAN_MAP;
        }
        ((EventHandler*)event->pEventHandler)->PushMessage(e);
    }


	return;
}