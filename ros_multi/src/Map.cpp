_Room::_Room(const char* name, Vector2 Pos) : Name(name) {
    printf("Room %s added at %f %f\n", name, Pos.x, Pos.y);
    Position.pose.position.x = Pos.x;
    Position.pose.position.y = Pos.y;
    Position.pose.orientation.w = 1.0;
    Position.header.frame_id = "map";
    Position.header.stamp = ros::Time(0);
}

void _Room::UpdatePos(Vector2 Pos){
    Position.pose.position.x = Pos.x;
    Position.pose.position.y = Pos.y;
    Position.pose.orientation.w = 1.0;
    Position.header.frame_id = "map";
    Position.header.stamp = ros::Time(0);
}

_MapHandler::_MapHandler(){}

_MapHandler::_MapHandler(bool CreateNew){}

void _MapHandler::CreateMap(){}

void _MapHandler::AddRoom(pRoom room){
    Rooms.push_back(room);
}

vector<pRoom> _MapHandler::GetRooms(){
    return Rooms;
}