

#include <vector>
#include <atomic>
#include <thread>
#include <cstdio>
#include <stdexcept>
#include <unistd.h>

#include <bits/stdc++.h>

using std::vector;
using std::atomic;
using std::thread;

vector<int> FDList;

EventHandler::EventHandler() : ThreadState(false), IsFinished(false) {
  printf("Constructor!\n");
}

EventHandler::~EventHandler() {
  printf("Destructor!\n");
  FinishExecution();
}

bool EventHandler::Run() {
  try {
    LoopThread = thread(&EventHandler::RunLoop, this);
  } catch (...) {
    return false;
  }
  return true;
}

bool EventHandler::Stop() {
  FinishExecution();
  return true;
}

bool EventHandler::IsRunning() {
  return ThreadState.load();
}

void EventHandler::PushMessage(pstEvent event){
  EventQueue.push(event);
}

void EventHandler::RunLoop() {
    ThreadState.store(true);
    while (!IsFinished.load()){
      ros::spinOnce();
        try {
            pstEvent event = nullptr;
            if(!EventQueue.empty()){
                event = EventQueue.front();
                EventQueue.pop();
                switch (event->Type) {
                    case SCAN_MAP:
                        printf("ScanMap!\n");
                        break;

                    case SAVE_MAP:
                        break;

                    case ADD_ROBOT:
                        ((RobotHandler*)(event->ObjPointer))->AddRobot((pRobot)event->Data);
                        break;
                    
                    case GET_ROBOT_PATH:
                        break;

                    case MOVE_ROBOT:
                        ((RobotHandler*)(event->ObjPointer))->MoveToGoal((RobotHandler::pMoveTo)event->Data);
                        printf("Robot Moved!\n");
                        break;

                    case CALLBACK_RESULT:
                        event->pEventHandler = (void*)this;
                        ((RobotHandler*)(event->ObjPointer))->FetchFunction(event);
                        break;

                    default:
                        printf("Unknow Event Type!\n");
                    break;
                }
                free(event);
            } else {
                // sleep for some time
            }
        } catch (std::runtime_error& e) {
            printf("Some Runtime Error!\n");
        } catch (...) {}
    }
    ThreadState.store(false);
}

void EventHandler::FinishExecution() {
  IsFinished.store(true);
  if (LoopThread.joinable()) {
    LoopThread.join();
  }
}

