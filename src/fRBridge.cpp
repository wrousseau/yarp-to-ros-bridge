#include "fRBridge.h"

FRBridge::FRBridge(int* _fd, string _topicName, string _yarpPort) : 
fd(_fd), topicName(_topicName), yarpPort(_yarpPort)
{
}

bool FRBridge::run()
{
    std::thread tWrite(&FRBridge::writeData,this);
    std::thread tRead(&FRBridge::readRosData,this);
    tWrite.join();
    tRead.join();
    return true;
}