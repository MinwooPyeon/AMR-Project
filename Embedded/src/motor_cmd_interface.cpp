#include "amr/motor_cmd_interface.h"

namespace amr {

void MotorCmdInterface::setCommand(DriveState state) {
    std::lock_guard<std::mutex> lock(mutex_);
    currentState_ = state;
}

DriveState MotorCmdInterface::getCommand() {
    std::lock_guard<std::mutex> lock(mutex_);
    return currentState_;
}

}  
