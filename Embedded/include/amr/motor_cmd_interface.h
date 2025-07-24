#pragma once
#include <mutex>

namespace amr {

enum class DriveState {
    STOP,
    FORWARD,
    REVERSE
};

class MotorCmdInterface {
public:
    void setCommand(DriveState state);
    DriveState getCommand();

private:
    std::mutex mutex_;
    DriveState currentState_ = DriveState::STOP;
};

}
