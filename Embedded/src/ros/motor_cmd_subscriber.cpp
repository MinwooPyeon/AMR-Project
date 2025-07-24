#include "amr/ros/motor_cmd_subscriber.h"

namespace amr {

MotorCmdSubscriber::MotorCmdSubscriber(std::shared_ptr<MotorCmdInterface> cmdIf)
    : Node("motor_cmd_subscriber"), cmdIf_(cmdIf) {
    sub_ = create_subscription<std_msgs::msg::Int32>(
        "motor_cmd", 10,
        std::bind(&MotorCmdSubscriber::topicCb, this, std::placeholders::_1)
    );
}

void MotorCmdSubscriber::topicCb(const std_msgs::msg::Int32::SharedPtr msg) {
    if (msg->data == 1) cmdIf_->setCommand(DriveState::FORWARD);
    else if (msg->data == -1) cmdIf_->setCommand(DriveState::REVERSE);
    else cmdIf_->setCommand(DriveState::STOP);
}

}
