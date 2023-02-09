#pragma once

#include "consts.hpp"
#include "state.hpp"
#include "types.hpp"

#include <optional>
#include <unordered_map>

#include <boost/unordered_map.hpp>

namespace hyped::state_machine {

class StateMachine {
 public:
  StateMachine() { current_state = State::kIdle; };

  void handleMessage(const Message message);

  std::optional<Message> handleData();

  Message stringToMessage(const std::string &message_name);

  std::string messageToString(const Message &message);

 private:
  const std::unordered_map<std::string, Message> string_to_message_
    = {{"mForward", Message::mForward}, {"mFailure", Message::mFailure}};

  const std::unordered_map<Message, std::string> message_to_string_
    = {{Message::mForward, "mForward"}, {Message::mFailure, "mFailure"}};

  const boost::unordered_map<SourceAndMessage, State> transition_to_state_
    = {{{State::kIdle, Message::mForward}, State::kCalibrating},
       {{State::kCalibrating, Message::mForward}, State::kReady},
       {{State::kReady, Message::mForward}, State::kAccelerating},
       {{State::kAccelerating, Message::mForward}, State::kCruising},
       {{State::kCruising, Message::mForward}, State::kNominalBraking},
       {{State::kNominalBraking, Message::mForward}, State::kStopped},
       {{State::kStopped, Message::mForward}, State::kOff},
       {{State::kAccelerating, Message::mFailureBrake}, State::kFailureBraking},
       {{State::kAccelerating, Message::mMotorBrake}, State::kMotorBraking},
       {{State::kAccelerating, Message::mFrictionBrake}, State::kFrictionBraking},
       {{State::kCruising, Message::mFailureBrake}, State::kFailureBraking},
       {{State::kCruising, Message::mFrictionBrake}, State::kFrictionBraking},
       {{State::kCruising, Message::mMotorBrake}, State::kMotorBraking},
       {{State::kNominalBraking, Message::mMotorBrake}, State::kMotorBraking},
       {{State::kNominalBraking, Message::mFrictionBrake}, State::kFrictionBraking},
       {{State::kNominalBraking, Message::mFailureBrake}, State::kFailureBraking},
       {{State::kMotorBraking, Message::mFailureBrake}, State::kFailureBraking},
       {{State::kMotorBraking, Message::mFrictionBrake}, State::kFrictionBraking},
       {{State::kMotorBraking, Message::mFailure}, State::kFailureStopped},
       {{State::kFrictionBraking, Message::mFailureBrake}, State::kFailureBraking},
       {{State::kFrictionBraking, Message::mFailure}, State::kFailureStopped},
       {{State::kFailureBraking, Message::mFailure}, State::kFailureStopped},
       {{State::kFailureStopped, Message::mFailure}, State::kOff}};

  State current_state;
};
}  // namespace hyped::state_machine