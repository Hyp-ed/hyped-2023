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
  const std::unordered_map<std::string, Message> StringMap
    = {{"mForward", Message::mForward}, {"mFailure", Message::mFailure}};

  const std::unordered_map<Message, std::string> MessageMap
    = {{Message::mForward, "mForward"}, {Message::mFailure, "mFailure"}};

  const boost::unordered_map<SourceAndMessage, State> TransitionMap
    = {{{State::kIdle, Message::mForward}, State::kCalibrating},
       {{State::kCalibrating, Message::mForward}, State::kReady},
       {{State::kReady, Message::mForward}, State::kAccelerating},
       {{State::kAccelerating, Message::mForward}, State::kCruising},
       {{State::kCruising, Message::mForward}, State::kNominalBraking},
       {{State::kNominalBraking, Message::mForward}, State::kStopped},
       {{State::kStopped, Message::mForward}, State::kOff},
       {{State::kAccelerating, Message::mFailure}, State::kFailureBraking},
       {{State::kAccelerating, Message::mFailure}, State::kNominalBraking},
       {{State::kAccelerating, Message::mFailure}, State::kMotorBraking},
       {{State::kAccelerating, Message::mFailure}, State::kFrictionBraking},
       {{State::kCruising, Message::mFailure}, State::kFailureBraking},
       {{State::kCruising, Message::mFailure}, State::kFrictionBraking},
       {{State::kCruising, Message::mFailure}, State::kMotorBraking},
       {{State::kNominalBraking, Message::mFailure}, State::kMotorBraking},
       {{State::kNominalBraking, Message::mFailure}, State::kFrictionBraking},
       {{State::kNominalBraking, Message::mFailure}, State::kFailureBraking},
       {{State::kMotorBraking, Message::mFailure}, State::kFailureBraking},
       {{State::kMotorBraking, Message::mFailure}, State::kFrictionBraking},
       {{State::kMotorBraking, Message::mFailure}, State::kFailureStopped},
       {{State::kFrictionBraking, Message::mFailure}, State::kFailureBraking},
       {{State::kFrictionBraking, Message::mFailure}, State::kFailureStopped},
       {{State::kFailureBraking, Message::mFailure}, State::kFailureStopped},
       {{State::kFailureStopped, Message::mFailure}, State::kOff}};

  State current_state;
};
}  // namespace hyped::state_machine