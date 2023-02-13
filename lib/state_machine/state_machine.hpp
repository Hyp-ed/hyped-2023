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
  StateMachine();

  // TODOLater handle ros messages and transition accordingly
  std::optional<Message> checkTransition();

  void handleMessage(const Message message);

  Message stringToMessage(const std::string &message_name);

  std::string messageToString(const Message &message);

 private:
  const std::unordered_map<std::string, Message> string_to_message_
    = {{"kNextNominalState", Message::kNextNominalState}, {"kFailure", Message::kFailure}};

  const std::unordered_map<Message, std::string> message_to_string_
    = {{Message::kNextNominalState, "kNextNominalState"}, {Message::kFailure, "kFailure"}};

  const boost::unordered_map<SourceAndMessage, State> transition_to_state_
    = {{{State::kIdle, Message::kNextNominalState}, State::kCalibrating},
       {{State::kCalibrating, Message::kNextNominalState}, State::kReady},
       {{State::kReady, Message::kNextNominalState}, State::kAccelerating},
       {{State::kAccelerating, Message::kNextNominalState}, State::kCruising},
       {{State::kCruising, Message::kNextNominalState}, State::kNominalBraking},
       {{State::kNominalBraking, Message::kNextNominalState}, State::kStopped},
       {{State::kStopped, Message::kNextNominalState}, State::kOff},
       {{State::kAccelerating, Message::kFailureBrake}, State::kFailureBraking},
       {{State::kAccelerating, Message::kMotorBrake}, State::kMotorBraking},
       {{State::kAccelerating, Message::kFrictionBrake}, State::kFrictionBraking},
       {{State::kCruising, Message::kFailureBrake}, State::kFailureBraking},
       {{State::kCruising, Message::kFrictionBrake}, State::kFrictionBraking},
       {{State::kCruising, Message::kMotorBrake}, State::kMotorBraking},
       {{State::kNominalBraking, Message::kMotorBrake}, State::kMotorBraking},
       {{State::kNominalBraking, Message::kFrictionBrake}, State::kFrictionBraking},
       {{State::kNominalBraking, Message::kFailureBrake}, State::kFailureBraking},
       {{State::kMotorBraking, Message::kFailureBrake}, State::kFailureBraking},
       {{State::kMotorBraking, Message::kFrictionBrake}, State::kFrictionBraking},
       {{State::kMotorBraking, Message::kFailure}, State::kFailureStopped},
       {{State::kFrictionBraking, Message::kFailureBrake}, State::kFailureBraking},
       {{State::kFrictionBraking, Message::kFailure}, State::kFailureStopped},
       {{State::kFailureBraking, Message::kFailure}, State::kFailureStopped},
       {{State::kFailureStopped, Message::kFailure}, State::kOff}};

  State current_state_;
};
}  // namespace hyped::state_machine