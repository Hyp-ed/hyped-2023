#pragma once

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

  void handleMessage(const Message &message);
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
       {{State::kCruising, Message::kNextNominalState}, State::kMotorBraking},
       {{State::kMotorBraking, Message::kNextNominalState}, State::kPreFrictionBraking},
       {{State::kPreFrictionBraking, Message::kNextNominalState}, State::kFrictionBraking},
       {{State::kFrictionBraking, Message::kNextNominalState}, State::kStopped},
       {{State::kStopped, Message::kNextNominalState}, State::kOff},
       {{State::kAccelerating, Message::kFailureBrake}, State::kFailureBraking},
       {{State::kAccelerating, Message::kPreFrictionBrake}, State::kPreFrictionBraking},
       {{State::kPreFrictionBraking, Message::kFrictionBrake}, State::kFrictionBraking},
       {{State::kCruising, Message::kFailureBrake}, State::kFailureBraking},
       {{State::kCruising, Message::kPreFrictionBrake}, State::kPreFrictionBraking},
       {{State::kPreFrictionBraking, Message::kFrictionBrake}, State::kFrictionBraking},
       {{State::kMotorBraking, Message::kFailureBrake}, State::kFailureBraking},
       {{State::kMotorBraking, Message::kPreFrictionBrake}, State::kPreFrictionBraking},
       {{State::kPreFrictionBraking, Message::kFrictionBrake}, State::kFrictionBraking},
       {{State::kFrictionBraking, Message::kFailureBrake}, State::kFailureBraking},
       {{State::kFrictionBraking, Message::kFailure}, State::kFailureStopped},
       {{State::kFailureBraking, Message::kFailure}, State::kFailureStopped},
       {{State::kFailureStopped, Message::kFailure}, State::kOff}};

  State current_state_;
};

}  // namespace hyped::state_machine