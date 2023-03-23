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
    = {{{State::kIdle, Message::kCalibrating}, State::kCalibrating},
       {{State::kCalibrating, Message::kReady}, State::kReady},
       {{State::kReady, Message::kAccelerating}, State::kAccelerating},
       {{State::kAccelerating, Message::kCruising}, State::kCruising},
       {{State::kCruising, Message::kMotorBrake}, State::kMotorBraking},
       {{State::kMotorBraking, Message::kPreFrictionBrake}, State::kPreFrictionBraking},
       {{State::kPreFrictionBraking, Message::kFrictionBrake}, State::kFrictionBraking},
       {{State::kFrictionBraking, Message::kStopped}, State::kStopped},
       {{State::kStopped, Message::kOff}, State::kOff},
       {{State::kAccelerating, Message::kFailureBrake}, State::kFailureBraking},
       {{State::kAccelerating, Message::kPreFrictionBrakeFail}, State::kPreFrictionBrakingFail},
       {{State::kPreFrictionBrakingFail, Message::kFrictionBrakeFail}, State::kFrictionBrakingFail},
       {{State::kCruising, Message::kFailureBrake}, State::kFailureBraking},
       {{State::kCruising, Message::kPreFrictionBrakeFail}, State::kPreFrictionBrakingFail},
       {{State::kMotorBraking, Message::kFailureBrake}, State::kFailureBraking},
       {{State::kMotorBraking, Message::kPreFrictionBrakeFail}, State::kPreFrictionBrakingFail},
       {{State::kFrictionBrakingFail, Message::kFailureBrake}, State::kFailureBraking},
       {{State::kFrictionBrakingFail, Message::kFailureStopped}, State::kFailureStopped},
       {{State::kFailureBraking, Message::kFailureStopped}, State::kFailureStopped},
       {{State::kFailureStopped, Message::kFailureOff}, State::kOff}};

  State current_state_;
};

}  // namespace hyped::state_machine