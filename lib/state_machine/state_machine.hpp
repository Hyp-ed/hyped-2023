#pragma once

#include "consts.hpp"
#include "state.hpp"
#include "types.hpp"

#include <optional>

#include <boost/unordered_map.hpp>

namespace hyped::state_machine {

class StateMachine {
 public:
  StateMachine() { current_state = State::kIdle; };

  void handleMessage(const Message message);

  std::optional<Message> handleData();

 private:
  const boost::unordered_map<SourceAndMessage, State> transitionMap
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
       {{State::kAccelerating, Message::mMitigate}, State::kMitigate},
       {{State::kCruising, Message::mFailure}, State::kFailureBraking},
       {{State::kCruising, Message::mFailure}, State::kFrictionBraking},
       {{State::kCruising, Message::mFailure}, State::kMotorBraking},
       {{State::kCruising, Message::mMitigate}, State::kMitigate},
       {{State::kNominalBraking, Message::mMitigate}, State::kMitigate},
       {{State::kNominalBraking, Message::mFailure}, State::kMotorBraking},
       {{State::kNominalBraking, Message::mFailure}, State::kFrictionBraking},
       {{State::kNominalBraking, Message::mFailure}, State::kFailureBraking},
       {{State::kMotorBraking, Message::mFailure}, State::kFailureBraking},
       {{State::kMotorBraking, Message::mFailure}, State::kFrictionBraking},
       {{State::kMotorBraking, Message::mFailure}, State::kFailureStopped},
       {{State::kFrictionBraking, Message::mFailure}, State::kFailureBraking},
       {{State::kFrictionBraking, Message::mFailure}, State::kFailureStopped},
       {{State::kFailureBraking, Message::mFailure}, State::kFailureStopped},
       {{State::kFailureStopped, Message::mFailure}, State::kOff},
       {{State::kPEAccelerating, Message::mFailure}, State::kMotorBraking},
       {{State::kPECruising, Message::mFailure}, State::kMotorBraking},
       {{State::kPEBraking, Message::mFailure}, State::kMotorBraking},
       {{State::kPEAccelerating, Message::mFailure}, State::kFrictionBraking},
       {{State::kPECruising, Message::mFailure}, State::kFrictionBraking},
       {{State::kPEBraking, Message::mFailure}, State::kFrictionBraking},
       {{State::kPEAccelerating, Message::mFailure}, State::kFailureBraking},
       {{State::kPECruising, Message::mFailure}, State::kFailureBraking},
       {{State::kPEBraking, Message::mFailure}, State::kFailureBraking},
       {{State::kMitigate, Message::mPostEmergency}, State::kPEAccelerating},
       {{State::kMitigate, Message::mPostEmergency}, State::kPECruising},
       {{State::kMitigate, Message::mPostEmergency}, State::kPEBraking},
       {{State::kMitigate, Message::mPostEmergency}, State::kMotorBraking},
       {{State::kMitigate, Message::mPostEmergency}, State::kFrictionBraking},
       {{State::kMitigate, Message::mPostEmergency}, State::kFailureBraking},
       {{State::kPEAccelerating, Message::mResume}, State::kAccelerating},
       {{State::kPECruising, Message::mResume}, State::kCruising},
       {{State::kPEBraking, Message::mResume}, State::kNominalBraking}};

  State current_state;
};
}  // namespace hyped::state_machine