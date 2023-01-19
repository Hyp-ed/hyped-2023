#pragma once

#include "consts.hpp"
#include "state.hpp"
#include "types.hpp"

#include <array>
#include <optional>

namespace hyped::state_machine {

class StateMachine {
 public:
  StateMachine() { current_state = State::kIdle; };

  void transition(const Message message);

  std::optional<Message> checkTransition();

  void reset();

  // TODO change transitions to actual transitions
  std::array<Transition, kNumTransitions> transitions{
    {{State::kIdle, State::kCalibrating, Message::mForward},
     {State::kCalibrating, State::kReady, Message::mForward},
     {State::kReady, State::kAccelerating, Message::mForward},
     {State::kAccelerating, State::kCruising, Message::mForward},
     {State::kCruising, State::kNominalBraking, Message::mForward},
     {State::kNominalBraking, State::kStopped, Message::mForward},
     {State::kStopped, State::kOff, Message::mForward},
     {State::kAccelerating, State::kFailureBraking, Message::mFailure},
     {State::kAccelerating, State::kNominalBraking, Message::mFailure},
     {State::kAccelerating, State::kMotorBraking, Message::mFailure},
     {State::kAccelerating, State::kFrictionBraking, Message::mFailure},
     {State::kAccelerating, State::kMitigate, Message::mMitigate},
     {State::kCruising, State::kFailureBraking, Message::mFailure},
     {State::kCruising, State::kFrictionBraking, Message::mFailure},
     {State::kCruising, State::kMotorBraking, Message::mFailure},
     {State::kCruising, State::kMitigate, Message::mMitigate},
     {State::kNominalBraking, State::kMitigate, Message::mMitigate},
     {State::kNominalBraking, State::kMotorBraking, Message::mFailure},
     {State::kNominalBraking, State::kFrictionBraking, Message::mFailure},
     {State::kNominalBraking, State::kFailureBraking, Message::mFailure},
     {State::kMotorBraking, State::kFailureBraking, Message::mFailure},
     {State::kMotorBraking, State::kFrictionBraking, Message::mFailure},
     {State::kMotorBraking, State::kFailureStopped, Message::mFailure},
     {State::kFrictionBraking, State::kFailureBraking, Message::mFailure},
     {State::kFrictionBraking, State::kFailureStopped, Message::mFailure},
     {State::kFailureBraking, State::kFailureStopped, Message::mFailure},
     {State::kFailureStopped, State::kOff, Message::mFailure},
     {State::kPEAccelerating, State::kMotorBraking, Message::mFailure},
     {State::kPECruising, State::kMotorBraking, Message::mFailure},
     {State::kPEBraking, State::kMotorBraking, Message::mFailure},
     {State::kPEAccelerating, State::kFrictionBraking, Message::mFailure},
     {State::kPECruising, State::kFrictionBraking, Message::mFailure},
     {State::kPEBraking, State::kFrictionBraking, Message::mFailure},
     {State::kPEAccelerating, State::kFailureBraking, Message::mFailure},
     {State::kPECruising, State::kFailureBraking, Message::mFailure},
     {State::kPEBraking, State::kFailureBraking, Message::mFailure},
     {State::kMitigate, State::kPEAccelerating, Message::mPostEmergency},
     {State::kMitigate, State::kPECruising, Message::mPostEmergency},
     {State::kMitigate, State::kPEBraking, Message::mPostEmergency},
     {State::kMitigate, State::kMotorBraking, Message::mPostEmergency},
     {State::kMitigate, State::kFrictionBraking, Message::mPostEmergency},
     {State::kMitigate, State::kFailureBraking, Message::mPostEmergency},
     {State::kPEAccelerating, State::kAccelerating, Message::mResume},
     {State::kPECruising, State::kCruising, Message::mResume},
     {State::kPEBraking, State::kNominalBraking, Message::mResume}}};

  State current_state;
};
}  // namespace hyped::state_machine