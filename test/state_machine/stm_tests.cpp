#include <gtest/gtest.h>

#include <state_machine/state_machine.hpp>

namespace hyped::test {

state_machine::StateMachine testSM;

void testTransition(state_machine::Message message, state_machine::State expected_state) {
    testSM.handleMessage(message);
    assert(testSM.current_state == expected_state);
}

TEST(Transition, forward) {
    //testSM.handleMessage(state_machine::Message::mForward);
    //assert(testSM.current_state == state_machine::State::kCalibrating);
    testTransition(state_machine::Message::mForward, state_machine::State::kCalibrating);
    testTransition(state_machine::Message::mForward, state_machine::State::kReady);
    testTransition(state_machine::Message::mForward, state_machine::State::kAccelerating);
    testTransition(state_machine::Message::mForward, state_machine::State::kCruising);
    testTransition(state_machine::Message::mForward, state_machine::State::kNominalBraking);
    testTransition(state_machine::Message::mForward, state_machine::State::kStopped);
    testTransition(state_machine::Message::mForward, state_machine::State::kOff);
}
}  // namespace hyped::test