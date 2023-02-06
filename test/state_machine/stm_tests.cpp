#include <gtest/gtest.h>

#include <state_machine/state_machine.hpp>

namespace hyped::test {

TEST(Transition, construction) {
    state_machine::StateMachine testSM;
    testSM.handleMessage(state_machine::Message::mForward);
    assert(testSM.current_state == state_machine::State::kCalibrating);
}
}  // namespace hyped::test