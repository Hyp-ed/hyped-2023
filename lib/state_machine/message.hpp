#pragma once

namespace hyped::state_machine {

// State class containing all the states
enum class Message {
  // TODOLater Add the actual messages
  mCalibrate,
  mReady,
  mAccelerating,
  mCruising,
  mBraking,
  mStoppped,
  mOff
};

}  // namespace hyped::state_machine