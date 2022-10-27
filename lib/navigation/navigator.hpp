#include "consts.hpp"

namespace hyped::navigation{

class Navigator{

  public:

  Navigator();

  void navigate();

  Trajectory trajectory_;

  void updateTrajectory();

};

}  // namespace hyped::navigation
