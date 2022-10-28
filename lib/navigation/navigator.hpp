#include "consts.hpp"

namespace hyped::navigation{

class Navigator{

  public:

  Navigator();

  void navigate();

  void publishTrajectory();

  private:

  Trajectory trajectory_;


};

}  // namespace hyped::navigation
