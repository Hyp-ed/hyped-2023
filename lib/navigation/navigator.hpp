#include "consts.hpp"

namespace hyped::navigation{

class Navigator{

  public:
  Navigator();

  /**
  *@brief Does navigation. Combines all navigation functionality to produce the best
  *current estimate of trajectory
  */
  void navigate();

  /**
  * @brief Publishes the current navigation trajectory to wherever it goes
  * TODO: make this more informative once implemented
  */
  void publishTrajectory();

  private:
  
  //current navigation trajectory
  Trajectory trajectory_;

  //current keyence value for displacement
  int keyenceDisplacement_;
};
}  // namespace hyped::navigation
