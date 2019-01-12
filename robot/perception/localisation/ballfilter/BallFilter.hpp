#pragma once

#include <vector>
#include <Eigen/Eigen>
#include "types/AbsCoord.hpp"
#include "types/RRCoord.hpp"
#include "types/Odometry.hpp"
#include "types/RobotInfo.hpp"
#include "types/GroupedRobots.hpp"
#include "types/RobotFilterUpdate.hpp"
#include "perception/localisation/LocalisationDefs.hpp"

/**
 * Robot filter deals with deciding what observations should be placed into
 * what groups depending on whether it is possible to merge it into that group.
 */
class BallFilter {
   public:
      std::vector<RobotObstacle> update(const RobotFilterUpdate &update);
      std::vector<RobotObstacle> filteredRobots;

   private:
      std::vector<RobotObstacle> generateRobotObstacles() const;
      std::vector<GroupedRobots> groupedRobots;
};

