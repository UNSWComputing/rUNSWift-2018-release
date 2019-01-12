#include "BallFilter.hpp"

#include <string>
#include <sstream>

const int NO_CLOSE_OBSERVATION = -1;

//Make my life easier for iterating over vectors
#define FOR_EACH(index, vector) for (unsigned int index = 0; index < vector.size(); ++index)

std::vector<RobotObstacle> BallFilter::update(const RobotFilterUpdate &update) {

    //Only update visual robots if not incapacitated
    if (!update.isIncapacitated) {

        std::vector<GroupedRobots>::iterator it = groupedRobots.begin();

        while (it != groupedRobots.end()) {
            GroupedRobots &group = (*it);
            group.tick(update.odometryDiff, update.headYaw, update.robotPos);
            if (group.isEmpty()) {
                it = groupedRobots.erase(it);
            } else {
                ++it;
            }
        }


        //Greedy algorithm to determine what observation goes into what group.
        //Current assumptions: multiple observations cannot go into the same group,
        //even if they are close together. The closest observation gets merge
        //into the group so there is many cases where this is suboptimal but as
        //this is a NP Complete problem it is good enough.
        std::vector<bool> observationMerged(update.visualRobots.size(), false);
        FOR_EACH(groupIndex, groupedRobots) {
            GroupedRobots &group = groupedRobots[groupIndex];

            int smallestIndex = NO_CLOSE_OBSERVATION;
            double smallestDistance = 0;
            FOR_EACH(visualIndex, update.visualRobots) {

                if (!observationMerged[visualIndex]) {

                    const RobotInfo &visualRobot = update.visualRobots[visualIndex];
                    if (group.canMergeRobot(visualRobot)) {

                        double distance = group.distanceToRobot(visualRobot);
                        if (smallestIndex == -1 || smallestDistance > distance) {
                            smallestIndex = visualIndex;
                            smallestDistance = distance;
                        }
                    }
                }
            }

            if (smallestIndex != -1) {
                const RobotInfo &observation = update.visualRobots[smallestIndex];
                group.mergeRobot(observation);
                observationMerged[smallestIndex] = true;
            }
        }

        FOR_EACH(visualIndex, update.visualRobots) {
            if (!observationMerged[visualIndex]) {
                GroupedRobots group(update.visualRobots[visualIndex]);
                groupedRobots.push_back(group);
            }
        }
    }

    filteredRobots = generateRobotObstacles();

    return filteredRobots;
}

std::vector<RobotObstacle> BallFilter::generateRobotObstacles() const {
    std::vector<RobotObstacle> obstacles;

    FOR_EACH(groupedIndex, groupedRobots) {
        const GroupedRobots &group = groupedRobots[groupedIndex];
        if (group.isOnField() && group.isImportantObstacle()) {
            std::string type;
            if (group.getType() == RobotInfo::rUnknown) {
                type = "unknown";
            } else if (group.getType() == RobotInfo::rBlue) {
                type = "blue";
            } else {
                type = "red";
            }
            obstacles.push_back(group.generateRobotObstacle());
        }
    }

    return obstacles;
}
