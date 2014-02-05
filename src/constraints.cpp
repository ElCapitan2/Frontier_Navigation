#include "frontier_navigation.h"

bool frontierRegionConstraints(vec_single &frontierRegion, int threshold) {
    if (frontierRegion.size() < threshold) {
        printf("\tFrontierRegion is too small - find next frontierRegion\n");
        return false;
    } else return true;
}

bool goalStateConstraints() {
    return true;
}

bool goalConstraints(std::vector<geometry_msgs::PoseStamped> &blacklist, geometry_msgs::PoseStamped &robotPos, geometry_msgs::PoseStamped &goal, double threshold) {
    // check if goal is blacklisted
    for (int i = 0; i < blacklist.size(); i++) {
        if (Helpers::distance(blacklist[i], goal) < threshold) {
            printf("\tGoal is BLACKLISTED - find next goal\n");
            return false;
        }
    }
    // check if goal is not within safety area of robot
    if (Helpers::distance(goal, robotPos) < 1.0) {
        printf("\tGoal is too close to robot - find next goal\n");
        return false;
    }
    return true;
}


bool Frontier_Navigation::evaluateConstraints(vec_single &frontierRegion, geometry_msgs::PoseStamped &goal) {
    bool c1 = frontierRegionConstraints(frontierRegion, this->threshold_);
    bool c2 = goalStateConstraints();
    bool c3 = goalConstraints(this->blackList_, this->robot_position_, goal, 0.5);
    return c1 && c2 && c3;
}
