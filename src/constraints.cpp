#include "frontier_navigation.h"

bool frontierRegionConstraints(vec_single &frontierRegion, int threshold) {
    Helpers::writeToFile("error.xlsx", "size - threshold", frontierRegion.size(), threshold);
    if (frontierRegion.size() < threshold) return false;
    else return true;
}

bool oSpaceConstraint(geometry_msgs::Point &pt, const nav_msgs::OccupancyGrid::ConstPtr&  map) {
    MapOperations mapOps;
    unsigned int startCell;
    int xIts, yIts;
    mapOps.setupRectangleArea(pt, 0.5, map, startCell, xIts, yIts);
    unsigned int index;
    for (int i = 0; i < yIts; i++) {
        for (int j = 0; j < xIts; j++) {
            index = startCell + j + i*map->info.width;
            if (mapOps.isOSpace(index, map)) return false;
        }
    }
    return true;
}

bool uSpaceConstraint(geometry_msgs::Point &pt, const nav_msgs::OccupancyGrid::ConstPtr &map) {
    MapOperations mapOps;
    unsigned int startCell;
    int xIts, yIts;
    mapOps.setupRectangleArea(pt, 0.5, map, startCell, xIts, yIts);
    unsigned int index;
    for (int i = 0; i < yIts; i++) {
        for (int j = 0; j < xIts; j++) {
            index = startCell + j + i*map->info.width;
            if (mapOps.isUSpace(index, map)) return false;
        }
    }
    return true;
}

bool goalStateConstraints() {
    return true;
}

bool blackListConstraints(std::vector<geometry_msgs::PoseStamped> &blacklist, geometry_msgs::PoseStamped &goal, double threshold) {
    for (unsigned int i = 0; i < blacklist.size(); i++) {
        if (Helpers::distance(blacklist[i], goal) < threshold) return false;
    }
    return true;
}

bool goalConstraints(geometry_msgs::PoseStamped &goal, geometry_msgs::PoseStamped &robotPos, double threshold) {
    if (Helpers::distance(goal, robotPos) < threshold) return false;
    else return true;
}

bool whiteListConstraints(std::vector<geometry_msgs::PoseStamped> &whitelist, geometry_msgs::PoseStamped &goal, double threshold) {
    for (int i = whitelist.size()-1; i >= 0; i--) {
        if (Helpers::distance(goal, whitelist[i]) < threshold) return false;
    }
    return true;
}


bool Frontier_Navigation::evaluateFrontierRegion(vec_single &frontierRegion, bool print) {
    bool c1 = frontierRegionConstraints(frontierRegion, this->threshold_);
    if (print) {
        if (!c1) printf("Too small (%d)\n", frontierRegion.size());
    }
    return c1;
}

bool Frontier_Navigation::evaluateGoal(geometry_msgs::PoseStamped &goal, bool print) {
    bool c1 = blackListConstraints(this->blackList_, goal, 0.5);
    bool c2 = goalStateConstraints();
    bool c3 = goalConstraints(goal, this->robot_position_, 1.5);
    bool c4 = oSpaceConstraint(goal.pose.position, this->map_);
    bool c5 = uSpaceConstraint(goal.pose.position, this->map_);
    if (print) {

    }
//    if (!c1) printf("Goal is BLACKLISTED - try next goal\n");
//    if (!c2) printf("Goal triggers bad state\n");
//    if (!c3) printf("Goal is too close to robot\n");
//    if (!c4) printf("Goal is too close to o-Space\n");
//    if (!c5) printf("Goal is too close to u-Space\n");
    return c1 && c2 && c3 && c4 && c5;
}

bool Frontier_Navigation::evaluateWhitelist(geometry_msgs::PoseStamped &goal) {
    bool c1 = blackListConstraints(this->blackList_, goal, 0.5);
    bool c2 = whiteListConstraints(this->whiteListedGoals_, goal, 0.5);
    bool c3 = oSpaceConstraint(goal.pose.position, this->map_);
//    if (!c1) printf("Goal is BLACKLISTED and won't be added to whitelist\n");
//    if (!c2) printf("Goal already whitelisted\n");
//    if (!c3) printf("Goal is too close to o-Space and won't be added to whitelist\n");
    return c1 && c2 && c3;
}

bool Frontier_Navigation::evaluateBlackList(geometry_msgs::PoseStamped &goal) {
    bool c1 = blackListConstraints(this->blackList_, goal, 0.5);
    if (!c1) printf("Goal already blacklisted\n");
    return c1;
}

bool Frontier_Navigation::evaluateMapCallback() {
    bool c1 = Helpers::distance(this->activeGoal_, this->robot_position_) < 5.0;
    bool c2 = this->processState_ == PROCESSING_MAP_DONE;
    bool c3 = this->strategy_ == NORMAL;
    if (!c1) printf("Active goal in far distance\n");
    if (!c2) printf("Not in valid processState\n");
    if (!c3) printf("No valid strategy running\n");
    return c1 && c2 && c3;
}
