#include "frontier_navigation.h"

Frontier_Navigation::Frontier_Navigation(ros::NodeHandle* node_ptr)
{
    this->nodeHandle_ = node_ptr;
    this->rectangle_pub_ = this->nodeHandle_->advertise<nav_msgs::GridCells>("/searchRadius", 1, true);
    this->frontiers_pub_ = this->nodeHandle_->advertise<nav_msgs::GridCells>("/frontiers", 1, true);
    this->bestFrontier_pub_ = this->nodeHandle_->advertise<nav_msgs::GridCells>("/bestFrontier", 1, true);
    this->goal_pub_ = this->nodeHandle_->advertise<geometry_msgs::PoseStamped>("/sb_navigation/simple_goal", 1, true);
    this->circle_pub_ = this->nodeHandle_->advertise<nav_msgs::GridCells>("/circle", 1, true);
    this->pathTracker_pub_ = this->nodeHandle_->advertise<nav_msgs::GridCells>("/pathTracker", 1, true);
    this->zeros_pub_ = this->nodeHandle_->advertise<nav_msgs::GridCells>("/zeros", 1, true);
    this->min1_pub_ = this->nodeHandle_->advertise<nav_msgs::GridCells>("/min1", 1, true);
    this->min2_pub_ = this->nodeHandle_->advertise<nav_msgs::GridCells>("/min2", 1, true);
    this->min3_pub_ = this->nodeHandle_->advertise<nav_msgs::GridCells>("/min3", 1, true);
    this->min4_pub_ = this->nodeHandle_->advertise<nav_msgs::GridCells>("/min4", 1, true);
    this->filteredMap_pub_ = this->nodeHandle_->advertise<nav_msgs::OccupancyGrid>("/filteredMap", 1, true);
    this->blackList_pub_ = this->nodeHandle_->advertise<nav_msgs::GridCells>("/blacklist", 1, true);
    this->whiteListedGoals_pub_ = this->nodeHandle_->advertise<nav_msgs::GridCells>("whitelistedGoals", 1, true);
    this->whiteListedFrontierRegions_pub_ = this->nodeHandle_->advertise<nav_msgs::GridCells>("whitelistedFrontierRegions", 1, true);

    // how to do this properly??
    boost::shared_ptr<nav_msgs::OccupancyGrid> mapCopy2_(new nav_msgs::OccupancyGrid);
    this->map_ = mapCopy2_;

    this->processState_ = PROCESSING_MAP_DONE;
    this->strategy_ = NORMAL;

    this->cmd_vel_cnt_ = 0;
    this->mapCallbackCnt_ = 0;

    this->pathCounter_ = 0;
    this->pathTracker_.header.frame_id = "/map";
    // has to be fixed somehow
    // doesn't work with "= map->info.resolution" since map is not there yet
    this->pathTracker_.cell_height = this->pathTracker_.cell_width = 0.05;

    this->nodeHandle_->param("/frontier_navigation/radius", radius_, 5.0);
    this->nodeHandle_->param("/frontier_navigation/attempts", attempts_, 4);
    this->nodeHandle_->param("/frontier_navigation/stepping", stepping_, 5.0);
    this->nodeHandle_->param("/frontier_navigation/threshold", threshold_, 250);
    this->nodeHandle_->param("/frontier_navigation/sleep", sleep_, 0);
    this->nodeHandle_->param("/frontier_navigation/minDistance", minDinstance_, 3.0);
    this->nodeHandle_->param("/frontier_navigation/timeout", timeout_, 5.0);
    this->nodeHandle_->param("/frontier_navigation/timeoutAttempts", timeoutAttempts_, 5);
    this->nodeHandle_->param("/frontier_navigation/weightOfConnectivity", weightOfConnectivity_, 3.0);
    this->nodeHandle_->param("/frontier_navigation/worstCase", worstCaseOfConnectivity_, 2.0);
    this->nodeHandle_->param("/frontier_navigation/weightOfSize", weightOfSize_, 2.0);
    this->nodeHandle_->param("/frontier_navigation/weightOfDistance", weightOfDistance_, 1.0);
    this->nodeHandle_->param("/frontier_navigation/weightOfDirection", weightOfDirection_, 4.0);
    this->nodeHandle_->param("/frontier_navigation/explore", explore_, true);
    this->nodeHandle_->param("/frontier_navigation/removeWhitelistedGoalThreshold", removeWhitelistedGoalThreshold_, 0.2);
    this->nodeHandle_->param("/frontier_navigation/duplicatedGoals", duplicatedGoals_, 10);

    not_moving_timer_ = nodeHandle_->createTimer(ros::Duration(timeout_), &Frontier_Navigation::timerCallback, this, true);

}

void Frontier_Navigation::timerCallback(const ros::TimerEvent&) {
//    if (this->escapeStrategy_) break;
    ROS_WARN("\"not_moving_timer\" fired. Robot is likely to be stuck!!");
    strategies strategy = STUCK;
    escapeStrategy(strategy);
    // determine reason for being stuck and set flags for mapCallback
    // - too close to obstacle
    //   -> backup robot and change orientation
    // - reached goal but did not receive map update
    //   -> use saved frontires
    //   -> scan entire map
    // - no frontiers detected
    //   -> same as above
    // - bad goal
    //   -> ??
    // - not moving but sending 0-valued cmd_vel commands
    //   -> trigger recording of cmd_vel commands and check whether they are 0-valued or valid

//    nav_msgs::GridCells circle = Helpers::circleArea(this->robot_position_.pose.position, 1.0, this->map_);
//    int8_t unknown = 100;
//    for (int i = 0; i < circle.cells.size(); i++) {
//        if (this->map_->data[mapOps.pointToCell(circle.cells[i], this->map_)] == unknown) printf("STUCK??\n");
//    }
//    if (Helpers::distance(this->activeGoal_, this->robot_position_) <= 1.0) printf("GOAL REACHED??\n");
//    if (this->goalStatus_.status == actionlib_msgs::GoalStatus::REJECTED) printf("BAD GOAL??\n");
}

void Frontier_Navigation::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map) {
    printf("\n");
    printf("Frontier_Navigation received %s map\n", Helpers::getOrdinal(++this->mapCallbackCnt_));
    printf("\twidth: %d; height: %d; res: %f; x_org: %f; y_org: %f\n", map->info.width, map->info.height, map->info.resolution, map->info.origin.position.x, map->info.origin.position.y);
    map_->data = map->data;
    map_->header = map->header;
    map_->info = map->info;

    // check for duplicated goals
    int size = goalTracker_.cells.size();
    int cnt = 0;
    if (size > duplicatedGoals_) {
        for (int i = size-2; i >= size - duplicatedGoals_ - 1; i--) {
            if (Helpers::distance(goalTracker_.cells[size-1], goalTracker_.cells[i]) < 0.5) cnt++;
        }
        if (cnt == duplicatedGoals_) {
            printf("Duplicated goals detected\n");
            escapeStrategy(DUPLICATED_GOAL);
        } else if (cnt != 0) {
            printf("\t%d of %d goals are duplicates or equal!\n", cnt, duplicatedGoals_);
        }
    }
    if (processState_ == PROCESSING_MAP_DONE && strategy_ == NORMAL) {
        printf("Map will be processed\n");
        explore();
    } else printf("Map will NOT be processed\n");
}

void Frontier_Navigation::posCallback(const geometry_msgs::PoseStamped& robot_position) {
    robot_position_ = robot_position;
    pathCounter_++;
    if (pathCounter_%10 == 0) {
        pathTracker_.cells.push_back(robot_position.pose.position);
        pathTracker_pub_.publish(pathTracker_);
    }
}

void Frontier_Navigation::cmdVelCallback(const geometry_msgs::Twist& cmd_vel) {
    if (cmd_vel.angular.x == 0.0 && cmd_vel.angular.y == 0.0 && cmd_vel.angular.z == 0.0 &&
            cmd_vel.linear.x == 0.0 && cmd_vel.linear.y == 0.0 && cmd_vel.linear.z == 0) {
        if(++cmd_vel_cnt_ > 25) this->blackList_.push_back(this->activeGoal_);
    }
    else {
        cmd_vel_cnt_ = 0;
        not_moving_timer_ = nodeHandle_->createTimer(ros::Duration(timeout_), &Frontier_Navigation::timerCallback, this, true);
        cmdVelConstraints(cmd_vel);
    }
}

void Frontier_Navigation::goalStatusCallback(const actionlib_msgs::GoalStatus& goalStatus) {
    if (goalStatus.status == actionlib_msgs::GoalStatus::REJECTED) {
        printf("\tGoal in state REJECTED\n");
        escapeStrategy(GOAL_REJECTED);
    }
    if (goalStatus.status == actionlib_msgs::GoalStatus::SUCCEEDED) {
        if (this->strategy_ == DRIVE_TO_GOAL_BEFORE_UPDATE) {
            printf("\tGoal area entered. Strategy will be set to NORMAL\n");
            this->strategy_ = NORMAL;
        } else printf("\tGoal area entered.\n");
    }
    goalStatus_ = goalStatus;
}

void Frontier_Navigation::explore() {

    this->processState_ = PROCESSING_MAP_STARTED;

//    nav_msgs::GridCells outline;
//    this->min1_pub_.publish(Helpers::circleArea2(mapOps_.pointToCell(this->robot_position_.pose.position, map_), 100.0, map_, outline));
//    this->min2_pub_.publish(outline);

    bool success = false;
    double radius = this->radius_;
    geometry_msgs::PoseStamped goal;
    for (int i = 1; i <= this->attempts_; i++) {
        success = findNextGoal(this->robot_position_, radius, goal);
        if (success) {
            publishGoal(goal, true);
            break;
        } else if (i == attempts_) {
            printf("\tNo suitable frontierRegions found within maximum radius. STRATEGY NEEDED!!!\n");
            escapeStrategy(NO_FRONTIER_REGIONS_FOUND);
        } else {
            // increase radius and do it again
            radius += stepping_;
            printf("\tNo frontierRegion passed constraints within current radius\n");
            printf("\tSearch radius increased (old: %f, new: %f, stepping: %f)\n", radius-stepping_, radius, stepping_);
        }
    }

    this->processState_ = PROCESSING_MAP_DONE;

}

bool Frontier_Navigation::findNextGoal(geometry_msgs::PoseStamped &center, double radius, geometry_msgs::PoseStamped &goal) {

    vec_double frontierRegions;
    vec_double adjacencyMatrixOfFrontierCells;

    publishOutlineOfSearchRectangle(center, radius);
    // 1. filter map within given radius
    mapOps_.preFilterMap(map_, center, radius);
    filteredMap_pub_.publish(map_);
    // 2. find frontiers and prepare them for further processing
    mapOps_.findFrontierRegions(map_, center, radius, frontierRegions, adjacencyMatrixOfFrontierCells);
    if (frontierRegions.size() == 0) {
        printf("No frontierRegions detected within given radius\n");
        return false;
    }
    // 3. quality measure of frontierRegion
    std::vector<int> frontierRegionIDs = determineBestFrontierRegions(adjacencyMatrixOfFrontierCells, frontierRegions);
    publishFrontierPts(frontierRegions);
    // 4. select best frontierRegion
    printf("frontierRegion selection...\n");
    geometry_msgs::PoseStamped tempGoal;
    bool found = false;
    int currentFRID;
    for (int j = 0; j < frontierRegionIDs.size(); j++) {
        currentFRID = frontierRegionIDs[j];
        printf("\tfrontierRegion %d\t", currentFRID+1);
        if (!evaluateFrontierRegion(frontierRegions[currentFRID])) continue;
        tempGoal = nextGoal(frontierRegions[currentFRID]);
        if (!found && evaluateGoal(tempGoal)) {
            publishFrontierPts(frontierRegions, currentFRID);
            printf("Constraints passed!\n");
            goal = tempGoal;
            found = true;
        }
        if (evaluateWhitelist(tempGoal)) {
            this->whiteListedFrontierRegions_.push_back(frontierRegions[currentFRID]);
            this->whiteListedGoals_.push_back(tempGoal);
            printf("Added to whitelist\n");
        }
    }
    publishLists();
    clenupWhitelist();

    if (found) {
        return true;
    } else return false;
}

bool Frontier_Navigation::cmdVelConstraints(const geometry_msgs::Twist &cmd_vel, bool print)
{
//    geometry_msgs::Vector3_ zeroVec;
//    zeroVec.x = 0;
//    zeroVec.y = 0;
//    zeroVec.z = 0;
//    if (cmd_vel.angular.x == 0) printf("BAD");
}

geometry_msgs::PoseStamped Frontier_Navigation::nextGoal(vec_single frontier)
{
    unsigned int robotPos = mapOps_.pointToCell(this->robot_position_.pose.position, map_);
    geometry_msgs::Point close = mapOps_.cellToPoint(Helpers::closestPoint(frontier, robotPos, map_->info.width), map_);
    geometry_msgs::Point far = mapOps_.cellToPoint(Helpers::furthermostPoint(frontier, robotPos, map_->info.width), map_);
//    printf("pos(%f/%f/%f)\n", robot_position_.pose.position.x, robot_position_.pose.position.y, robot_position_.pose.position.z);
//    printf("close(%f/%f/%f)\n", close.x, close.y, close.z);
//    printf("far(%f/%f/%f)\n", far.x, far.y, far.z);

    // get frontiers vector
    geometry_msgs::Vector3 frontierVec;
    frontierVec.x = far.x - close.x;
    frontierVec.y = far.y - close.y;
    frontierVec.z = far.z - close.z;
//    printf("frontierVec(%f/%f/%f)\n", frontierVec.x, frontierVec.y, frontierVec.z);

    geometry_msgs::PoseStamped point;
    int goalIndex = Helpers::closestPoint(frontier, mapOps_.pointToCell(this->robot_position_.pose.position, this->map_), this->map_->info.width);
    point.pose.position = mapOps_.cellToPoint(goalIndex, map_);

    // get vector between robot and goal
    geometry_msgs::Vector3 robotToGoalVec;
    robotToGoalVec.x = point.pose.position.x - robot_position_.pose.position.x;
    robotToGoalVec.y = point.pose.position.y - robot_position_.pose.position.y;
    robotToGoalVec.z = point.pose.position.z - robot_position_.pose.position.z;
    double length = Helpers::length(robotToGoalVec);
//    printf("robotToGoalVec(%f/%f/%f)\n", robotToGoalVec.x, robotToGoalVec.y, robotToGoalVec.z);
//    printf("length of robotToGoalVec: %f\n", length);

    nav_msgs::GridCells circle = Helpers::circle(goalIndex, 1.5, map_);
    publishCircle(goalIndex);
    geometry_msgs::Vector3 goalToCircleVec;

    double desiredAngle = 80.0;
    double bestAnlgeDelta = 180.0;
    for (int i = 0; i < circle.cells.size(); i++) {
        goalToCircleVec.x = circle.cells[i].x - close.x;
        goalToCircleVec.y = circle.cells[i].y - close.y;
        goalToCircleVec.z = circle.cells[i].z - close.z;
        double angle = Helpers::angleInDegree(frontierVec, goalToCircleVec);        
        if (abs(desiredAngle - angle) < bestAnlgeDelta) {
            bestAnlgeDelta = abs(desiredAngle-angle);
            point.pose.position.x = circle.cells[i].x;
            point.pose.position.y = circle.cells[i].y;
            point.pose.position.z = circle.cells[i].z;
        }
//        printf("\tgoalToCircleVec(%f/%f/%f) angle: %f delta: %f\n", goalToCircleVec.x, goalToCircleVec.y, goalToCircleVec.z, angle, bestAnlgeDelata);
    }

    geometry_msgs::Vector3 x;
    x.x = 1.0;
    x.y = x.z = 0.0;
    geometry_msgs::Vector3 y;

    y.x = far.x - point.pose.position.x;
    y.y = far.y - point.pose.position.y;
    y.z = 0.0;

    double angle = Helpers::angleInDegree(y, x);

    if (y.y < 0) angle = 360-angle;
    tf::Quaternion quaternion = tf::createQuaternionFromYaw(angle*M_PI / 180);
    point.pose.orientation.x = quaternion.getX();
    point.pose.orientation.y = quaternion.getY();
    point.pose.orientation.z = quaternion.getZ();
    point.pose.orientation.w = quaternion.getW();

//    printf("Next Goal! goal(%f, %f, %f)\n", point.pose.position.x, point.pose.position.y, point.pose.position.z);
    return point;
}

void Frontier_Navigation::clenupWhitelist() {

    printf("\tCleaning up whitelist...\n");

    int cnt = 0;
    int cells = 0;

    std::vector<geometry_msgs::PoseStamped>::iterator goalIterator = whiteListedGoals_.begin();
    std::vector<vec_single>::iterator frIterator = whiteListedFrontierRegions_.begin();

    bool keepGoal;
    double uSpace;
    for (frIterator; frIterator != whiteListedFrontierRegions_.end();) {
        keepGoal = true;
        uSpace = 0.0;
        for (int i = 0; i < (*frIterator).size(); i++) {
            if (this->mapOps_.neighbourhoodValue((*frIterator)[i], this->map_) < 0) uSpace += 1.0;
        }
        if (uSpace / double((*frIterator).size()) < this->removeWhitelistedGoalThreshold_) keepGoal = false;

        if (keepGoal) {
            cells += (*frIterator).size();
            ++goalIterator;
            ++frIterator;
        } else {
            goalIterator = whiteListedGoals_.erase(goalIterator);
            frIterator = whiteListedFrontierRegions_.erase(frIterator);
            cnt++;
        }
    }

    printf("\t\t%d goals removed from whitelist\n", cnt);
    printf("\t\twhitelisted goals:\t%d\n", this->whiteListedGoals_.size());
    printf("\t\tassociated cells:\t%d\n", cells);
    printf("\t\tblacklisted goals:\t%d\n", this->blackList_.size());
}

void Frontier_Navigation::clenupBlacklist() {

    printf("Cleaning up blacklist...\n");

    int cnt = 0;
    printf("\tNOT IMPLEMENTED YET \n");
    printf("\t%d goals removed from blacklist\n", cnt);
}

bool Frontier_Navigation::findWhiteListedGoal() {
    printf("Going through white listed frontierRegions...\n");
    if (whiteListedFrontierRegions_.size() == 0) {
        printf("\tNo frontierRegions in whitelist\n");
        return false;
    }
    std::vector<int> frontierRegionIDs = determineBestFrontierRegions(whiteListedFrontierRegions_);
    publishFrontierPts(whiteListedFrontierRegions_);
    printf("whitelisted frontierRegion selection...\n");
    geometry_msgs::PoseStamped tempGoal;
    bool found = false;
    int currentFRID;
    for (int i = 0; i < frontierRegionIDs.size(); i++) {
        currentFRID = frontierRegionIDs[i];
        printf("\twhitelisted frontierRegion %d\t", currentFRID+1);
        tempGoal = nextGoal(whiteListedFrontierRegions_[currentFRID]);
        if (evaluateGoal(whiteListedGoals_[currentFRID])) {
            publishFrontierPts(whiteListedFrontierRegions_, currentFRID);
            printf("Constraints passed!\n");
            tempGoal = whiteListedGoals_[currentFRID];
            found = true;
            break;
        }
    }
    if (found) {
        printf("\tNew goal found\n");
        publishGoal(tempGoal, true);
        return true;
    } else {
        printf("\tNew goal NOT found\n");
        return false;
    }
}

void Frontier_Navigation::escapeStrategy(strategies strategy) {

    if (strategy_ == NORMAL) {
        strategy_ = strategy;
        switch (strategy_) {
        case NO_FRONTIER_REGIONS_FOUND: {
            printf("NO_FRONTIER_REGIONS_FOUND strategy initiated...\n");
            if (findWhiteListedGoal()) {
                this->strategy_ = NORMAL;
            }
            else {
                printf("Searching across entire map...\n");
                geometry_msgs::PoseStamped center;
                double radius = this->map_->info.width/2 * map_->info.resolution;
                center.pose.position.x = map_->info.origin.position.x + radius;
                center.pose.position.y = map_->info.origin.position.y + radius;
                if (findNextGoal(center, radius, this->activeGoal_)) {
                    strategy_ = DRIVE_TO_GOAL_BEFORE_UPDATE;
                    printf("DRIVE_TO_GOAL_BEFORE_UPDATE strategy initiated...\n");
                    publishGoal(this->activeGoal_, true);
                    publishLists();
                }
                else printf("\tMap seems to be explored totally\n");
            }
            break;
        }
        case STUCK: {
            printf("STUCK strategy initiated...\n");
            if (this->goalStatus_.status == actionlib_msgs::GoalStatus::SUCCEEDED) {
                printf("\tGoal area entered and no action triggered\n");
                this->strategy_ = NORMAL;
                escapeStrategy(NO_FRONTIER_REGIONS_FOUND);
            }
            if (this->goalStatus_.status == actionlib_msgs::GoalStatus::REJECTED) {
                this->goalStatus_.status = actionlib_msgs::GoalStatus::SUCCEEDED;
                printf("\tGoal rejected\n");
                this->strategy_ = NORMAL;
                not_moving_timer_ = nodeHandle_->createTimer(ros::Duration(timeout_), &Frontier_Navigation::timerCallback, this, true);
                escapeStrategy(GOAL_REJECTED);
            }
            // - reached goal but did not receive map update
            //   -> go through whitelist
            // - setting same goal multiple times. happens when robot is within goal area and map update doesn't
            //   change anything
            //   -> solved by DUPLICATED_GOAL
            // - zero vel_cmd commands
            // - no map_update for other reasons
            strategy_ = NORMAL;
            break;
        }
        case GOAL_REJECTED: {
            printf("GOAL_REJECTED strategy initiated...\n");
            // - caused by path going through u-Space
            //   -> blacklist last goal
            //   -> map update is coming -> all good
            //   -> map update is NOT coming -> STUCK
            this->blackList_.push_back(this->activeGoal_);
            printf("\tBad goal added to blacklist\n");
            Helpers::writeToFile("blacklist.txt", "rejected");
            strategy_ = NORMAL;
            not_moving_timer_ = nodeHandle_->createTimer(ros::Duration(timeout_), &Frontier_Navigation::timerCallback, this, true);
            break;
        }
        case DUPLICATED_GOAL: {
            printf("DUPLICATED_GOAL strategy initiated...\n");
            this->blackList_.push_back(this->activeGoal_);
            Helpers::writeToFile("blacklist.txt", "duplicated");
            printf("\tDuplicated goal added to blacklist\n");
            strategy_ = NORMAL;
            break;
        }
//        case DRIVE_TO_GOAL_BEFORE_UPDATE: {
//            printf("DRIVE_TO_GOAL_BEFORE_UPDATE strategy initiated...\n");
//            strategy_ = NORMAL;
//        }
        default: printf("Strategy not implemented yet...\n");
        }
    } else printf("Strategy rejected. Another strategy already running...\n");
}
