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

    // how to do this properly??
    boost::shared_ptr<nav_msgs::OccupancyGrid> mapCopy2_(new nav_msgs::OccupancyGrid);
    this->map_ = mapCopy2_;

    this->processState_ = PROCESSING_MAP_DONE;
    this->strategy_ = NORMAL;

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

void Frontier_Navigation::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr&  map) {
    printf("\n");
    printf("Frontier_Navigation received map\n");
    printf("\twidth: %d; height: %d; res: %f; x_org: %f; y_org: %f\n", map->info.width, map->info.height, map->info.resolution, map->info.origin.position.x, map->info.origin.position.y);
    map_->data = map->data;
    map_->header = map->header;
    map_->info = map->info;

    // check for duplicated goals
    int size = goalTracker_.cells.size();
    if (size > 1 && Helpers::distance(goalTracker_.cells[size-1], goalTracker_.cells[size-2]) < 0.5) {
        printf("Duplicatet goals detected\n");
        strategies strategy = DUPLICATED_GOAL;
        escapeStrategy(strategy);
    }
    else {
        if (processState_ == PROCESSING_MAP_DONE && strategy_ == NORMAL) {
            printf("Map will be processed\n");
            processMap(robot_position_);
        } else printf("Map will NOT be processed\n");
    }
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
            cmd_vel.linear.x == 0.0 && cmd_vel.linear.y == 0.0 && cmd_vel.linear.z == 0);
    else {
        not_moving_timer_ = nodeHandle_->createTimer(ros::Duration(timeout_), &Frontier_Navigation::timerCallback, this, true);
        cmdVelConstraints(cmd_vel);
    }
}

void Frontier_Navigation::goalStatusCallback(const actionlib_msgs::GoalStatus& goalStatus) {
    if (goalStatus.status == actionlib_msgs::GoalStatus::SUCCEEDED) printf("SUCEEDED!\n");
    if (goalStatus.status == actionlib_msgs::GoalStatus::REJECTED) {
        ROS_WARN("Goal in state REJECTED");
        strategies strategy = GOAL_REJECTED;
        escapeStrategy(strategy);
    }
    goalStatus_ = goalStatus;
}

void Frontier_Navigation::processMap(geometry_msgs::PoseStamped center) {

    processState_ = PROCESSING_MAP_STARTED;



    vec_double frontierRegions;
    vec_double adjacencyMatrixOfFrontierCells;

    double radius;
    if (strategy_ == NO_FRONTIER_REGIONS_FOUND) {
        radius = this->map_->info.width/2 * map_->info.resolution;
        center.pose.position.x = map_->info.origin.position.x + radius;
        center.pose.position.y = map_->info.origin.position.y + radius;
    } else radius = radius_;

    for (int i = 1; i <= attempts_; i++) {

        publishOutlineOfSearchRectangle(center, radius);

        // 1. filter map within given radius
        mapOps_.preFilterMap(map_, center, radius);
        filteredMap_pub_.publish(map_);

        // 2. find frontiers and prepare them for further processing
        mapOps_.findFrontierRegions(map_, center, radius, frontierRegions, adjacencyMatrixOfFrontierCells);
        if (frontierRegions.size() == 0) {
            radius += stepping_;
            printf("No frontierRegions detected within given radius\n");
            printf("Search radius increased (old: %f, new: %f, stepping: %f)\n", radius-stepping_, radius, stepping_);
            continue;
        }

        // 3. find best suitable frontierRegion
        bool found = false;
        std::vector<int> frontierRegionIDs = determineBestFrontierRegions(adjacencyMatrixOfFrontierCells, frontierRegions);
        for (int j = 0; j < frontierRegionIDs.size(); j++) {
            geometry_msgs::PoseStamped goal = nextGoal(frontierRegions[j]);
            bool add = true;
            for (int k = 0; k < whiteList_.size(); k++) {
                if (Helpers::distance(goal, goals_[k]) < 1.0) {
                    add = false;
                    break;
                }
            }
            if (add) {
                whiteList_.push_back(frontierRegions[j]);
                goals_.push_back(goal);
                printf("\tfrontierRegion %d added\n", j+1);
            } else printf("\tfrontierRegion %d NOT added\n", j+1);
            add = true;
        }
        nav_msgs::GridCells min1;
        min1.cell_height = min1.cell_width = map_->info.resolution;
        min1.header.frame_id = "/map";
        int cnt = 0;
        for (int j = 0; j < whiteList_.size(); j++) {
            cnt += whiteList_[j].size();
            for (int k = 0; k < whiteList_[j].size(); k++) min1.cells.push_back(mapOps_.cellToPoint(whiteList_[j][k], this->map_));
        }

        min1_pub_.publish(min1);

        printf("\tAmount of stored frontierRegions (white list): %d\n", whiteList_.size());
        printf("\tAmount of stored frontierCells: %d\n", cnt);

        Helpers::writeToFile("whiteList1.txt", "", whiteList_.size());
        Helpers::writeToFile("whiteList2.txt", "", cnt);

        publishFrontierPts(frontierRegions);
        printf("Frontier selection...\n");
        for (int j = 0; j < frontierRegionIDs.size(); j++) {
            if (frontierConstraints(frontierRegions[frontierRegionIDs[j]], true)) {
                publishFrontierPts(frontierRegions, frontierRegionIDs[j]);
                geometry_msgs::PoseStamped goal = nextGoal(frontierRegions[frontierRegionIDs[j]]);
                activeGoal_ = goal;
                printf("\tfrontier %d of %d size: %d - Constraints passed!\n", frontierRegionIDs[j], frontierRegions.size(), frontierRegions[frontierRegionIDs[j]].size());
                printf("\tNext Goal! goal(%f, %f, %f)\n", activeGoal_.pose.position.x, activeGoal_.pose.position.y, activeGoal_.pose.position.z);
                publishGoal(goal);
                goalTracker_.cells.push_back(goal.pose.position);
                found = true;
                break;
            } else {
                printf("\tfrontier %d of %d size: %d - Constraints NOT passed!\n", frontierRegionIDs[j], frontierRegions.size(), frontierRegions[frontierRegionIDs[j]].size());
            }
        }

        if (found) break;
        else if (i == attempts_) {
            printf("\tNo suitable frontierRegions found within maximum radius. STRATEGY NEEDED!!!\n");
            strategies strategy = NO_FRONTIER_REGIONS_FOUND;
            escapeStrategy(strategy);
        } else {
            // 3.c increase radius and do it again
            radius += stepping_;
            printf("\tNo frontierRegion passed constraints within current radius\n");
            printf("\tSearch radius increased (old: %f, new: %f, stepping: %f)\n", radius-stepping_, radius, stepping_);
            frontierRegions.clear();
            adjacencyMatrixOfFrontierCells.clear();
        }
    }
    processState_ = PROCESSING_MAP_DONE;
}

// Define constraints which are necassary for further processing of found connected frontiers
// I.e. set minimum amount of points in set of connected frontiers
bool Frontier_Navigation::frontierConstraints(vec_single &frontier, bool print) {
    bool frontierSize (frontier.size() > threshold_);
    bool goalStatus = true;
//    bool goalStatus = (goalStatus_.status != actionlib_msgs::GoalStatus::REJECTED);
    if (print) printf("\tConstraints: size = %s; goalStatus = %s\n", (frontierSize)?"true":"false", (goalStatus)?"true":"false");
    return (frontierSize && goalStatus);
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

    double desiredAnle = 80.0;
    double bestAnlgeDelata = 180.0;
    for (int i = 0; i < circle.cells.size(); i++) {
        goalToCircleVec.x = circle.cells[i].x - close.x;
        goalToCircleVec.y = circle.cells[i].y - close.y;
        goalToCircleVec.z = circle.cells[i].z - close.z;
        double angle = Helpers::angleInDegree(frontierVec, goalToCircleVec);        
        if (abs(desiredAnle - angle) < bestAnlgeDelata) {
            bestAnlgeDelata = abs(desiredAnle-angle);
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

void Frontier_Navigation::escapeStrategy(strategies strategy) {
    if (strategy_ == NORMAL) {
        strategy_ = strategy;
        switch (strategy_) {
        case NO_FRONTIER_REGIONS_FOUND: {
            printf("NO_FRONTIER_REGIONS_FOUND strategy initiated...\n");
            printf("Going through white listed frontierRegions...\n");
            bool found = false;
            for (int i = whiteList_.size()-1; i >= 0; i--) {
                if (frontierConstraints(whiteList_[i], true)) {
                    publishGoal(goals_[i]);
                    found = true;
                    break;
                }
            }
            if (found) printf("\tNew goal found\n");
            else printf("\tNew goal NOT found\n");
            break;
        }
        case STUCK: {
            printf("STUCK strategy initiated...\n");
            // - reached goal but did not receive map update
            //   -> go through whitelist
            // - setting same goal multiple times. happens when robot is within goal area and map update doesn't
            //   change anything
            //   -> solved by DUPLICATED_GOAL
            // - zero vel_cmd commands
            break;
        }
        case GOAL_REJECTED: {
            printf("GOAL_REJECTED strategy initiated...\n");
            // - caused by path going through u-Space
            //   -> blacklist last goal
            break;
        }
        case DUPLICATED_GOAL: {
            printf("DUPLICATED_GOAL strategy initiated...\n");
            // blacklist last goal
            // remove last goal from goalTracker
            break;
        }
        default: printf("Strategy not implemented yet...\n");
        }
        strategy_ = NORMAL;
    } else printf("Strategy rejected. Another strategy already running...\n");
}
