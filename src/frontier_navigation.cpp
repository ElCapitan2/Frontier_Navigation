#include "frontier_navigation.h"
#include "tf/transform_listener.h"
#include "neighbours.h"
#include <vector>
#include <stdlib.h>
#include "helpers.h"

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

    this->pathCounter_ = 0;
    this->pathTracker_.header.frame_id = "/map";
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
    ROS_WARN("\"not_moving_timer\" fired. Robot is likely to be stuck!!");
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

    nav_msgs::GridCells circle = Helpers::circleArea(this->robot_position_.pose.position, 1.0, this->map_);
    int8_t unknown = 100;
    for (int i = 0; i < circle.cells.size(); i++) {
        if (this->map_->data[Helpers::pointToGrid(circle.cells[i], this->map_)] == unknown) printf("STUCK??\n");
    }
    if (Helpers::distance(this->activeGoal_, this->robot_position_) <= 1.0) printf("GOAL REACHED??\n");
    if (this->goalStatus_.status == actionlib_msgs::GoalStatus::REJECTED) printf("BAD GOAL??\n");

//    geometry_msgs::PoseStamped goal;
//    goal.header.frame_id = "/map";
//    goal.pose.position.x = this->robot_position_.x+2;
//    goal.pose.position.y = this->robot_position_.y;
//    this->goal_pub_.publish(goal);
//    system("rostopic pub -1 cmd_vel geometry_msgs/Twist  '{linear:  {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'");
//    system("rostopic pub -1 cmd_vel geometry_msgs/Twist  '{linear:  {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'");
//    system("rostopic pub -1 cmd_vel geometry_msgs/Twist  '{linear:  {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'");
//    system("rostopic pub -1 cmd_vel geometry_msgs/Twist  '{linear:  {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'");
//    system("rostopic pub -1 cmd_vel geometry_msgs/Twist  '{linear:  {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'");
}

void Frontier_Navigation::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr&  map) {
    printf("\n");
    ROS_INFO("Frontier_Navigation received map");
    printf("width: %d; height: %d; res: %f; x_org: %f; y_org: %f\n", map->info.width, map->info.height, map->info.resolution, map->info.origin.position.x, map->info.origin.position.y);
    this->map_ = map;
    processMap();
}

void Frontier_Navigation::posCallback(const geometry_msgs::PoseStamped& robot_position) {
    this->robot_position_ = robot_position;
    this->pathCounter_++;
    if (pathCounter_%10 == 0) {
        this->pathTracker_.cells.push_back(robot_position.pose.position);
        this->pathTracker_pub_.publish(this->pathTracker_);
    }
//    if (Helpers::distance(robot_position, this->activeGoal_) < 3.0) {
//        ROS_WARN("Robot reached goal area without sending new map data.");
//        processMap();
//    }
}

void Frontier_Navigation::cmdVelCallback(const geometry_msgs::Twist& cmd_vel) {
    this->not_moving_timer_ = this->nodeHandle_->createTimer(ros::Duration(this->timeout_), &Frontier_Navigation::timerCallback, this, true);
    cmdVelConstraints(cmd_vel);
}

void Frontier_Navigation::goalStatusCallback(const actionlib_msgs::GoalStatus& goalStatus) {
    if (goalStatus.status == actionlib_msgs::GoalStatus::REJECTED) ROS_WARN("Goal in state REJECTED");
    this->goalStatus_ = goalStatus;
}


void Frontier_Navigation::processMap() {
    vec_double frontiers;
    vec_double adjacencyMatrixOfFrontiers;
    double radius = this->radius_;
    for (int i = 1; i <= this->attempts_; i++) {
        publishOutlineOfSearchRectangle(radius);
        // 1. find frontiers and prepare them for further processing
        findAndPrepareFrontiersWithinRadius(radius, frontiers, adjacencyMatrixOfFrontiers);
        if (frontiers.size() == 0) {
            radius += this->stepping_;
            printf("No frontiers detected within given radius\n");
            printf("Search radius increased (old: %f, new: %f, stepping: %f)\n", radius-this->stepping_, radius, this->stepping_);
            continue;
        }
        // 2. find best suitable frontier
        bool found = false;
        std::vector<int> frontierIDs = determineBestFrontier(adjacencyMatrixOfFrontiers, frontiers);
        publishFrontierPts(frontiers);
        printf("Frontier selection...\n");
        for (int j = 0; j < frontierIDs.size(); j++) {
//            publishFrontierPts(frontiers, frontierIDs[j]);
            if (frontierConstraints(frontiers[frontierIDs[j]], true)) {
                publishFrontierPts(frontiers, frontierIDs[j]);
                geometry_msgs::PoseStamped goal = nextGoal(frontiers[frontierIDs[j]]);
                this->activeGoal_ = goal;
                printf("\tfrontier %d of %d size: %d - Constraints passed!\n", frontierIDs[j], frontiers.size(), frontiers[frontierIDs[j]].size());
                printf("\tNext Goal! goal(%f, %f, %f)\n", this->activeGoal_.pose.position.x, this->activeGoal_.pose.position.y, this->activeGoal_.pose.position.z);
//                publishGoal(goal);
                found = true;
                break;
            } else {
                printf("\tfrontier %d of %d size: %d - Constraints NOT passed!\n", frontierIDs[j], frontiers.size(), frontiers[frontierIDs[j]].size());
            }
        }

        if (found) break;
        else if (i == this->attempts_) {
            // 3.b no valid connected frontiers found
            // - look up saved but not used connected frontiers
            // - find connected frontiers including the whole map
            printf("\tNo suitable frontiers found within maximum radius. STRATEGY NEEDED!!!\n");
        } else {
            // 3.c increase radius and do it again
            radius += this->stepping_;
            printf("\tNo frontier passed constraints within current radius\n");
            printf("\tSearch radius increased (old: %f, new: %f, stepping: %f)\n", radius-this->stepping_, radius, this->stepping_);
            frontiers.clear();
            adjacencyMatrixOfFrontiers.clear();
        }
    }
}

// Define constraints which are necassary for further processing of found connected frontiers
// I.e. set minimum amount of points in set of connected frontiers
bool Frontier_Navigation::frontierConstraints(vec_single &frontier, bool print) {
    bool frontierSize (frontier.size() > this->threshold_);
    bool goalStatus = (this->goalStatus_.status != actionlib_msgs::GoalStatus::REJECTED);
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
    unsigned int robotPos = Helpers::pointToGrid(this->robot_position_.pose.position, this->map_);
    geometry_msgs::Point close = Helpers::gridToPoint(Helpers::closestPoint(frontier, robotPos, this->map_->info.width), this->map_);
    geometry_msgs::Point far = Helpers::gridToPoint(Helpers::furthermostPoint(frontier, robotPos, this->map_->info.width), this->map_);
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
    int goalIndex = Helpers::closestPoint(frontier, Helpers::pointToGrid(this->robot_position_.pose.position, this->map_), this->map_->info.width);
    point.pose.position = Helpers::gridToPoint(goalIndex, this->map_);

    // get vector between robot and goal
    geometry_msgs::Vector3 robotToGoalVec;
    robotToGoalVec.x = point.pose.position.x - this->robot_position_.pose.position.x;
    robotToGoalVec.y = point.pose.position.y - this->robot_position_.pose.position.y;
    robotToGoalVec.z = point.pose.position.z - this->robot_position_.pose.position.z;
    double length = Helpers::length(robotToGoalVec);
//    printf("robotToGoalVec(%f/%f/%f)\n", robotToGoalVec.x, robotToGoalVec.y, robotToGoalVec.z);
//    printf("length of robotToGoalVec: %f\n", length);

    nav_msgs::GridCells circle = Helpers::circle(goalIndex, 1.5, this->map_);
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
