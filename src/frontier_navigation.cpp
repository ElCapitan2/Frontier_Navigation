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

    this->pathCounter_ = 0;
    this->pathTracker_.header.frame_id = "/map";
    this->pathTracker_.cell_height = this->pathTracker_.cell_width = 0.05;

    node_ptr->param("/frontier_navigation/radius", radius_, 5.0);
    node_ptr->param("/frontier_navigation/attempts", attempts_, 4);
    node_ptr->param("/frontier_navigation/stepping", stepping_, 5.0);
    node_ptr->param("/frontier_navigation/threshold", threshold_, 250);
    node_ptr->param("/frontier_navigation/sleep", sleep_, 0);
    node_ptr->param("/frontier_navigation/minDistance", minDinstance_, 3.0);
    node_ptr->param("/frontier_navigation/timeout", timeout_, 5.0);
    node_ptr->param("/frontier_navigation/timeoutAttempts", timeoutAttempts_, 5);
    node_ptr->param("/frontier_navigation/weightOfConnectivity", weightOfConnectivity_, 3.0);
    node_ptr->param("/frontier_navigation/worstCase", worstCaseOfConnectivity_, 2.0);
    node_ptr->param("/frontier_navigation/weightOfSize", weightOfSize_, 2.0);
    node_ptr->param("/frontier_navigation/weightOfDistance", weightOfDistance_, 1.0);
    node_ptr->param("/frontier_navigation/weightOfDirection", weightOfDirection_, 4.0);
}

void Frontier_Navigation::timerCallback(const ros::TimerEvent&) {
    ROS_WARN("\"not_moving_timer\" fired. Robot is likely to be stuck!!");
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
    ROS_INFO("Frontier_Navigation received map");
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
    if (Helpers::distance(robot_position, this->activeGoal_) < 3.0) {
        ROS_WARN("Robot reached goal area without sending new map data.");
        processMap();
    }
}

void Frontier_Navigation::cmdVelCallback(const geometry_msgs::Twist& cmd_vel) {
    this->not_moving_timer_ = this->nodeHandle_->createTimer(ros::Duration(this->timeout_), &Frontier_Navigation::timerCallback, this, true);
}

void Frontier_Navigation::processMap() {
    vec_double frontiers;
    vec_double adjacencyMatrixOfFrontiers;
    double radius = this->radius_;
    for (int i = 1; i <= this->attempts_; i++) {
        // 1. find frontiers and prepare them for further processing
        findAndPrepareFrontiersWithinRadius(radius, frontiers, adjacencyMatrixOfFrontiers);
        if (frontiers.size() == 0) {
            radius += this->stepping_;
            printf("No frontiers detected within given radius\n");
            publishOutlineOfSearchRectangle(radius);
            continue;
        }
        // 2. find best suitable frontier
        int frontier = determineBestFrontier(adjacencyMatrixOfFrontiers, frontiers);
        publishOutlineOfSearchRectangle(radius);
        publishFrontierPts(frontiers);
        publishFrontierPts(frontiers, frontier);
        printf("frontier %d of %d size: %d\n", frontier, frontiers.size(), frontiers[frontier].size());
        // 3. run some more validations in order to really use an appropriate frontier
        if (frontierConstraints(frontiers[frontier])) {
            geometry_msgs::PoseStamped goal = nextGoal(frontiers[frontier]);
            this->activeGoal_ = goal;
//            publishGoal(goal);
            break;
        } else if (i == this->attempts_) {
            // 3.b no valid connected frontiers found
            // - look up saved but not used connected frontiers
            // - find connected frontiers including the whole map
            printf("Not enough frontier points found\n");
        } else {
            // 3.c increase radius and do it again
            radius += this->stepping_;
            printf("Search radius increased (old: %f, new: %f, stepping: %f)\n", radius-this->stepping_, radius, this->stepping_);
//            publishOutlineOfSearchRectangle(radius);
            frontiers.clear();
            adjacencyMatrixOfFrontiers.clear();
        }
    }
}

// Define constraints which are necassary for further processing of found connected frontiers
// I.e. set minimum amount of points in set of connected frontiers
bool Frontier_Navigation::frontierConstraints(vec_single &frontier) {
    return frontier.size() > this->threshold_;
//    return true;
}

geometry_msgs::PoseStamped Frontier_Navigation::nextGoal(vec_single frontier)
{
    unsigned int robotPos = Helpers::pointToGrid(this->robot_position_.pose.position, this->map_);
    geometry_msgs::Point close = Helpers::gridToPoint(Helpers::closestPoint(frontier, robotPos, this->map_->info.width), this->map_);
    geometry_msgs::Point far = Helpers::gridToPoint(Helpers::furthermostPoint(frontier, robotPos, this->map_->info.width), this->map_);

    // get frontiers vector
    geometry_msgs::Vector3 frontierVec;
    frontierVec.x = far.x - close.x;
    frontierVec.y = far.y - close.y;
    frontierVec.z = far.z - close.z;

    geometry_msgs::PoseStamped point;
    int goalIndex = Helpers::closestPoint(frontier, Helpers::pointToGrid(this->robot_position_.pose.position, this->map_), this->map_->info.width);
    point.pose.position = Helpers::gridToPoint(goalIndex, this->map_);

    // get vector between robot and goal
    geometry_msgs::Vector3 robotToGoalVec;
    robotToGoalVec.x = point.pose.position.x - this->robot_position_.pose.position.x;
    robotToGoalVec.y = point.pose.position.y - this->robot_position_.pose.position.y;
    robotToGoalVec.z = point.pose.position.z - this->robot_position_.pose.position.z;
    double length = Helpers::length(robotToGoalVec);

//    point.pose.position.x = this->robot_position_.pose.position.x + deltaX/length * (length-this->minDinstance_);
//    point.pose.position.y = this->robot_position_.pose.position.y + deltaY/length * (length-this->minDinstance_);
//    point.pose.position.z = this->robot_position_.pose.position.z + deltaZ/length * (length-this->minDinstance_);
    nav_msgs::GridCells circle = Helpers::circle(goalIndex, 1.0, this->map_);
    publishCircle(goalIndex);
    geometry_msgs::Vector3 goalToCircleVec;
    bool found = false;
    for (int i = 0; i < circle.cells.size(); i++) {
        goalToCircleVec.x = circle.cells[i].x - point.pose.position.x;
        goalToCircleVec.y = circle.cells[i].y - point.pose.position.y;
        goalToCircleVec.z = circle.cells[i].z - point.pose.position.z;
        double angle = Helpers::angleInDegree(frontierVec, goalToCircleVec);
//        printf("angle: %f\n", angle);
        if (angle >= 78.0 && angle <= 82.00) {
            point.pose.position.x = circle.cells[i].x;
            point.pose.position.y = circle.cells[i].y;
            point.pose.position.z = circle.cells[i].z;

            geometry_msgs::Vector3 x;
            x.x = 1.0;
            x.y = x.z = 0.0;
            geometry_msgs::Vector3 y;

            y.x = far.x - point.pose.position.x;
            y.y = far.y - point.pose.position.y;
            y.z = 0.0;

            double angle2 = Helpers::angleInDegree(y, x);

            if (y.y < 0) angle2 = 360-angle2;
            tf::Quaternion quaternion = tf::createQuaternionFromYaw(angle2*M_PI / 180);
            point.pose.orientation.x = quaternion.getX();
            point.pose.orientation.y = quaternion.getY();
            point.pose.orientation.z = quaternion.getZ();
            point.pose.orientation.w = quaternion.getW();
            found = true;
            break;
        }

    }

    if(!found) printf("Not found!!!!!!!\n");

    printf("Next Goal! goal(%f, %f, %f)\n", point.pose.position.x, point.pose.position.y, point.pose.position.z);
    return point;
}
