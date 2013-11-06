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

void Frontier_Navigation::circle1(double radius) {
    nav_msgs::GridCells circle;
    int index = Helpers::pointToGrid(this->robot_position_.pose.position, this->map_);
    double boxes = radius/0.05;
    int cnt = 0;
    for (int row = index-boxes*4000; row < index+boxes*4000; row+=4000) {
        for (int i = row-boxes; i < row+boxes; i++) {
            cnt++;
            if (Helpers::distance(index, i, 4000, 0.05) <= radius) {
                circle.cells.push_back(Helpers::gridToPoint(i, 4000, 4000, 0.05));
            }
        }
    }
    printf("circle1 - radius: %f, size: %d, cycles: %d\n", radius, circle.cells.size(), cnt);
    circle.header.frame_id = "/map";
    circle.cell_height = circle.cell_width = 0.05;
    frontiers_pub_.publish(circle);
}

void Frontier_Navigation::circle2(double radius) {
    nav_msgs::GridCells circle;
    int index = Helpers::pointToGrid(this->robot_position_.pose.position, this->map_);
    double boxes = radius/0.05;
    int cnt = 0;
    for (int row = index-boxes*4000; row < index+boxes*4000; row+=4000) {
        for (int i = row; i < row+boxes; i++) {
            cnt++;
            if (Helpers::distance(index, i, 4000, 0.05) <= radius) {
                geometry_msgs::Point pt1 = Helpers::gridToPoint(i, 4000, 4000, 0.05);
                geometry_msgs::Point pt2;
                pt2.x = pt1.x - 2*(i-row)*0.05;
                pt2.y = pt1.y;
                circle.cells.push_back(pt1);
                circle.cells.push_back(pt2);
            } else {break;}
        }
    }
    printf("circle2 - radius: %f, size: %d, cycles: %d\n", radius, circle.cells.size(), cnt);
    circle.header.frame_id = "/map";
    circle.cell_height = circle.cell_width = 0.05;
    frontiers_pub_.publish(circle);
}

void Frontier_Navigation::circle3(double radius) {
    nav_msgs::GridCells circle;
    int index = Helpers::pointToGrid(this->robot_position_.pose.position, this->map_);
    double boxes = radius/0.05;
    int cnt = 0;
    for (int row = index-boxes*4000; row < index+boxes*4000; row+=4000) {
        for (int i = row; i < row+boxes; i++) {
            cnt++;
            if (Helpers::distance(index, i, 4000, 0.05) <= radius) {
                circle.cells.push_back(Helpers::gridToPoint(i, 4000, 4000, 0.05));
            } else {break;}
        }
        for (int i = row; i > row-boxes; i--) {
            cnt++;
            if (Helpers::distance(index, i, 4000, 0.05) <= radius) {
                circle.cells.push_back(Helpers::gridToPoint(i, 4000, 4000, 0.05));
            } else {break;}
        }
    }
//    printf("circle3 - radius: %f, size: %d, cycles: %d\n", radius, circle.cells.size(), cnt);
    circle.header.frame_id = "/map";
    circle.cell_height = circle.cell_width = 0.05;
    circle_pub_.publish(circle);
}

void Frontier_Navigation::circle4(double radius, geometry_msgs::Point pt) {
    nav_msgs::GridCells circle;
    int index = Helpers::pointToGrid(pt, this->map_);
    double boxes = radius/0.05;
    int cnt = 0;
    for (int row = 0; row < boxes; row++) {
        for (int i = index+row*4000; i < index+row*4000+boxes; i++) {
            cnt++;
            if (Helpers::distance(index, i, 4000, 0.05) <= radius) {
                geometry_msgs::Point pt1 = Helpers::gridToPoint(i, 4000, 4000, 0.05);
                circle.cells.push_back(pt1);
                geometry_msgs::Point pt2;
                pt2.x = pt1.x;
                pt2.y = pt1.y - 2*(row)*0.05;
                circle.cells.push_back((pt2));
                pt2.x = pt1.x - 2*(i-(index+row*4000))*0.05;
                pt2.y = pt1.y;
                circle.cells.push_back(pt2);
                pt2.y = pt1.y - 2*(row)*0.05;
                circle.cells.push_back(pt2);
            } else {break;}
        }
    }
    circle.header.frame_id = "/map";
    circle.cell_height = circle.cell_width = 0.05;
    circle_pub_.publish(circle);
}


void Frontier_Navigation::timerCallback(const ros::TimerEvent&) {
    ROS_WARN("Robot not moving! Stuff will be done to robot in order to get it moving again...");
//    geometry_msgs::PoseStamped goal;
//    goal.header.frame_id = "/map";
//    goal.pose.position.x = this->robot_position_.x+2;
//    goal.pose.position.y = this->robot_position_.y;
//    this->goal_pub_.publish(goal);
    system("rostopic pub -1 cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 45.0}}'");
    system("rostopic pub -1 cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 45.0}}'");
    system("rostopic pub -1 cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 45.0}}'");
    system("rostopic pub -1 cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 45.0}}'");
    system("rostopic pub -1 cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 45.0}}'");
}

void Frontier_Navigation::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr&  map) {

//    ROS_INFO("Frontier_Navigation received map");
//    this->not_moving_timer_ = this->nodeHandle_->createTimer(ros::Duration(timeout_), &Frontier_Navigation::timerCallback, this, true);
//    ROS_INFO("Timer started to prevent dead locks");

    this->map_ = map;

    vec_double frontiers;
    vec_double adjacencyMatrixOfFrontiers;
    double radius = this->radius_;
    for (int i = 1; i <= this->attempts_; i++) {
        // 1. find frontiers and prepare them for further processing
        findAndPrepareFrontiersWithinRadius(radius, frontiers, adjacencyMatrixOfFrontiers);
        if (frontiers.size() == 0) {
            radius += this->stepping_;
            printf("No frontiers detected within given radius\n");
            continue;
        }
        // 2. find best suitable frontier
        int frontier = determineBestFrontier(adjacencyMatrixOfFrontiers, frontiers);
        // 3. run some more validations in order to really use an appropriate frontier
        if (frontierConstraints(frontiers[frontier])) {
            publishOutlineOfSearchRectangle(radius);
            publishFrontierPts(frontiers);
            publishFrontierPts(frontiers, frontier);
            geometry_msgs::PoseStamped goal = nextGoal(frontiers[frontier]);
            publishGoal(goal);
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
            publishOutlineOfSearchRectangle(radius);
            frontiers.clear();
            adjacencyMatrixOfFrontiers.clear();
        }
    }
}

void Frontier_Navigation::posCallback(const geometry_msgs::PoseStamped& robot_position) {
    this->robot_position_ = robot_position;
}

// Define constraints which are necassary for further processing of found connected frontiers
// I.e. set minimum amount of points in set of connected frontiers
bool Frontier_Navigation::frontierConstraints(vec_single &frontier) {
    return frontier.size() > this->threshold_;
//    return true;
}

geometry_msgs::PoseStamped Frontier_Navigation::nextGoal(vec_single frontier)
{
    geometry_msgs::PoseStamped point;
    int goalIndex = Helpers::closestPoint(frontier, Helpers::pointToGrid(this->robot_position_.pose.position, this->map_), this->map_->info.width);
    point.pose.position = Helpers::gridToPoint(goalIndex, this->map_);
    printf("Next Goal! (%f, %f)\n", point.pose.position.x, point.pose.position.y);
    double deltaX = point.pose.position.x - this->robot_position_.pose.position.x;
    double deltaY = point.pose.position.y - this->robot_position_.pose.position.y;
    point.pose.position.x = this->robot_position_.pose.position.x + 0.75 * deltaX;
    point.pose.position.y = this->robot_position_.pose.position.y + 0.75 * deltaY;
    circle4(1.0, point.pose.position);
    printf("Next Goal! delta(%f, %f), goal(%f, %f)\n", deltaX, deltaY, point.pose.position.x, point.pose.position.y);
    return point;
}

void Frontier_Navigation::publishFrontierPts(vec_double frontiers) {
    nav_msgs::GridCells frontierPts;
    for (int i = 0; i < frontiers.size(); i++) {
        for (int j = 0; j < frontiers[i].size(); j++) {
            frontierPts.cells.push_back(Helpers::gridToPoint(frontiers[i][j], this->map_));
        }
    }
    frontierPts.cell_height = frontierPts.cell_width = this->map_->info.resolution;
    frontierPts.header.frame_id = "/map";
    this->frontiers_pub_.publish(frontierPts);
}

void Frontier_Navigation::publishFrontierPts(vec_double frontiers, int best_frontier) {
    nav_msgs::GridCells frontierPts;
    for (int i = 0; i < frontiers[best_frontier].size(); i++) {
        frontierPts.cells.push_back(Helpers::gridToPoint(frontiers[best_frontier][i], this->map_));
    }
    frontierPts.cell_height = frontierPts.cell_width = this->map_->info.resolution;
    frontierPts.header.frame_id = "/map";
    this->bestFrontier_pub_.publish(frontierPts);
}

void Frontier_Navigation::publishGoal(geometry_msgs::PoseStamped goal) {
    goal.header.frame_id = "/map";
    this->goal_pub_.publish(goal);
}

void Frontier_Navigation::publishOutlineOfSearchRectangle(int radius) {
    nav_msgs::GridCells rectangle;
    geometry_msgs::Point pos = this->robot_position_.pose.position;
    geometry_msgs::Point startPoint;
    startPoint.x = pos.x - radius;
    startPoint.y = pos.y - radius;
    startPoint.z = 0;

    int startIndex = Helpers::pointToGrid(startPoint, this->map_);
    int iterations = radius*2/map_->info.resolution;
    for (int i = 0; i < iterations; i++) {
        rectangle.cells.push_back(Helpers::gridToPoint(startIndex + i, map_));
        rectangle.cells.push_back(Helpers::gridToPoint(startIndex+iterations*map_->info.height + i, map_));
        rectangle.cells.push_back(Helpers::gridToPoint(startIndex + i*map_->info.height, map_));
        rectangle.cells.push_back(Helpers::gridToPoint(startIndex + i*map_->info.height + iterations, map_));
    }
    rectangle.cell_height = rectangle.cell_width = this->map_->info.resolution;
    rectangle.header.frame_id = "/map";
    this->rectangle_pub_.publish(rectangle);
}




double Frontier_Navigation::distanceToSetOfFrontiers(std::vector<geometry_msgs::Point> &frontierSet) {
    // simple approach to find distance form set of frontiers to robot
    // more advanced approaches might be necessaray and useful
    double dist = 0.0;
    for (unsigned int i = 0; i < frontierSet.size(); i++) {
        dist += Helpers::distance(robot_position_.pose.position, frontierSet[i]);
    }
    return dist / frontierSet.size();
}

double Frontier_Navigation::distanceToSetOfFrontiers(std::vector<unsigned int> &frontierSet) {
    return 0.0;
}


// Define constraints for every single found frontier point
// I.e. skip frontiers which are surounded by free-space (due to sensor resolution)
bool Frontier_Navigation::validateFrontierPoint(int index) {
    return true;
}
