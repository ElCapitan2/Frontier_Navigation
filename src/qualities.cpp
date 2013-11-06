#include "frontier_navigation.h"
#include "helpers.h"
#include <limits>

double qualityOfConnectivity(int edges, int frontierPoints) {
    if (frontierPoints == 1) return 0.0;
    double ratio = double(edges)/double(frontierPoints*2-2);
    return Helpers::linearInterpolation(1.0, 10.0, 2.0, 0.0, ratio);
}

double qualityOfSize(int totalPoints, int frontierPoints) {
    return Helpers::linearInterpolation(1.0, 0.0, totalPoints, 10.0, frontierPoints);
}

double qualityOfDistance(double distance, double totalDistance) {
    return Helpers::linearInterpolation(1.0, 10.0, totalDistance, 0.0, distance);
}

double qualityOfDirection(geometry_msgs::PoseStamped robot_position, geometry_msgs::Point goal) {

    double yaw = tf::getYaw(robot_position.pose.orientation);
    double xDirOfRobot = cos(yaw);
    double yDirOfRobot = sin(yaw);
    double xDirToGoal = goal.x - robot_position.pose.position.x;
    double yDirToGoal = goal.y - robot_position.pose.position.y;
    double angle = acos((xDirOfRobot*xDirToGoal + yDirOfRobot*yDirToGoal)/sqrt(pow(xDirToGoal, 2) + pow(yDirToGoal, 2)));
//    printf("pos_x: %f pos_y: %f dir_x: %f dir_y: %f yaw: %f vec_x: %f vec_y: %f angle: %f\n", robot_position.pose.position.x, robot_position.pose.position.y, x_1, y_1, yaw, x_2, y_2, angle);
    return Helpers::linearInterpolation(0, 10, M_PI, 0, angle);
}

int Frontier_Navigation::determineBestFrontier(vec_double &adjacencyMatrixOfFrontiers, vec_double &frontiers) {
    std::vector<double> qualities = computeQualityOfFrontiers(adjacencyMatrixOfFrontiers, frontiers);
    double max = 0.0;
    int idx = 0;
    for (int i = 0; i < qualities.size(); i++) {
        if (qualities[i] > max) {
            max = qualities[i];
            idx = i;
        }
    }
    return idx;
}

std::vector<double> Frontier_Navigation::computeQualityOfFrontiers(vec_double &adjacencyMatrixOfFrontiers, vec_double &frontiers) {
    // load and prepare data which will be needed for quality measure
    int frontiersCnt = frontiers.size();
    int points = 0;
    int edges[frontiersCnt];
    geometry_msgs::Point goals[frontiersCnt];
    int robot_pos = Helpers::pointToGrid(this->robot_position_.pose.position, this->map_);
    double distances[frontiersCnt];
    double totalDistance = 0.0;
    for (int i = 0; i < frontiersCnt; i++) {
        points += frontiers[i].size();
        edges[i] = 0;
        int closestPoint = Helpers::closestPoint(frontiers[i], robot_pos, this->map_->info.width);
        distances[i] = (Helpers::distance(closestPoint, robot_pos, this->map_->info.width, this->map_->info.resolution));
        totalDistance += distances[i];
        goals[i] = Helpers::gridToPoint(closestPoint, this->map_);
    }
    for (unsigned int i = 0; i < adjacencyMatrixOfFrontiers.size(); i++) {
        edges[adjacencyMatrixOfFrontiers[i][0]-1]+=adjacencyMatrixOfFrontiers[i].size()-2;
    }
    // actual quality measure of each frontier
    std::vector<double> qualities;
    for (int i = 0; i < frontiersCnt; i++) {
        double qualOfConnectivity = qualityOfConnectivity(edges[i], frontiers[i].size());
        double qualOfSize = qualityOfSize(points, frontiers[i].size());
        double qualOfDistance = qualityOfDistance(distances[i], totalDistance);
        double qualOfDirection = qualityOfDirection(this->robot_position_, goals[i]);
        double quality = this->weightOfConnectivity_*qualOfConnectivity + this->weightOfSize_*qualOfSize + this->weightOfDistance_*qualOfDistance + this->weightOfDirection_*qualOfDirection;
        qualities.push_back(quality);
        printf("frontier: %d\telements: %d\tqual: %f\tqualOf-\tconn: %f\tsize: %f\tdist: %f\tdir: %f\n", i, frontiers[i].size(), quality, qualOfConnectivity, qualOfSize, qualOfDistance, qualOfDirection);
    }
    return qualities;
}


