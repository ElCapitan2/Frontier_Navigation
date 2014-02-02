#include <limits>
#include <algorithm>

#include "frontier_navigation.h"

double qualityOfConnectivity(int edges, int frontierPoints) {
    if (frontierPoints == 1) return 0.0;
    double ratio = double(edges)/double(frontierPoints*2-2);
    return Helpers::linearInterpolation(1.0, 10.0, 2.0, 0.0, ratio);
}

double qualityOfSize(int smallestFrontierSize, int largestFrontierSize, int frontierSize) {
    return Helpers::linearInterpolation(smallestFrontierSize, 0.0, largestFrontierSize, 10.0, frontierSize);
}

double qualityOfDistance(double smallestDistance, double largestDistance, double distance) {
    return Helpers::linearInterpolation(smallestDistance, 10.0, largestDistance, 0.0, distance);
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

bool compare(std::vector<double> vec1, std::vector<double> vec2) {return (vec1[0] > vec2[0]);}

std::vector<int> Frontier_Navigation::determineBestFrontierRegions(vec_double &adjacencyMatrixOfFrontiers, vec_double &frontiers) {
    std::vector<double> qualities = computeQualityOfFrontiers(adjacencyMatrixOfFrontiers, frontiers);
    std::vector<std::vector<double> > sortedQualities;
    for (double i = 0; i < qualities.size(); i++) {
        std::vector<double> quality;
        quality.push_back(qualities[i]);
        quality.push_back(i);
        sortedQualities.push_back(quality);
        quality.clear();
    }
    std::sort(sortedQualities.begin(), sortedQualities.end(), compare);
    std::vector<int> sortedFrontierIDs;
    for (int i = 0; i < qualities.size(); i++) {
        sortedFrontierIDs.push_back(static_cast<int>(sortedQualities[i][1]));
    }
    return sortedFrontierIDs;
}

std::vector<double> Frontier_Navigation::computeQualityOfFrontiers(vec_double &adjacencyMatrixOfFrontiers, vec_double &frontiers) {
    // load and prepare data which will be needed for quality measure
    printf("Processing of frontier qualitites ...\n");
    int frontiersCnt = frontiers.size();
    int smallestFrontierSize = std::numeric_limits<int>::max();
    int largestFrontierSize = 0;
    double smallestDistance = std::numeric_limits<double>::max();
    double largestDistance = 0.0;
    int edgesCntOfFrontiers[frontiersCnt];
    double distancesToFrontiers[frontiersCnt];
    geometry_msgs::Point goalsInFrontiers[frontiersCnt];
    int robotPos = mapOps_.pointToCell(this->robot_position_.pose.position, this->map_);

    for (int i = 0; i < frontiersCnt; i++) {
        if (frontiers[i].size() < smallestFrontierSize) smallestFrontierSize = frontiers[i].size();
        if (frontiers[i].size() > largestFrontierSize) largestFrontierSize = frontiers[i].size();
        int closestPoint = Helpers::closestPoint(frontiers[i], robotPos, this->map_->info.width);
        distancesToFrontiers[i] = (Helpers::distance(closestPoint, robotPos, this->map_->info.width, this->map_->info.resolution));
        if (distancesToFrontiers[i] < smallestDistance) smallestDistance = distancesToFrontiers[i];
        if (distancesToFrontiers[i] > largestDistance) largestDistance = distancesToFrontiers[i];
        goalsInFrontiers[i] = mapOps_.cellToPoint(closestPoint, this->map_);
        // init to prevent fraud data
        edgesCntOfFrontiers[i] = 0;
    }
    // make sure that small frontiers will receive small qualities
//    if (smallestFrontierSize < this->threshold_) smallestFrontierSize = this->threshold_;
    for (unsigned int i = 0; i < adjacencyMatrixOfFrontiers.size(); i++) {
        edgesCntOfFrontiers[adjacencyMatrixOfFrontiers[i][0]-1]+=adjacencyMatrixOfFrontiers[i].size()-2;
    }
    printf("\tfrontiers: %d\n", frontiersCnt);
    printf("\tsmallest frontier size: %d; largest frontier size: %d\n", smallestFrontierSize, largestFrontierSize);
    printf("\tsmallest dist to frontiers: %f; largest dist to frontiers: %f\n", smallestDistance, largestDistance);
    printf("\tweight of: connectivity: %f; size: %f; distance: %f; direction: %f\n", this->weightOfConnectivity_, this->weightOfSize_, this->weightOfDistance_, this->weightOfDirection_);
//    printf("Quality measure:\n");
    // actual quality measure of each frontier
    std::vector<double> qualities;
    for (int i = 0; i < frontiersCnt; i++) {
        double qualOfConnectivity = qualityOfConnectivity(edgesCntOfFrontiers[i], frontiers[i].size());
        double qualOfSize = qualityOfSize(smallestFrontierSize, largestFrontierSize, frontiers[i].size());
        double qualOfDistance = qualityOfDistance(smallestDistance, largestDistance, distancesToFrontiers[i]);
        double qualOfDirection = qualityOfDirection(this->robot_position_, goalsInFrontiers[i]);
        double quality = this->weightOfConnectivity_*qualOfConnectivity + this->weightOfSize_*qualOfSize + this->weightOfDistance_*qualOfDistance + this->weightOfDirection_*qualOfDirection;
        qualities.push_back(quality);
//        printf("%d  size: %d\tqual: %f\tconn: %f\tsize: %f\tdist: %f\tdir: %f\n", i, frontiers[i].size(), quality, qualOfConnectivity, qualOfSize, qualOfDistance, qualOfDirection);
    }
    printf("\tProcessing of frontier qualitites finished!\n");
    return qualities;
}


