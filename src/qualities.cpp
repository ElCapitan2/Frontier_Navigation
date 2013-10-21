#include "frontier_navigation.h"
#include "helpers.h"

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
    int robot_pos = Helpers::pointToGrid(this->robot_position_, this->map_);
    double distances[frontiersCnt];
    double totalDistance = 0.0;
    for (int i = 0; i < frontiersCnt; i++) {
        points += frontiers[i].size();
        edges[i] = 0;
        double distance = 0.0;
        for (int j = 0; j < frontiers[i].size(); j++) {
            distance += Helpers::distance(robot_pos, frontiers[i][j], 4000, 0.05);
        }
        distances[i] = (distance/frontiers[i].size());
        totalDistance += distances[i];
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
        double quality = this->weightOfConnectivity_*qualOfConnectivity + this->weightOfSize_*qualOfSize + this->weightOfDistance_*qualOfDistance;
        qualities.push_back(quality);
        printf("frontier: %d\telements: %d\tquality: %f\tqualOf-\tconnectivity: %f\tsize: %f\tdistance: %f\n", i, frontiers[i].size(), quality, qualOfConnectivity, qualOfSize, qualOfDistance);
    }
    return qualities;
}


