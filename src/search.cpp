#include "frontier_navigation.h"
#include "helpers.h"
#include "neighbours.h"
#include "types.h"

// free-space = 0
// occupied-space = 100
// unknown-space == -1

void Frontier_Navigation::findAndPrepareFrontiersWithinRadius(int radius, vec_double &frontiers, vec_double &adjacencyMatrixOfFrontiers) {
    // 1. find raw frontier indices themselves
    std::vector<unsigned int> frontierIdxs = findFrontierIdxsWithinRadius(radius);
    // 2. compute adjacency matrix out of found frontier indices
    adjacencyMatrixOfFrontiers = computeAdjacencyMatrixOfFrontiers(frontierIdxs);
    // 3. find frontiers out of computed adjacency matrix
    frontiers = findFrontiers(adjacencyMatrixOfFrontiers);
}

std::vector<unsigned int> Frontier_Navigation::findFrontierIdxsWithinRadius(int radius) {
    // Based on robots coordinate system
    // Make sure not to leave given coordinate system!

    geometry_msgs::Point pos = this->robot_position_.pose.position;
    geometry_msgs::Point startPoint;
    startPoint.x = pos.x - radius;
    startPoint.y = pos.y - radius;
    startPoint.z = 0;

    int startIndex = Helpers::pointToGrid(startPoint, this->map_);

    Neighbours neighbours(map_->info.width, map_->info.height);
    int iterations = radius*2/map_->info.resolution;

    std::vector<unsigned int> frontierIdxs;
    nav_msgs::GridCells zeros;
    nav_msgs::GridCells min1;
    nav_msgs::GridCells min2;
    nav_msgs::GridCells min3;
    nav_msgs::GridCells min4;
    int8_t data = 0;
    int index;
    for (int i = 0; i < iterations; i++) {
        for (int j = 0; j < iterations; j++) {
            index = startIndex + j + i*map_->info.height;

            data = map_->data[index];
            // if free-space is next to unknown-space free-space-index will be added
            // to indexedRawFrontiers
            if (data == 0) {
                if (neighbours.getValLeft(index, map_) == -1) {
                    frontierIdxs.push_back(index);
                }
                else if (neighbours.getValRight(index, map_) == -1) {
                    frontierIdxs.push_back(index);
                }
                else if (neighbours.getValTop(index, map_) == -1) {
                    frontierIdxs.push_back(index);
                }
                else if (neighbours.getValBottom(index, map_) == -1) {
                    frontierIdxs.push_back(index);
                }
            }
            // try to skip faulty frontiers
            if (data == -1) {
                int result = neighbours.getValLeft(index, map_) + neighbours.getValRight(index, map_) + neighbours.getValTop(index, map_) + neighbours.getValBottom(index, map_);
                if (result == 0) zeros.cells.push_back(Helpers::gridToPoint(index, map_));
                else if (result == -1) min1.cells.push_back(Helpers::gridToPoint(index, map_));
                else if (result == -2) min2.cells.push_back(Helpers::gridToPoint(index, map_));
                else if (result == -3) min3.cells.push_back(Helpers::gridToPoint(index, map_));
                else if (result == -4) min4.cells.push_back(Helpers::gridToPoint(index, map_));
                else printf("%d\n", result);
            }            
        }
    }
    printf("zeros: %d\n", zeros.cells.size());
    printf("min1: %d\n", min1.cells.size());
    printf("min2: %d\n", min2.cells.size());
    printf("min3: %d\n", min3.cells.size());
    printf("min4: %d\n", min4.cells.size());
    zeros.cell_height = zeros.cell_width = map_->info.resolution;
    min1.cell_height = min1.cell_width = map_->info.resolution;
    min2.cell_height = min2.cell_width = map_->info.resolution;
    min3.cell_height = min3.cell_width = map_->info.resolution;
    min4.cell_height = min4.cell_width = map_->info.resolution;
    zeros.header.frame_id = min1.header.frame_id = min2.header.frame_id = min3.header.frame_id = min4.header.frame_id = "/map";
    this->zeros_pub_.publish(zeros);
    this->min1_pub_.publish(min1);
    this->min2_pub_.publish(min2);
    this->min3_pub_.publish(min3);
    this->min4_pub_.publish(min4);
    return frontierIdxs;
}

vec_double Frontier_Navigation::computeAdjacencyMatrixOfFrontiers(std::vector<unsigned int> &frontierIdxs) {

    // frontierIdxs has to be sorted!!!!
    // i.e.:
    // frontierIdxs = {12, 25, 26, 31}
    // right(25) = 26
    // => frontierIdxs[i] = frontierIdxs[i+1]-1 => right(frontierIdxs[i]) = frontierIdxs[i+1]
    // therefore we can easily check left and right neighbours by just looking up next/previous index in frontierIdxs

    std::vector<std::vector<unsigned int> > adjacencyMatrixOfFrontiers;
    std::vector<unsigned int> neighbours;
    int width = this->map_->info.width;

    for (unsigned int position = 0; position < frontierIdxs.size(); position++) {
        int frontierIndex = frontierIdxs[position];
        neighbours.push_back(0);
        neighbours.push_back(frontierIndex);
        // bottom
        for (unsigned int i = 0; i < position; i++) {
            int currentElement = frontierIdxs[i];
            if (currentElement == frontierIndex-width-1) neighbours.push_back(currentElement);
            else if (currentElement == frontierIndex-width) neighbours.push_back(currentElement);
            else if (currentElement == frontierIndex-width+1) {neighbours.push_back(currentElement); break;}
        }
        // left
        if (frontierIdxs[position-1] == frontierIndex-1) neighbours.push_back(frontierIdxs[position-1]);
        // right
        if (frontierIdxs[position+1] == frontierIndex+1) neighbours.push_back(frontierIdxs[position+1]);
        // top
        for (unsigned int i = position+1; i < frontierIdxs.size(); i++) {
            int currentElement = frontierIdxs[i];
            if (currentElement == frontierIndex+width-1) neighbours.push_back(currentElement);
            else if (currentElement == frontierIndex+width) neighbours.push_back(currentElement);
            else if (currentElement == frontierIndex+width+1) {neighbours.push_back(currentElement); break;}
        }
        adjacencyMatrixOfFrontiers.push_back(neighbours);
        neighbours.clear();
    }
    return adjacencyMatrixOfFrontiers;
}

vec_double Frontier_Navigation::findFrontiers(vec_double &adjacencyMatrixOfFrontiers) {
    vec_double frontiers;
    std::vector<unsigned int> neighbours;
    int component = 1;
    for (unsigned int i = 0; i < adjacencyMatrixOfFrontiers.size(); i++) {
        if (adjacencyMatrixOfFrontiers[i][0] == 0) {
            recursivelyFindFrontiers(adjacencyMatrixOfFrontiers, neighbours, i, component);
            frontiers.push_back(neighbours);
            neighbours.clear();
            component++;
        }
    }
    return frontiers;
}

void Frontier_Navigation::recursivelyFindFrontiers(vec_double &adjacencyMatrixOfFrontiers, std::vector<unsigned int> &neighbours, int index, int component) {
    // point not yet visited
    if (adjacencyMatrixOfFrontiers[index][0] == 0) {
        adjacencyMatrixOfFrontiers[index][0] = component;
        neighbours.push_back(adjacencyMatrixOfFrontiers[index][1]);
        for (unsigned int i = 2; i < adjacencyMatrixOfFrontiers[index].size(); i++) {
            int nextIndex = adjacencyMatrixOfFrontiers[index][i];
            for (unsigned int j = 0; j < adjacencyMatrixOfFrontiers.size(); j++) {
                if (adjacencyMatrixOfFrontiers[j][1] == nextIndex) {
                    recursivelyFindFrontiers(adjacencyMatrixOfFrontiers, neighbours, j, component);
                }
            }
        }
    }
}
