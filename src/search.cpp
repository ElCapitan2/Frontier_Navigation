//#include "frontier_navigation.h"
#include "map_operations.h"

// free-space = 0
// occupied-space = 100
// unknown-space == -1

void MapOperations::findFrontierRegions(boost::shared_ptr<nav_msgs::OccupancyGrid> &map, const geometry_msgs::PoseStamped &center, int radius, vec_double &frontierRegions, vec_double &adjacencyMatrixOfFrontierCells) {
    map_ = map;
    // 1. find raw frontierCells themselves
    printf("Searching for frontier cells...\n");
    std::vector<unsigned int> frontierCells = findFrontierCells(center, radius);
    // 2. compute adjacency matrix out of found frontierCells
    printf("Computing adjacency matrix...\n");
    adjacencyMatrixOfFrontierCells = computeAdjacencyMatrixOfFrontierCells(frontierCells);
    // 3. find frontierRegions out of computed adjacency matrix
    printf("Computing frontierRegions...\n");
    frontierRegions = computeFrontierRegions(adjacencyMatrixOfFrontierCells);
}

std::vector<unsigned int> MapOperations::findFrontierCells(const geometry_msgs::PoseStamped &center, int radius) {

    // setup search area
    int startCell;
    int iterations;
    setupSearchArea(center, radius, map_, startCell, iterations);

    // performance measure
    int compares = 0;
    int neighbourLookUps = 0;
    int fSpaceCells = 0;

//    Neighbours neighbours(map_->info.width, map_->info.height);

    std::vector<unsigned int> frontierCells;
    int8_t data = 0;
    unsigned int index;
    // O(iterations*iterations)
    for (int i = 0; i < iterations; i++) {
        for (int j = 0; j < iterations; j++) {
            index = startCell + j + i*this->map_->info.height;
            data = this->map_->data[index];
            // if free-space is next to unknown-space free-space-index will be added
            // to indexedRawFrontiers
            compares++;
            if (data == F_SPACE) {
                fSpaceCells++;
                if (isUSpace(getLeftVal(index, map_), map_)) {
//                if (neighbours.getValLeft(index, map_) == U_SPACE) {
                    frontierCells.push_back(index);
                    compares++;
                    neighbourLookUps++;
                }
//                else if (neighbours.getValRight(index, map_) == U_SPACE) {
                else if (isUSpace(getRightVal(index, map_), map_)) {
                    frontierCells.push_back(index);
                    compares += 2;
                    neighbourLookUps += 2;
                }
//                else if (neighbours.getValTop(index, map_) == U_SPACE) {
                else if (isUSpace(getTopVal(index, map_), map_)) {
                    frontierCells.push_back(index);
                    compares += 3;
                    neighbourLookUps += 3;
                }
//                else if (neighbours.getValBottom(index, map_) == U_SPACE) {
                else if (isUSpace(getBottomVal(index, map_), map_)) {
                    frontierCells.push_back(index);
                    compares += 4;
                    neighbourLookUps += 4;
                } else {
                    compares += 4;
                    neighbourLookUps += 4;
                }
            }        
        }
    }
    printf("\tVisited cells: %d\n", iterations*iterations);
    printf("\tf-Space cells: %d\n", fSpaceCells);
    printf("\tFound frontierCells: %d\n", frontierCells.size());
    printf("\tCompares: %d\n", compares);
    printf("\tNeighbour lookups: %d\n", neighbourLookUps);
    return frontierCells;
}

vec_double MapOperations::computeAdjacencyMatrixOfFrontierCells(std::vector<unsigned int> &frontierCells) {

    // frontierIdxs has to be sorted!!!!
    // i.e.:
    // frontierIdxs = {12, 25, 26, 31}
    // right(25) = 26
    // => frontierIdxs[i] = frontierIdxs[i+1]-1 => right(frontierIdxs[i]) = frontierIdxs[i+1]
    // therefore we can easily check left and right neighbours by just looking up next/previous index in frontierIdxs

    std::vector<std::vector<unsigned int> > adjacencyMatrixOfFrontiers;
    std::vector<unsigned int> neighbours;
    int width = this->map_->info.width;

    // s = frontierIdxs.size()
    // O([sum(i=1;s-1;i)*2 + 2] * s)
    // O(s*s*s)
    for (unsigned int position = 0; position < frontierCells.size(); position++) {
        int frontierIndex = frontierCells[position];
        neighbours.push_back(0);
        neighbours.push_back(frontierIndex);
        // bottom
        for (unsigned int i = 0; i < position; i++) {
            int currentElement = frontierCells[i];
            if (currentElement == frontierIndex-width-1) neighbours.push_back(currentElement);
            else if (currentElement == frontierIndex-width) neighbours.push_back(currentElement);
            else if (currentElement == frontierIndex-width+1) {neighbours.push_back(currentElement); break;}
        }
        // left
        if (frontierCells[position-1] == frontierIndex-1) neighbours.push_back(frontierCells[position-1]);
        // right
        if (frontierCells[position+1] == frontierIndex+1) neighbours.push_back(frontierCells[position+1]);
        // top
        for (unsigned int i = position+1; i < frontierCells.size(); i++) {
            int currentElement = frontierCells[i];
            if (currentElement == frontierIndex+width-1) neighbours.push_back(currentElement);
            else if (currentElement == frontierIndex+width) neighbours.push_back(currentElement);
            else if (currentElement == frontierIndex+width+1) {neighbours.push_back(currentElement); break;}
        }
        adjacencyMatrixOfFrontiers.push_back(neighbours);
        neighbours.clear();
    }
    return adjacencyMatrixOfFrontiers;
}

vec_double MapOperations::computeFrontierRegions(vec_double &adjacencyMatrixOfFrontierCells) {
    vec_double frontierRegions;
    vec_single frontierRegion;
    int component = 1;
    for (unsigned int i = 0; i < adjacencyMatrixOfFrontierCells.size(); i++) {
        if (adjacencyMatrixOfFrontierCells[i][0] == 0) {
            recursivelyComputeFrontierRegions(adjacencyMatrixOfFrontierCells, frontierRegion, i, component);
            frontierRegions.push_back(frontierRegion);
            frontierRegion.clear();
            component++;
        }
    }
    return frontierRegions;
}

void MapOperations::recursivelyComputeFrontierRegions(vec_double &adjacencyMatrixOfFrontierCells, std::vector<unsigned int> &neighbours, int index, int component) {
    // point not yet visited
    if (adjacencyMatrixOfFrontierCells[index][0] == 0) {
        adjacencyMatrixOfFrontierCells[index][0] = component;
        neighbours.push_back(adjacencyMatrixOfFrontierCells[index][1]);
        for (unsigned int i = 2; i < adjacencyMatrixOfFrontierCells[index].size(); i++) {
            int nextIndex = adjacencyMatrixOfFrontierCells[index][i];
            for (unsigned int j = 0; j < adjacencyMatrixOfFrontierCells.size(); j++) {
                if (adjacencyMatrixOfFrontierCells[j][1] == nextIndex) {
                    recursivelyComputeFrontierRegions(adjacencyMatrixOfFrontierCells, neighbours, j, component);
                }
            }
        }
    }
}
