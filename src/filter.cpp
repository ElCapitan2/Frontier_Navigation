#include "map_operations.h"

void MapOperations::preFilterMap_fi(const geometry_msgs::PoseStamped &center, double radius) {

    printf("Filtering map using fi...\n");

//    nav_msgs::GridCells min1;

    unsigned int startCell;
    int iterations;
    setupSearchArea(center.pose.position, radius, this->map_, startCell, iterations);

    PreFilterMap_1 log(0, radius, center, startCell, iterations);

    boost::shared_ptr<nav_msgs::OccupancyGrid> filteredMap(new nav_msgs::OccupancyGrid);
    filteredMap->data = map_->data;
    filteredMap->header = map_->header;
    filteredMap->info = map_->info;

    int filterCycles = 0;
    int filteredCells = 0;
    int filteredCellsInIt = 0;
    int ops = 0;
    int opsInIt = 0;

    std::vector<int8_t> filteredData = map_->data;
    filteredMap->data = filteredData;
    bool go_on = true;
    while (go_on) {
        filterCycles++;
        go_on = false;
        for (int i = 0; i < iterations; i++) {
            for (int j = 0; j < iterations; j++) {
                unsigned int index = startCell + j + i*map_->info.width;
                opsInIt += 19;
                if (isUSpace(index, filteredMap)) {
                    int kernel = neighbourhoodValue(index, filteredMap);
                    if (kernel <= 0 && kernel >= -3) {
                        filteredCellsInIt++;
                        go_on = true;
//                        min1.cells.push_back(Helpers::gridToPoint(index, map_));
                        filteredMap->data[index] = F_SPACE;
                    }
                }
            }
        }
        log.cntOfFilteredCellsPerCycle.push_back(filteredCellsInIt);
        log.cntOfOpsPerCycle.push_back(opsInIt);
        log.cntOfAdditionalOpsPerCycle.push_back(0);
        filteredCells += filteredCellsInIt;
        ops += opsInIt;
        filteredCellsInIt = 0;
        opsInIt = 0;
    }

    printf("\tRadius: %f\n", radius);
    printf("\tFilter cylces: %d\n", filterCycles);
    printf("\tFiltered cells: %d\n", filteredCells);
    printf("\tActual runtime: %d\n", ops);
//    min1.cell_height = min1.cell_width = map_->info.resolution;
//    min1.header.frame_id = "/map";
//    this->min1_pub_.publish(min1);
//    check(min1.cells);

    log.printLog(filteredCells, ops, 0);

//    this->map_ = filteredMap;
}

void MapOperations::preFilterMap_Fi(const geometry_msgs::PoseStamped &center, double radius) {

    printf("Filtering map using Fi...\n");

    unsigned int startCell;
    int iterations;
    setupSearchArea(center.pose.position, radius, this->map_, startCell, iterations);

    PreFilterMap_1 log(0, radius, center, startCell, iterations);

    boost::shared_ptr<nav_msgs::OccupancyGrid> filteredMap(new nav_msgs::OccupancyGrid);
    filteredMap->data = map_->data;
    filteredMap->header = map_->header;
    filteredMap->info = map_->info;

    int filterCycles = 0;
    int filteredCells = 0;
    int filteredCellsInIt = 0;
    int ops = 0;
    int opsInIt = 0;

    std::vector<int8_t> filteredData = map_->data;
    filteredMap->data = filteredData;
    bool go_on = true;
    while (go_on) {
        filterCycles++;
        go_on = false;
        for (int i = 0; i < iterations; i++) {
            for (int j = 0; j < iterations; j++) {
                unsigned int index = startCell + j + i*map_->info.width;
                opsInIt++;
                if (isUSpace(index, filteredMap)) {
                    opsInIt += 17;
                    int kernel = neighbourhoodValue(index, filteredMap);
                    if (kernel <= 0 && kernel >= -3) {
                        ops++;
                        filteredCellsInIt++;
                        go_on = true;
                        filteredMap->data[index] = F_SPACE;
                    }
                }
            }
        }
        log.cntOfAdditionalOpsPerCycle.push_back(0);
        log.cntOfFilteredCellsPerCycle.push_back(filteredCellsInIt);
        log.cntOfOpsPerCycle.push_back(opsInIt);
        filteredCells += filteredCellsInIt;
        ops += opsInIt;
        filteredCellsInIt = 0;
        opsInIt = 0;
    }

    printf("\tRadius: %f\n", radius);
    printf("\tFilter cylces: %d\n", filterCycles);
    printf("\tFiltered cells: %d\n", filteredCells);
    printf("\tActual runtime: %d\n", ops);

    log.printLog(filteredCells, ops, 0);

//    this->map_ = filteredMap;
}

void MapOperations::preFilterMap_FI(const geometry_msgs::PoseStamped &center, double radius) {

    printf("Filtering map using FI...\n");

    unsigned int startCell;
    int iterations;
    setupSearchArea(center.pose.position, radius, this->map_, startCell, iterations);

    PreFilterMap_2 logs(0, radius, center, startCell, iterations);

    boost::shared_ptr<nav_msgs::OccupancyGrid> filteredMap(new nav_msgs::OccupancyGrid);
    filteredMap->data = map_->data;
    filteredMap->header = map_->header;
    filteredMap->info = map_->info;

    int filterCycles = 0;
    int filteredCells = 0;
    int filteredCellsInIt = 0;
    int addOps = 0;
    int addOpsInIt = 0;
    int ops = 0;
    int opsInIt = 0;

    // at first every cell within radius potentially is a cell which can be filtered
    vec_single importantIdxs;
    ops += iterations*iterations;
    for (int i = 0; i < iterations; i++) {
        for (int j = 0; j < iterations; j++) {
            importantIdxs.push_back(startCell + j + i*map_->info.width);
        }
    }

    unsigned int centerCell = pointToCell(center.pose.position, this->map_);
    while (importantIdxs.size() > 0) {
        filterCycles++;
        vec_single temp;
        unsigned int index;
        for (int i = 0; i < importantIdxs.size(); i++) {
            index = importantIdxs[i];
            opsInIt += 3;
            if (isUSpace(index, filteredMap)) {
                if (filterCycles > 1) {
                    if (Helpers::distance(index, centerCell, this->map_->info.width, this->map_->info.resolution) >= SQRT2 * radius) {
                    continue;
                    }
                }
                opsInIt += 17;
                int kernel = neighbourhoodValue(index, filteredMap);
                if (kernel <= 0 && kernel >= -3) {
                    filteredCellsInIt++;
                    opsInIt += 12;
                    filteredMap->data[index] = F_SPACE;
                    // fetch important indices for next round
                    if (getLeftVal(index, filteredMap) == -1) temp.push_back(getLeftCell(index, map_));
                    if (getRightVal(index, filteredMap) == -1) temp.push_back(getRightCell(index, map_));
                    if (getTopVal(index, filteredMap) == -1) temp.push_back(getTopCell(index, map_));
                    if (getBottomVal(index, filteredMap) == -1) temp.push_back(getBottomCell(index, map_));
                    if (getLeftTopVal(index, filteredMap) == -1) temp.push_back(getLeftTopCell(index, map_));
                    if (getLeftBottomVal(index, filteredMap) == -1) temp.push_back(getLeftBottomCell(index, map_));
                    if (getRightTopVal(index, filteredMap) == -1) temp.push_back(getRightTopCell(index, map_));
                    if (getRightBottomVal(index, filteredMap) == -1) temp.push_back(getRightBottomCell(index, map_));
                }
            }
        }

        logs.cntOfImpIdxsPerCycle.push_back(importantIdxs.size());
        logs.cntOfFilteredCellsPerCycle.push_back(filteredCellsInIt);
        logs.cntOfPotentialImpIdxs.push_back(temp.size());
        logs.cntOfOpsPerCycle.push_back(opsInIt);
        if (temp.size() > 0) addOpsInIt += (temp.size()-1 + temp.size() * (log(temp.size())/log(2)));
        logs.cntOfAdditionalOpsPerCycle.push_back(addOpsInIt);

        importantIdxs = Helpers::sortAndRemoveEquals(temp);

        // runtime measure
        addOps += addOpsInIt;
        ops += opsInIt;
        filteredCells += filteredCellsInIt;

        // reset
        temp.clear();
        opsInIt = 0;
        addOpsInIt = 0;
        filteredCellsInIt = 0;

    }
    printf("\tRadius: %f\n", radius);
    printf("\tFilter cylces: %d\n", filterCycles);
    printf("\tFiltered cells: %d\n", filteredCells);
    printf("\tAdditional ops: %d\n", addOps);
    printf("\tActual runtime: %d\n", ops + addOps);
    logs.printLog(filteredCells, ops, addOps);

//        this->map_ = filteredMap;
}

void MapOperations::preFilterMap_FII(const geometry_msgs::PoseStamped &center, double radius) {

    printf("Filtering map using FII...\n");

    unsigned int startCell;
    int iterations;
    setupSearchArea(center.pose.position, radius, this->map_, startCell, iterations);

    PreFilterMap_2 log(0, radius, center, startCell, iterations);

    boost::shared_ptr<nav_msgs::OccupancyGrid> filteredMap(new nav_msgs::OccupancyGrid);
    filteredMap->data = map_->data;
    filteredMap->header = map_->header;
    filteredMap->info = map_->info;

    int filterCycles = 0;
    int filteredCells = 0;
    int filteredCellsInIt = 0;
    int addOps = 0;
    int addOpsInIt = 0;
    int ops = 0;
    int opsInIt = 0;

    // at first every cell within radius potentially is a cell which can be filtered
    vec_single importantIdxs;
    ops += iterations*iterations;
    for (int i = 0; i < iterations; i++) {
        for (int j = 0; j < iterations; j++) {
            importantIdxs.push_back(startCell + j + i*map_->info.width);
        }
    }

    // very huge overhead
    // use offset!!
    // currently we use vector which goes from index 0 of map to index maxIndex of map
    // use minIndex = startCell, go from 0 to (maxIndex-minIndex) and offset it with minIndex
    int maxIndex = startCell + iterations-1 + map_->info.width*(iterations-1);
    std::vector<bool> flags;
//    addOps += maxIndex;
    for (unsigned int i = 0; i < maxIndex; i++) {
        flags.push_back(false);
    }

    unsigned int centerCell = pointToCell(center.pose.position, this->map_);
    while (importantIdxs.size() > 0) {
        filterCycles++;
        vec_single temp;
        unsigned int index;
        for (unsigned int i = 0; i < importantIdxs.size(); i++) {
            index = importantIdxs[i];
            opsInIt += 3;
            if (isUSpace(index, filteredMap)) {
                if (filterCycles > 1) {
                    if (Helpers::distance(index, centerCell, this->map_->info.width, this->map_->info.resolution) >= SQRT2 * radius) {
                    continue;
                    }
                }
                opsInIt += 17;
                int kernel = neighbourhoodValue(index, filteredMap);
                if (kernel <= 0 && kernel >= -3) {
                    filteredCellsInIt++;
                    // max 3 pushes and 8 compares and 1 write (as in FI)
                    // max 3 writes and max 8 compares (needed exclusively in FII)
                    opsInIt += 12;
                    addOpsInIt += 11;
                    filteredMap->data[index] = F_SPACE;

                    // fetch important indices for next round
                    if (getLeftVal(index, filteredMap) == -1 && flags[getLeftCell(index, map_)] == false) {
                        temp.push_back(getLeftCell(index, map_));
                        flags[getLeftCell(index, map_)] = true;
                    }
                    if (getRightVal(index, filteredMap) == -1 && flags[getRightCell(index, map_)] == false) {
                        temp.push_back(getRightCell(index, map_));
                        flags[getRightCell(index, map_)] = true;
                    }
                    if (getTopVal(index, filteredMap) == -1 && flags[getTopCell(index, map_)] == false) {
                        temp.push_back(getTopCell(index, map_));
                        flags[getTopCell(index, map_)] = true;
                    }
                    if (getBottomVal(index, filteredMap) == -1 && flags[getBottomCell(index, map_)] == false) {
                        temp.push_back(getBottomCell(index, map_));
                        flags[getBottomCell(index, map_)] = true;
                    }
                    if (getLeftTopVal(index, filteredMap) == -1 && flags[getLeftTopCell(index, map_)] == false) {
                        temp.push_back(getLeftTopCell(index, map_));
                        flags[getLeftTopCell(index, map_)] = true;
                    }
                    if (getLeftBottomVal(index, filteredMap) == -1 && flags[getLeftBottomCell(index, map_)] == false) {
                        temp.push_back(getLeftBottomCell(index, map_));
                        flags[getLeftBottomCell(index, map_)] = true;
                    }
                    if (getRightTopVal(index, filteredMap) == -1 && flags[getRightTopCell(index, map_)] == false) {
                        temp.push_back(getRightTopCell(index, map_));
                        flags[getRightTopCell(index, map_)] = true;
                    }
                    if (getRightBottomVal(index, filteredMap) == -1 && flags[getRightBottomCell(index, map_)] == false) {
                        temp.push_back(getRightBottomCell(index, map_));
                        flags[getRightBottomCell(index, map_)] = true;
                    }
                }
            }
        }

        log.cntOfImpIdxsPerCycle.push_back(importantIdxs.size());
        log.cntOfFilteredCellsPerCycle.push_back(filteredCellsInIt);
        log.cntOfPotentialImpIdxs.push_back(temp.size());
        log.cntOfOpsPerCycle.push_back(opsInIt);
        log.cntOfAdditionalOpsPerCycle.push_back(addOpsInIt);

        importantIdxs = temp;

        // runtime measure
        addOps += temp.size();
        addOps += addOpsInIt;
        ops += opsInIt;
        filteredCells += filteredCellsInIt;

        // reset
        for (unsigned int i = 0; i < temp.size(); i++) {
            flags[temp[i]] = false;
        }
        temp.clear();
        opsInIt = 0;
        addOpsInIt = 0;
        filteredCellsInIt = 0;

    }
    printf("\tRadius: %f\n", radius);
    printf("\tFilter cylces: %d\n", filterCycles);
    printf("\tFiltered cells: %d\n", filteredCells);
    printf("\tAdditional ops: %d\n", addOps);
    printf("\tActual runtime: %d\n", ops + addOps);

    log.printLog(filteredCells, ops, addOps);

    this->map_ = filteredMap;
}



void MapOperations::preFilterMap(boost::shared_ptr<nav_msgs::OccupancyGrid> &map, const geometry_msgs::PoseStamped &center, int radius) {
    this->map_ = map;
//    preFilterMap_fi(center, radius);
//    preFilterMap_Fi(center, radius);
    preFilterMap_FI(center, radius);
    preFilterMap_FII(center, radius);
    map = this->map_;
}
