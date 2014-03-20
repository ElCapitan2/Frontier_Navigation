#include "map_operations.h"

//void MapOperations::preFilterMap_fi(const geometry_msgs::PoseStamped &center, double radius) {

//    printf("Filtering map using fi...\n");

////    nav_msgs::GridCells min1;

//    unsigned int startCell;
//    int iterations;
//    setupSearchArea(center.pose.position, radius, this->map_, startCell, iterations);

////    PreFilterMap_1 log(0, radius, center, startCell, iterations);

//    boost::shared_ptr<nav_msgs::OccupancyGrid> filteredMap(new nav_msgs::OccupancyGrid);
//    filteredMap->data = map_->data;
//    filteredMap->header = map_->header;
//    filteredMap->info = map_->info;

//    int filterCycles = 0;
//    int filteredCells = 0;
//    int filteredCellsInIt = 0;
////    int ops = 0;
////    int opsInIt = 0;

//    std::vector<int8_t> filteredData = map_->data;
//    filteredMap->data = filteredData;
//    bool go_on = true;
//    while (go_on) {
//        filterCycles++;
//        go_on = false;
//        for (int i = 0; i < iterations; i++) {
//            for (int j = 0; j < iterations; j++) {
//                unsigned int index = startCell + j + i*map_->info.width;
////                opsInIt += 19;
//                if (isUSpace(index, filteredMap)) {
//                    int kernel = neighbourhoodValue(index, filteredMap);
//                    if (kernel <= 0 && kernel >= -3) {
////                        filteredCellsInIt++;
//                        go_on = true;
////                        min1.cells.push_back(Helpers::gridToPoint(index, map_));
//                        filteredMap->data[index] = F_SPACE;
//                    }
//                }
//            }
//        }
////        log.cntOfFilteredCellsPerCycle.push_back(filteredCellsInIt);
////        log.cntOfOpsPerCycle.push_back(opsInIt);
////        log.cntOfAdditionalOpsPerCycle.push_back(0);
//        filteredCells += filteredCellsInIt;
////        ops += opsInIt;
//        filteredCellsInIt = 0;
////        opsInIt = 0;
//    }

//    printf("\tRadius: %f\n", radius);
//    printf("\tFilter cylces: %d\n", filterCycles);
//    printf("\tFiltered cells: %d\n", filteredCells);
////    printf("\tActual runtime: %d\n", ops);
////    min1.cell_height = min1.cell_width = map_->info.resolution;
////    min1.header.frame_id = "/map";
////    this->min1_pub_.publish(min1);
////    check(min1.cells);

////    log.printLog(filteredCells, ops, 0);

////    this->map_ = filteredMap;
//}

//void MapOperations::preFilterMap_Fi(const geometry_msgs::PoseStamped &center, double radius) {

//    printf("Filtering map using Fi...\n");

//    unsigned int startCell;
//    int iterations;
//    setupSearchArea(center.pose.position, radius, this->map_, startCell, iterations);

//    PreFilterMap_1 log(0, radius, center, startCell, iterations);

//    boost::shared_ptr<nav_msgs::OccupancyGrid> filteredMap(new nav_msgs::OccupancyGrid);
//    filteredMap->data = map_->data;
//    filteredMap->header = map_->header;
//    filteredMap->info = map_->info;

//    int filterCycles = 0;
//    int filteredCells = 0;
//    int filteredCellsInIt = 0;
//    int ops = 0;
//    int opsInIt = 0;
//    int kernels = 0;
//    int isXSpace = 0;
//    int isXSpaceInIt = 0;
//    int kernelsInIt = 0;

//    std::vector<int8_t> filteredData = map_->data;
//    filteredMap->data = filteredData;
//    bool go_on = true;
//    unsigned int index;
//    while (go_on) {
//        filterCycles++;
//        go_on = false;
//        for (int i = 0; i < iterations; i++) {
//            for (int j = 0; j < iterations; j++) {
//                index = startCell + j + i*map_->info.width;
//                isXSpaceInIt++;
//                if (isUSpace(index, filteredMap)) {
//                    kernelsInIt++;
//                    int kernel = neighbourhoodValue(index, filteredMap);
//                    opsInIt += 3;
//                    if (kernel <= 0 && kernel >= -3) {
//                        opsInIt += 2;
//                        filteredCellsInIt++;
//                        go_on = true;
//                        filteredMap->data[index] = F_SPACE;
//                    }
//                }
//            }
//        }
//        log.cntOfAdditionalOpsPerCycle.push_back(0);
//        log.cntOfFilteredCellsPerCycle.push_back(filteredCellsInIt);
//        log.cntOfOpsPerCycle.push_back(opsInIt);
//        log.cntOfKernelsInCycle.push_back(kernelsInIt);
//        log.cntOfIsXSpaceInCycle.push_back(isXSpaceInIt);
//        filteredCells += filteredCellsInIt;
//        ops += opsInIt;
//        filteredCellsInIt = 0;
//        opsInIt = 0;
//        kernels += kernelsInIt;
//        isXSpace += isXSpaceInIt;
//        kernelsInIt = 0;
//        isXSpaceInIt = 0;
//    }

//    printf("\tRadius: %f\n", radius);
//    printf("\tFilter cylces: %d\n", filterCycles);
//    printf("\tFiltered cells: %d\n", filteredCells);
////    printf("\tActual runtime: %d\n", ops);

//    log.printLog(filteredCells, ops, 0, kernels, isXSpace);

////    this->map_ = filteredMap;
//}

//void MapOperations::preFilterMap_FI(const geometry_msgs::PoseStamped &center, double radius) {

//    printf("Filtering map using FI...\n");

//    unsigned int startCell;
//    int iterations;
//    setupSearchArea(center.pose.position, radius, this->map_, startCell, iterations);

//    PreFilterMap_2 logs(0, radius, center, startCell, iterations, true);

//    boost::shared_ptr<nav_msgs::OccupancyGrid> filteredMap(new nav_msgs::OccupancyGrid);
//    filteredMap->data = map_->data;
//    filteredMap->header = map_->header;
//    filteredMap->info = map_->info;

//    int filterCycles = 0;
//    int filteredCells = 0;
//    int filteredCellsInIt = 0;
//    int addOps = 0;
//    int addOpsInIt = 0;
//    int ops = 0;
//    int opsInIt = 0;
//    int uSpace = 0;
//    int fSpace = 0;
//    int oSpace = 0;
//    int distances = 0;
//    int distancesInIt = 0;
//    int kernels = 0;
//    int kernelsInIt = 0;
//    int isXSpace = 0;
//    int isXSpaceInIt = 0;
//    int getXVal = 0;
//    int getXValInIt = 0;
//    int getXCell = 0;
//    int getXCellInIt = 0;

//    // at first every cell within radius potentially is a cell which can be filtered
//    vec_single importantIdxs;
//    ops += iterations*iterations;
//    for (int i = 0; i < iterations; i++) {
//        for (int j = 0; j < iterations; j++) {
//            importantIdxs.push_back(startCell + j + i*map_->info.width);
//        }
//    }
//    unsigned int centerCell = pointToCell(center.pose.position, this->map_);
//    while (importantIdxs.size() > 0) {
//        filterCycles++;

//        unsigned int index;
//        for (int i = 0; i < importantIdxs.size(); i++) {
//            index = importantIdxs[i];
//            if (isUSpace(index, filteredMap)) uSpace++;
//            if (isOSpace(index, filteredMap)) oSpace++;
//            if (isFSpace(index, filteredMap)) fSpace++;
//        }
//        logs.cntOfUSpaceCells.push_back(uSpace);
//        logs.cntOfOSpaceCells.push_back(oSpace);
//        logs.cntOfFSpaceCells.push_back(fSpace);
//        uSpace = 0;
//        oSpace = 0;
//        fSpace = 0;

//        vec_single temp;

//        for (int i = 0; i < importantIdxs.size(); i++) {
//            index = importantIdxs[i];
//            isXSpaceInIt++;
//            if (isUSpace(index, filteredMap)) {
//                opsInIt++;
//                if (filterCycles > 1) {
//                    distancesInIt++;
//                    if (Helpers::distance(index, centerCell, this->map_->info.width, this->map_->info.resolution) >= SQRT2 * radius) {
//                    continue;
//                    }
//                }
//                kernelsInIt++;
//                int kernel = neighbourhoodValue(index, filteredMap);
//                opsInIt += 3;
//                if (kernel <= 0 && kernel >= -3) {
//                    filteredCellsInIt++;
//                    opsInIt += 12;
//                    getXValInIt += 8;
//                    getXCellInIt += 3;
//                    filteredMap->data[index] = F_SPACE;
//                    // fetch important indices for next round
//                    if (getLeftVal(index, filteredMap) == -1) temp.push_back(getLeftCell(index, map_));
//                    if (getRightVal(index, filteredMap) == -1) temp.push_back(getRightCell(index, map_));
//                    if (getTopVal(index, filteredMap) == -1) temp.push_back(getTopCell(index, map_));
//                    if (getBottomVal(index, filteredMap) == -1) temp.push_back(getBottomCell(index, map_));
//                    if (getLeftTopVal(index, filteredMap) == -1) temp.push_back(getLeftTopCell(index, map_));
//                    if (getLeftBottomVal(index, filteredMap) == -1) temp.push_back(getLeftBottomCell(index, map_));
//                    if (getRightTopVal(index, filteredMap) == -1) temp.push_back(getRightTopCell(index, map_));
//                    if (getRightBottomVal(index, filteredMap) == -1) temp.push_back(getRightBottomCell(index, map_));
//                }
//            }
//        }

//        logs.cntOfImpIdxsPerCycle.push_back(importantIdxs.size());
//        logs.cntOfFilteredCellsPerCycle.push_back(filteredCellsInIt);
//        logs.cntOfPotentialImpIdxs.push_back(temp.size());
//        logs.cntOfOpsPerCycle.push_back(opsInIt);
//        logs.cntOfDistancesInCycle.push_back(distancesInIt);
//        logs.cntOfKernelsInCycle.push_back(kernelsInIt);
//        logs.cntOfIsXSpaceInCycle.push_back(isXSpaceInIt);
//        logs.cntOfGetXCellInCycle.push_back(getXCellInIt);
//        logs.cntOfGetXValInCycle.push_back(getXValInIt);
//        if (temp.size() > 0) addOpsInIt += (temp.size()-1 + temp.size() * (log(temp.size())/log(2)));
//        logs.cntOfAdditionalOpsPerCycle.push_back(addOpsInIt);

//        importantIdxs = Helpers::sortAndRemoveEquals(temp);

//        // runtime measure
//        addOps += addOpsInIt;
//        ops += opsInIt;
//        filteredCells += filteredCellsInIt;
//        distances += distancesInIt;
//        kernels += kernelsInIt;
//        isXSpace += isXSpaceInIt;
//        getXVal += getXValInIt;
//        getXCell += getXCellInIt;

//        // reset
//        temp.clear();
//        opsInIt = 0;
//        addOpsInIt = 0;
//        filteredCellsInIt = 0;
//        distancesInIt = 0;
//        kernelsInIt = 0;
//        isXSpaceInIt = 0;
//        getXValInIt = 0;
//        getXCellInIt = 0;

//    }
//    printf("\tRadius: %f\n", radius);
//    printf("\tFilter cylces: %d\n", filterCycles);
//    printf("\tFiltered cells: %d\n", filteredCells);

//    logs.printLog(filteredCells, ops, addOps, distances, kernels, isXSpace, getXVal, getXCell);

////        this->map_ = filteredMap;
//}

void MapOperations::preFilterMap_FII(const geometry_msgs::PoseStamped &center, double radius) {

    printf("Filtering map using FII...\n");

    unsigned int startCell;
    int xIts, yIts;
    setupRectangleArea(center.pose.position, radius, this->map_, startCell, xIts, yIts);

    PreFilterMap_2 log(0, radius, center, startCell, 0, false);

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
    int uSpace = 0;
    int fSpace = 0;
    int oSpace = 0;
    int distances = 0;
    int distancesInIt = 0;
    int kernels = 0;
    int kernelsInIt = 0;
    int isXSpace = 0;
    int isXSpaceInIt = 0;
    int getXVal = 0;
    int getXValInIt = 0;
    int getXCell = 0;
    int getXCellInIt = 0;

    // at first every cell within radius potentially is a cell which can be filtered
    vec_single importantIdxs;
    ops += xIts*yIts;
    for (int i = 0; i < yIts; i++) {
        for (int j = 0; j < xIts; j++) {
            importantIdxs.push_back(startCell + j + i*map_->info.width);
        }
    }

//    int maxIndex = startCell + iterations-1 + map_->info.width*(iterations-1);

    std::vector<bool> flags;
//    addOps += maxIndex-startCell;
//    for (unsigned int i = 0; i < maxIndex; i++) {
//        flags.push_back(false);
//    }
    for (unsigned int i = 0; i < map_->info.width*map_->info.height; i++) {
        flags.push_back(false);
    }

    unsigned int centerCell = pointToCell(center.pose.position, this->map_);
//    int size = map_->info.height*map_->info.width;
    while (importantIdxs.size() > 0) {
        filterCycles++;

//        bool flags[16000000] = {false};
        unsigned int index;
        for (int i = 0; i < importantIdxs.size(); i++) {
            index = importantIdxs[i];
            if (isUSpace(index, filteredMap)) uSpace++;
            if (isOSpace(index, filteredMap)) oSpace++;
            if (isFSpace(index, filteredMap)) fSpace++;
        }
        log.cntOfUSpaceCells.push_back(uSpace);
        log.cntOfOSpaceCells.push_back(oSpace);
        log.cntOfFSpaceCells.push_back(fSpace);
        uSpace = 0;
        oSpace = 0;
        fSpace = 0;

        vec_single temp;
        for (unsigned int i = 0; i < importantIdxs.size(); i++) {
            index = importantIdxs[i];
            isXSpaceInIt++;
            if (isUSpace(index, filteredMap)) {
                opsInIt++;
                if (filterCycles > 1) {
                    distancesInIt++;
                    if (Helpers::distance(index, centerCell, this->map_->info.width, this->map_->info.resolution) >= SQRT2 * radius) {
                    continue;
                    }
                }
                kernelsInIt++;
                int kernel = neighbourhoodValue(index, filteredMap);
                opsInIt += 3;
                if (kernel <= 0 && kernel >= -3) {
                    filteredCellsInIt++;
                    // max 3 pushes and 8 compares and 1 write (as in FI)
                    // max 3 writes and max 8 compares (needed exclusively in FII)
                    opsInIt += 12;
                    getXValInIt += 8;
                    getXCellInIt += 3;
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
        log.cntOfDistancesInCycle.push_back(distancesInIt);
        log.cntOfKernelsInCycle.push_back(kernelsInIt);
        log.cntOfIsXSpaceInCycle.push_back(isXSpaceInIt);
        log.cntOfGetXCellInCycle.push_back(getXCellInIt);
        log.cntOfGetXValInCycle.push_back(getXValInIt);

        importantIdxs = temp;

        // runtime measure
//        addOps += temp.size();
        addOps += addOpsInIt;
        ops += opsInIt;
        filteredCells += filteredCellsInIt;
        distances += distancesInIt;
        kernels += kernelsInIt;
        isXSpace += isXSpaceInIt;
        getXVal += getXValInIt;
        getXCell += getXCellInIt;

        // reset
        for (unsigned int i = 0; i < temp.size(); i++) {
            flags[temp[i]] = false;
        }
        temp.clear();
        opsInIt = 0;
        addOpsInIt = 0;
        filteredCellsInIt = 0;
        distancesInIt = 0;
        kernelsInIt = 0;
        isXSpaceInIt = 0;
        getXValInIt = 0;
        getXCellInIt = 0;

    }
    printf("\tRadius: %f\n", radius);
    printf("\tFilter cylces: %d\n", filterCycles);
    printf("\tFiltered cells: %d\n", filteredCells);

    log.printLog(filteredCells, ops, addOps, distances, kernels, isXSpace, getXVal, getXCell);

    this->map_ = filteredMap;
}



void MapOperations::preFilterMap(boost::shared_ptr<nav_msgs::OccupancyGrid> &map, const geometry_msgs::PoseStamped &center, int radius) {
    this->map_ = map;
//    preFilterMap_fi(center, radius);
//    preFilterMap_Fi(center, radius);
//    preFilterMap_FI(center, radius);
    preFilterMap_FII(center, radius);
    map = this->map_;
}
