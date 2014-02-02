#include "map_operations.h"

//void MapOperations::check(std::vector<geometry_msgs::Point> test) {
//    vec_single transfer;
//    for (int i = 0; i < test.size(); i++) {
//        transfer.push_back(Helpers::pointToGrid(test[i], map_));
//        if (Helpers::gridToPoint(transfer[i], map_).x != test[i].x) printf("CONVERSION ERROR\n");
//        if (Helpers::gridToPoint(transfer[i], map_).y != test[i].y) printf("CONVERSION ERROR\n");
//    }
//    printf("org: %d\n", test.size());
//    printf("copy: %d\n", Helpers::sortAndRemoveEquals(transfer).size());
//    if (Helpers::sortAndRemoveEquals(transfer).size() != test.size()) printf("SIZE ERROR\n");

//}

//void MapOperations::preFilterMap_fi(int radius) {
//    // we search in an area around robot
//    nav_msgs::GridCells min1;
//    geometry_msgs::Point pos = this->robot_position_.pose.position;
//    geometry_msgs::Point startPoint;
//    startPoint.x = pos.x - radius;
//    startPoint.y = pos.y - radius;
//    startPoint.z = 0;
//    int startIndex = Helpers::pointToGrid(startPoint, this->map_);
//    Neighbours neighbours(map_->info.width, map_->info.height);
//    int iterations = radius*2/map_->info.resolution;
//    printf("Filtering map using fi...\n");
//    const int8_t FREESPACE = 0;
//    int ops = 0;
//    int filterCycles = 0;
//    int filteredCells = 0;
//    std::vector<int8_t> filteredData = map_->data;
//    boost::shared_ptr<nav_msgs::OccupancyGrid> filteredMap(new nav_msgs::OccupancyGrid);
//    filteredMap->data = filteredData;
//    bool go_on = true;
//    while (go_on) {
//        go_on = false;
//        for (int i = 0; i < iterations; i++) {
//            for (int j = 0; j < iterations; j++) {
//                unsigned int index = startIndex + j + i*map_->info.height;
//                int8_t data = filteredMap->data[index];
//                ops += 19;
//                if ((data == -1) && (Helpers::distance(Helpers::pointToGrid(this->robot_position_.pose.position, map_), index, 4000, 0.05) < 8.0)) {
//                    int kernel = neighbours.getValLeft(index, filteredMap) + neighbours.getValRight(index, filteredMap) + neighbours.getValTop(index, filteredMap) + neighbours.getValBottom(index, filteredMap) +
//                            neighbours.getValLeftBottom(index, filteredMap) + neighbours.getValLeftTop(index, filteredMap) + neighbours.getValRightBottom(index, filteredMap) + neighbours.getValRightTop(index, filteredMap);
//                    if (kernel <= 0 && kernel >= -3) {
//                        go_on = true;
//                        min1.cells.push_back(Helpers::gridToPoint(index, map_));
//                        filteredCells++;
//                        filteredMap->data[index] = FREESPACE;
//                    }
//                }
//            }
//        }
//        filterCycles++;
//    }
//    printf("\tRadius: %d\n", radius);
//    printf("\tFilter cylces: %d\n", filterCycles);
//    printf("\tFiltered cells: %d\n", filteredCells);
//    printf("\tActual runtime: %d\n", ops);
//    min1.cell_height = min1.cell_width = map_->info.resolution;
//    min1.header.frame_id = "/map";
//    this->min1_pub_.publish(min1);
////    check(min1.cells);
//}

//void MapOperations::preFilterMap_Fi(int radius) {
//    // we search in an area around robot
//    nav_msgs::GridCells min2;
//    geometry_msgs::Point pos = this->robot_position_.pose.position;
//    geometry_msgs::Point startPoint;
//    startPoint.x = pos.x - radius;
//    startPoint.y = pos.y - radius;
//    startPoint.z = 0;
//    int startIndex = Helpers::pointToGrid(startPoint, this->map_);
//    Neighbours neighbours(map_->info.width, map_->info.height);
//    int iterations = radius*2/map_->info.resolution;
//    printf("Filtering map using Fi...\n");
//    int8_t data = 0;
//    const int8_t FREESPACE = 0;
//    int ops = 0;
//    int filterCycles = 0;
//    int filteredCells = 0;
//    std::vector<int8_t> filteredData = map_->data;
//    boost::shared_ptr<nav_msgs::OccupancyGrid> filteredMap(new nav_msgs::OccupancyGrid);
//    filteredMap->data = filteredData;
//    bool go_on = true;
//    while (go_on) {
//        go_on = false;
//        for (int i = 0; i < iterations; i++) {
//            for (int j = 0; j < iterations; j++) {
//                unsigned int index = startIndex + j + i*map_->info.height;
//                int8_t data = filteredMap->data[index];
//                ops++;
//                if ((data == -1) && (Helpers::distance(Helpers::pointToGrid(this->robot_position_.pose.position, map_), index, 4000, 0.05) < 8.0)) {
//                    ops += 17;
//                    int kernel = neighbours.getValLeft(index, filteredMap) + neighbours.getValRight(index, filteredMap) + neighbours.getValTop(index, filteredMap) + neighbours.getValBottom(index, filteredMap) +
//                            neighbours.getValLeftBottom(index, filteredMap) + neighbours.getValLeftTop(index, filteredMap) + neighbours.getValRightBottom(index, filteredMap) + neighbours.getValRightTop(index, filteredMap);
//                    if (kernel <= 0 && kernel >= -3) {
//                        ops++;
//                        go_on = true;
//                        min2.cells.push_back(Helpers::gridToPoint(index, map_));
//                        filteredCells++;
//                        filteredMap->data[index] = FREESPACE;
//                    }
//                }
//            }
//        }
//        filterCycles++;
//    }
//    printf("\tRadius: %d\n", radius);
//    printf("\tFilter cylces: %d\n", filterCycles);
//    printf("\tFiltered cells: %d\n", filteredCells);
////    printf("\tRuntime estimation: %f\n", 10*pow(2*radius/0.05, 4) + 10*pow(2*radius/0.05, 2));
//    printf("\tActual runtime: %d\n", ops);
//    min2.cell_height = min2.cell_width = map_->info.resolution;
//    min2.header.frame_id = "/map";
//    this->min2_pub_.publish(min2);
////    check(min2.cells);
//}

//void MapOperations::preFilterMap_FI(int radius) {

//    // we search in an area around robot
//    geometry_msgs::Point pos = this->robot_position_.pose.position;
//    geometry_msgs::Point startPoint;
//    startPoint.x = pos.x - radius;
//    startPoint.y = pos.y - radius;
//    startPoint.z = 0;

//    nav_msgs::GridCells min3;
//    nav_msgs::GridCells min2;
////    nav_msgs::GridCells min1;
////    nav_msgs::GridCells min2;
////    nav_msgs::GridCells min3;
////    nav_msgs::GridCells min4;

//    int startIndex = Helpers::pointToGrid(startPoint, this->map_);

//    Neighbours neighbours(map_->info.width, map_->info.height);
//    int iterations = radius*2/map_->info.resolution;
//    printf("Filtering map using FI...\n");

//    int8_t data = 0;
//    const int8_t FREESPACE = 0;
//    std::vector<int8_t> filteredData = map_->data;
//    boost::shared_ptr<nav_msgs::OccupancyGrid> filteredMap(new nav_msgs::OccupancyGrid);

//    int filterCycles = 0;
//    int filteredCells = 0;
//    int compares = 0;
//    int neighbourLookUps = 0;


//    filteredMap->data = filteredData;

//    int u_space = 0;
//    int f_space = 0;
//    int o_space = 0;
//    vec_single importantIdxs;
//    // O(iterations*iterations)
//    for (int i = 0; i < iterations; i++) {
//        for (int j = 0; j < iterations; j++) {
//            importantIdxs.push_back(startIndex + j + i*map_->info.height);
//            int8_t value = map_->data[startIndex + j + i*map_->info.height];
//            if (value == 0) f_space++;
//            if (value == -1) u_space++;
//            if (value == 100) o_space++;
//        }
//    }

////    printf("\tf: %d u: %d o: %d\n", f_space, u_space, o_space);
//    Helpers::writeToFile("max.txt", "", iterations*iterations - o_space - f_space);

//    int cntOfImportantIdxs = 0;
//    int sortingOperations = 0;
//    int ops = 0;
//    while (importantIdxs.size() > 0) {
//    int rightKernels = 0;
////    while (filterCycles < 1) {
////    while (filterCycles < 7) {
////        printf("\tMax size of importantIdxs START: %d\n", iterations*iterations - o_space - f_space);
//        vec_single temp;
////        zeros.cells.clear();
////        min1.cells.clear();
//        min2.cells.clear();
////        min3.cells.clear();
////        min4.cells.clear();
//        unsigned int index;
//        for (int i = 0; i < importantIdxs.size(); i++) {
//            index = importantIdxs[i];
////            data = filteredData[index];
//            data = filteredMap->data[index];
//            compares++;
//            ops++;
//            if ((data == -1 && true) && (Helpers::distance(Helpers::pointToGrid(this->robot_position_.pose.position, map_), index, 4000, 0.05) < 8.0)) {
//                ops += 17;
//                neighbourLookUps += 8;
//                int kernel = neighbours.getValLeft(index, filteredMap) + neighbours.getValRight(index, filteredMap) + neighbours.getValTop(index, filteredMap) + neighbours.getValBottom(index, filteredMap) +
//                        neighbours.getValLeftBottom(index, filteredMap) + neighbours.getValLeftTop(index, filteredMap) + neighbours.getValRightBottom(index, filteredMap) + neighbours.getValRightTop(index, filteredMap);
//                switch (kernel) {
////                case 0: zeros.cells.push_back(Helpers::gridToPoint(index, map_)); break;
////                case -1: min1.cells.push_back(Helpers::gridToPoint(index, map_)); break;
////                case -2: min2.cells.push_back(Helpers::gridToPoint(index, map_)); break;
////                case -3: min3.cells.push_back(Helpers::gridToPoint(index, map_)); break;
////                case -4: min4.cells.push_back(Helpers::gridToPoint(index, map_)); break;
//                default: break;
//                }
//                compares++;
//                if (kernel <= 0 && kernel >= -3) {
//                    rightKernels++;
//                    // 1 write and 8 compares and max 3 pushs
//                    ops += 12;
//                    neighbourLookUps += 8;
////                    filteredData[index] = FREESPACE;
//                    filteredCells++;
//                    filteredMap->data[index] = FREESPACE;
//                    f_space++;
//                    compares += 4;
//                    min3.cells.push_back(Helpers::gridToPoint(index, map_));
//                    // fetch important indices for next round
//                    if (neighbours.getValLeft(index, filteredMap) == -1) temp.push_back(neighbours.getLeft(index));
//                    if (neighbours.getValRight(index, filteredMap) == -1) temp.push_back(neighbours.getRight(index));
//                    if (neighbours.getValTop(index, filteredMap) == -1) temp.push_back(neighbours.getTop(index));
//                    if (neighbours.getValBottom(index, filteredMap) == -1) temp.push_back(neighbours.getBottom(index));
//                    if (neighbours.getValLeftTop(index, filteredMap) == -1) temp.push_back(neighbours.getLeftTop(index));
//                    if (neighbours.getValLeftBottom(index, filteredMap) == -1) temp.push_back(neighbours.getLeftBottom(index));
//                    if (neighbours.getValRightTop(index, filteredMap) == -1) temp.push_back(neighbours.getRightTop(index));
//                    if (neighbours.getValRightBottom(index, filteredMap) == -1) temp.push_back(neighbours.getRightBottom(index));
//                }

//            }
//        }

//        // O(temp.size()*log(temp.size())) + O(temp.size())
//        if (temp.size() > 0) sortingOperations += (temp.size()-1 + temp.size() * (log(temp.size())/log(2)));
//        importantIdxs = Helpers::sortAndRemoveEquals(temp);
//        printf("TEMP: %d - IMPORTANT: %d - DIFF: %d\n", temp.size(), importantIdxs.size(), temp.size()-importantIdxs.size());

//        for (int i = 0; i < importantIdxs.size(); i++) {
//            min2.cells.push_back(Helpers::gridToPoint(importantIdxs[i], map_));
//        }
////        printf("TEMP: %d - %d\n", filterCycles, importantIdxs.size());
////        printf("rightKernels: %d\n", rightKernels);
//        rightKernels = 0;
////        for (int i = 0; i < importantIdxs.size(); i++) printf("%d\n", importantIdxs[i]);
//        filterCycles++;
////        filteredMap->data = filteredData;
//        cntOfImportantIdxs += temp.size();
//        compares++;
////        printf("\tMax size of importantIdxs: %d\n", iterations*iterations - o_space - f_space);
////        printf("\tImportant indices before cut down: %d\n", temp.size());
////        printf("\tImportant Indices after cut down : %d\n", importantIdxs.size());
//        Helpers::writeToFile("max.txt", "", iterations*iterations - o_space - f_space);
//        Helpers::writeToFile("before.txt", "", temp.size());
//        Helpers::writeToFile("after.txt", "", importantIdxs.size());
//        temp.clear();
////        printf("Flip cylces: %d\n", flipCycles);
////        printf("Compares: %d\n", compares);
////        printf("Neighbour look ups: %d\n", neighbourLookUps);
////        for (int i = 0; i < importantIdxs.size(); i++) min4.cells.push_back(Helpers::gridToPoint(importantIdxs[i], map_));
////        min4.cell_height = min4.cell_width = map_->info.resolution;
////        min4.header.frame_id = "/map";
////        this->min4_pub_.publish(min4);
////        check(min3.cells);
//    }

//    Helpers::writeToFile("filter.txt", "Filtering map...", -1);
//    Helpers::writeToFile("filter.txt", "Filter cycles", filterCycles);
//    Helpers::writeToFile("filter.txt", "Filtered cells", filteredCells);
//    Helpers::writeToFile("filter.txt", "Total count of important indices", cntOfImportantIdxs);
//    Helpers::writeToFile("filter.txt", "Average size of importantIdxs", cntOfImportantIdxs/filterCycles);
//    Helpers::writeToFile("filter.txt", "Compares", compares);
//    Helpers::writeToFile("filter.txt", "Neighbour lookups", neighbourLookUps);
//    Helpers::writeToFile("filter.txt", "Sorting ops", sortingOperations);
//    Helpers::writeToFile("filter.txt", "Operations", iterations*iterations + sortingOperations + compares + neighbourLookUps);

////    for (int i = 0; i < importantIdxs.size(); i++) min4.cells.push_back(Helpers::gridToPoint(importantIdxs[i], map_));

//    min3.cell_height = min3.cell_width = map_->info.resolution;
////    min1.cell_height = min1.cell_width = map_->info.resolution;
////    min2.cell_height = min2.cell_width = map_->info.resolution;
////    min3.cell_height = min3.cell_width = map_->info.resolution;
////    min4.cell_height = min4.cell_width = map_->info.resolution;
//    min3.header.frame_id = "/map"; //min1.header.frame_id = min2.header.frame_id = min3.header.frame_id = min4.header.frame_id = "/map";
//    min2.cell_height = min2.cell_width = map_->info.resolution;
//    min2.header.frame_id = "/map";
//    filteredMap->header = map_->header;
//    filteredMap->info = map_->info;
//    printf("\tRadius: %d\n", radius);
//    printf("\tFilter cylces: %d\n", filterCycles);
//    printf("\tFiltered cells: %d\n", filteredCells);
////    printf("\tCompares: %d\n", compares);
////    printf("\tNeighbour lookups: %d\n", neighbourLookUps);
//    printf("\tSorting ops: %d\n", sortingOperations);
//    printf("\tActual runtime: %d\n", ops + sortingOperations);
////    printf("\tOperations: %d\n", iterations*iterations + sortingOperations + compares + neighbourLookUps);

//    this->min3_pub_.publish(min3);
//    this->min2_pub_.publish(min2);
////    check(min3.cells);
////    this->min1_pub_.publish(min1);
////    this->min2_pub_.publish(min2);
////    this->min3_pub_.publish(min3);
////    this->min4_pub_.publish(min4);
////    this->filteredMap_pub_.publish(filteredMap);
//}

void MapOperations::preFilterMap_FII(const geometry_msgs::PoseStamped &center, int radius) {

    printf("Filtering map using FII...\n");

    // we search in an area around robot
    nav_msgs::GridCells min4;
    nav_msgs::GridCells min1;

    // setup search area
    int startCell;
    int iterations;
    setupSearchArea(center, radius, this->map_, startCell, iterations);

//    Neighbours neighbours(map_->info.width, map_->info.height);





    int8_t data = 0;
//    const int8_t FREESPACE = 0;
    std::vector<int8_t> filteredData = map_->data;
    boost::shared_ptr<nav_msgs::OccupancyGrid> filteredMap(new nav_msgs::OccupancyGrid);
    filteredMap->header = map_->header;
    filteredMap->info = map_->info;

    int filterCycles = 0;
    int filteredCells = 0;
    int additionalOperations = 0;

    filteredMap->data = filteredData;

    vec_single importantIdxs;
    // O(iterations*iterations)
    for (int i = 0; i < iterations; i++) {
        for (int j = 0; j < iterations; j++) {
            importantIdxs.push_back(startCell + j + i*map_->info.height);
        }
    }

    int ops = 0;
    std::vector<bool> flags;
    for (unsigned int i = 0; i < map_->info.height * map_->info.width; i++) {
        flags.push_back(false);
    }
    int filteredCellsInIteration = 0;





    while (importantIdxs.size() > 0) {




//    while (filterCycles < 2) {
        filterCycles++;
//        printf("\tCycle %d\n", filterCycles);
//        printf("\t\tImportantIdxs.size() = %d\n", importantIdxs.size());
        min1.cells.clear();
        vec_single temp;
        unsigned int index;




        for (unsigned int i = 0; i < importantIdxs.size(); i++) {
            index = importantIdxs[i];
//            data = filteredData[index];
            data = filteredMap->data[index];
            ops++;
//            if ((data == -1 && true) && (Helpers::distance(Helpers::pointToGrid(this->robot_position_.pose.position, map_), index, 4000, 0.05) < sqrt(2.0)*radius)) {




            if (data == -1) {



                ops += 17;



//                int kernel = neighbours.getValLeft(index, filteredMap) + neighbours.getValRight(index, filteredMap) + neighbours.getValTop(index, filteredMap) + neighbours.getValBottom(index, filteredMap) +
//                        neighbours.getValLeftBottom(index, filteredMap) + neighbours.getValLeftTop(index, filteredMap) + neighbours.getValRightBottom(index, filteredMap) + neighbours.getValRightTop(index, filteredMap);
                int kernel = neighbourhoodValue(index, filteredMap);



                if (kernel <= 0 && kernel >= -3) {





                    filteredCellsInIteration++;
                    // 1 write and max 16 compares and max 3 pushs and max 3 writes
                    ops += 12;
                    additionalOperations += 11;
                    filteredCells++;
                    filteredMap->data[index] = F_SPACE;
                    min4.cells.push_back(cellToPoint(index, map_));


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


//        printf("\t\tFiltered Cells: %d - %d\n", filterCycles, filteredCellsInIteration);
//        printf("\t\tPotentials: %d - %d\n", filterCycles, temp.size());
        filteredCellsInIteration = 0;
        additionalOperations += temp.size();
        for (unsigned int i = 0; i < temp.size(); i++) {
            flags[temp[i]] = false;
        }
        importantIdxs = Helpers::sortAndRemoveEquals(temp);

        for (unsigned int i = 0; i < importantIdxs.size(); i++) {
            min1.cells.push_back(Helpers::gridToPoint(importantIdxs[i], map_));
        }
//        for (int i = 0; i < importantIdxs.size(); i++) printf("%d\n", importantIdxs[i]);
        temp.clear();


    }





    printf("\tRadius: %d\n", radius);
    printf("\tFilter cylces: %d\n", filterCycles);
    printf("\tFiltered cells: %d\n", filteredCells);
    printf("\tAdditional ops: %d\n", additionalOperations);
    printf("\tActual runtime: %d\n", ops + additionalOperations);
//    min4.cell_height = min4.cell_width = map_->info.resolution;
//    min4.header.frame_id = "/map";
//    min1.cell_height = min1.cell_width = map_->info.resolution;
//    min1.header.frame_id = "/map";
//    this->min4_pub_.publish(min4);
//    this->min1_pub_.publish(min1);

//    this->filteredMap_pub_.publish(filteredMap);

    this->map_ = filteredMap;

}



void MapOperations::preFilterMap(boost::shared_ptr<nav_msgs::OccupancyGrid> &map, const geometry_msgs::PoseStamped &center, int radius) {
    this->map_ = map;
//    preFilterMap_fi(radius);
//    preFilterMap_Fi(radius);
//    preFilterMap_FI(radius);
    preFilterMap_FII(center, radius);
    map = this->map_;
}
