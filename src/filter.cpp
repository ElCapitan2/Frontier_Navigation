#include "frontier_navigation.h"
#include "helpers.h"
#include "neighbours.h"
#include <algorithm>
#include <math.h>


void Frontier_Navigation::preFilterMap(int radius) {

    // we search in an area around robot
    geometry_msgs::Point pos = this->robot_position_.pose.position;
    geometry_msgs::Point startPoint;
    startPoint.x = pos.x - radius;
    startPoint.y = pos.y - radius;
    startPoint.z = 0;

    nav_msgs::GridCells zeros;
    nav_msgs::GridCells min1;
    nav_msgs::GridCells min2;
    nav_msgs::GridCells min3;
    nav_msgs::GridCells min4;

    int startIndex = Helpers::pointToGrid(startPoint, this->map_);

    Neighbours neighbours(map_->info.width, map_->info.height);
    int iterations = radius*2/map_->info.resolution;
    printf("Iterations: %d\n", iterations);

    int8_t data = 0;
    const int8_t FREESPACE = 0;
    std::vector<int8_t> filteredData = map_->data;
    boost::shared_ptr<nav_msgs::OccupancyGrid> filteredMap(new nav_msgs::OccupancyGrid);

    int flipCycles = 0;
    int compares = 0;
    int neighbourLookUps = 0;


    filteredMap->data = filteredData;

    vec_single importantIdxs;
    // O(n*n)
    for (int i = 0; i < iterations; i++) {
        for (int j = 0; j < iterations; j++) {
            importantIdxs.push_back(startIndex + j + i*map_->info.height);
        }
    }

    int cntOfImportantIdxs = 0;
    int sortingOperations = 0;
    while (importantIdxs.size() > 0) {
        vec_single temp;
        zeros.cells.clear();
        min1.cells.clear();
        min2.cells.clear();
        min3.cells.clear();
        min4.cells.clear();
        unsigned int index;
        for (int i = 0; i < importantIdxs.size(); i++) {
            index = importantIdxs[i];
            data = filteredData[index];
            compares++;
            if (data == -1) {
                neighbourLookUps += 8;
                int kernel = neighbours.getValLeft(index, filteredMap) + neighbours.getValRight(index, filteredMap) + neighbours.getValTop(index, filteredMap) + neighbours.getValBottom(index, filteredMap) +
                        neighbours.getValLeftBottom(index, filteredMap) + neighbours.getValLeftTop(index, filteredMap) + neighbours.getValRightBottom(index, filteredMap) + neighbours.getValRightTop(index, filteredMap);
                switch (kernel) {
                case 0: zeros.cells.push_back(Helpers::gridToPoint(index, map_)); break;
                case -1: min1.cells.push_back(Helpers::gridToPoint(index, map_)); break;
                case -2: min2.cells.push_back(Helpers::gridToPoint(index, map_)); break;
                case -3: min3.cells.push_back(Helpers::gridToPoint(index, map_)); break;
                default: break;
                }
                if (kernel <= 0 && kernel >= -3) {
                    neighbourLookUps += 16;
                    filteredData[index] = FREESPACE;
//                    filteredMap->data[index] = FREESPACE;
                    // fetch important indices for next round
                    if (neighbours.getValLeft(index, filteredMap) == -1) temp.push_back(neighbours.getLeft(index));
                    if (neighbours.getValRight(index, filteredMap) == -1) temp.push_back(neighbours.getRight(index));
                    if (neighbours.getValTop(index, filteredMap) == -1) temp.push_back(neighbours.getTop(index));
                    if (neighbours.getValBottom(index, filteredMap) == -1) temp.push_back(neighbours.getBottom(index));
                    if (neighbours.getValLeftTop(index, filteredMap) == -1) temp.push_back(neighbours.getLeftTop(index));
                    if (neighbours.getValLeftBottom(index, filteredMap) == -1) temp.push_back(neighbours.getLeftBottom(index));
                    if (neighbours.getValRightTop(index, filteredMap) == -1) temp.push_back(neighbours.getRightTop(index));
                    if (neighbours.getValRightBottom(index, filteredMap) == -1) temp.push_back(neighbours.getRightBottom(index));
                }

            }
        }
        importantIdxs = Helpers::sortAndRemoveEquals(temp);
        flipCycles++;
        filteredMap->data = filteredData;
        cntOfImportantIdxs += temp.size();
        if (temp.size() > 0) sortingOperations += temp.size() * (log(temp.size())/log(2));
        Helpers::writeToFile("/home/u_private/ros_develop/frontier_navigation/files/filter.txt", "temp", temp.size());
        Helpers::writeToFile("/home/u_private/ros_develop/frontier_navigation/files/filter.txt", "importantIdxs", importantIdxs.size());
        Helpers::writeToFile("/home/u_private/ros_develop/frontier_navigation/files/filter.txt", "sorting ops", temp.size() * (log(temp.size())/log(2)));
//        printf("Important indices before cut: %d\n", temp.size());
//        printf("Important Indices: %d\n", importantIdxs.size());
        temp.clear();
//        printf("Flip cylces: %d\n", flipCycles);
//        printf("Compares: %d\n", compares);
//        printf("Neighbour look ups: %d\n", neighbourLookUps);
    }

    Helpers::writeToFile("/home/u_private/ros_develop/frontier_navigation/files/filter.txt", "Total count of important indices", cntOfImportantIdxs);
    Helpers::writeToFile("/home/u_private/ros_develop/frontier_navigation/files/filter.txt", "Total count of flip cycles", flipCycles);
    Helpers::writeToFile("/home/u_private/ros_develop/frontier_navigation/files/filter.txt", "Average size of importantIdxs", cntOfImportantIdxs/flipCycles);
    Helpers::writeToFile("/home/u_private/ros_develop/frontier_navigation/files/filter.txt", "Additional sorting ops", sortingOperations);

    for (int i = 0; i < importantIdxs.size(); i++) min4.cells.push_back(Helpers::gridToPoint(importantIdxs[i], map_));

    zeros.cell_height = zeros.cell_width = map_->info.resolution;
    min1.cell_height = min1.cell_width = map_->info.resolution;
    min2.cell_height = min2.cell_width = map_->info.resolution;
    min3.cell_height = min3.cell_width = map_->info.resolution;
    min4.cell_height = min4.cell_width = map_->info.resolution;
    zeros.header.frame_id = min1.header.frame_id = min2.header.frame_id = min3.header.frame_id = min4.header.frame_id = "/map";
    filteredMap->header = map_->header;
    filteredMap->info = map_->info;

    printf("Flip cylces: %d\n", flipCycles);
    printf("Compares: %d\n", compares);
    printf("Neighbour look ups: %d\n", neighbourLookUps);
    printf("Sorting ops: %d\n", sortingOperations);

    this->zeros_pub_.publish(zeros);
    this->min1_pub_.publish(min1);
    this->min2_pub_.publish(min2);
    this->min3_pub_.publish(min3);
    this->min4_pub_.publish(min4);
    this->map_2_pub_.publish(filteredMap);
}
