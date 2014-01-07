#include "frontier_navigation.h"
#include "helpers.h"
#include "neighbours.h"

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

    int8_t data = 0;
    int index;
    for (int i = 0; i < iterations; i++) {
        for (int j = 0; j < iterations; j++) {
            index = startIndex + j + i*map_->info.height;
            data = map_->data[index];
            // if we find unknown-space we try to determine whether it is really unknown-space
            // or just a resolution error
            if (data == -1) {
                // we add up all surrounding values and exclude heuristically unwanted pixels
                int kernel = neighbours.getValLeft(index, map_) + neighbours.getValRight(index, map_) + neighbours.getValTop(index, map_) + neighbours.getValBottom(index, map_) +
                        neighbours.getValLeftBottom(index, map_) + neighbours.getValLeftTop(index, map_) + neighbours.getValRightBottom(index, map_) + neighbours.getValRightTop(index, map_);
                switch (kernel) {
                case 0: zeros.cells.push_back(Helpers::gridToPoint(index, map_)); break;
                case -1: min1.cells.push_back(Helpers::gridToPoint(index, map_)); break;
                case -2: min2.cells.push_back(Helpers::gridToPoint(index, map_)); break;
                case -3: min3.cells.push_back(Helpers::gridToPoint(index, map_)); break;
                case -4: min4.cells.push_back(Helpers::gridToPoint(index, map_)); break;
                default: break;
                }
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
}
