#include "frontier_navigation.h"
#include "helpers.h"

void Frontier_Navigation::publishFrontierPts(vec_double frontiers) {
    nav_msgs::GridCells frontierPts;
    for (int i = 0; i < frontiers.size(); i++) {
        for (int j = 0; j < frontiers[i].size(); j++) {
            frontierPts.cells.push_back(Helpers::gridToPoint(frontiers[i][j], this->map_));
        }
    }
    frontierPts.cell_height = frontierPts.cell_width = this->map_->info.resolution;
    frontierPts.header.frame_id = "/map";
    this->frontiers_pub_.publish(frontierPts);
}

void Frontier_Navigation::publishFrontierPts(vec_double frontiers, int best_frontier) {
    nav_msgs::GridCells frontierPts;
    for (int i = 0; i < frontiers[best_frontier].size(); i++) {
        frontierPts.cells.push_back(Helpers::gridToPoint(frontiers[best_frontier][i], this->map_));
    }
    frontierPts.cell_height = frontierPts.cell_width = this->map_->info.resolution;
    frontierPts.header.frame_id = "/map";
    this->bestFrontier_pub_.publish(frontierPts);
}

void Frontier_Navigation::publishGoal(geometry_msgs::PoseStamped goal) {
    goal.header.frame_id = "/map";
    this->goal_pub_.publish(goal);
}

void Frontier_Navigation::publishOutlineOfSearchRectangle(int radius) {
    nav_msgs::GridCells rectangle;
    geometry_msgs::Point pos = this->robot_position_.pose.position;
    geometry_msgs::Point startPoint;
    startPoint.x = pos.x - radius;
    startPoint.y = pos.y - radius;
    startPoint.z = 0;

    int startIndex = Helpers::pointToGrid(startPoint, this->map_);
    int iterations = radius*2/map_->info.resolution;
    for (int i = 0; i < iterations; i++) {
        rectangle.cells.push_back(Helpers::gridToPoint(startIndex + i, map_));
        rectangle.cells.push_back(Helpers::gridToPoint(startIndex+iterations*map_->info.height + i, map_));
        rectangle.cells.push_back(Helpers::gridToPoint(startIndex + i*map_->info.height, map_));
        rectangle.cells.push_back(Helpers::gridToPoint(startIndex + i*map_->info.height + iterations, map_));
    }
    rectangle.cell_height = rectangle.cell_width = this->map_->info.resolution;
    rectangle.header.frame_id = "/map";
    this->rectangle_pub_.publish(rectangle);
}

void Frontier_Navigation::publishCircle(int goalIndex) {
    nav_msgs::GridCells circle = Helpers::circle(goalIndex, 1.5, this->map_, true);
    circle.cell_height = circle.cell_width = this->map_->info.resolution;
    circle.header.frame_id = "/map";
    this->circle_pub_.publish(circle);
}
