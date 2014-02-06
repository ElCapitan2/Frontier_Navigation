#include "frontier_navigation.h"

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

void Frontier_Navigation::publishGoal(geometry_msgs::PoseStamped &goal, bool print) {
    if (this->explore_) {
        this->activeGoal_ = goal;
        goalTracker_.cells.push_back(goal.pose.position);
        if (print) printf("\tNext Goal! goal(%f, %f, %f)\n", activeGoal_.pose.position.x, activeGoal_.pose.position.y, activeGoal_.pose.position.z);
        goal.header.frame_id = "/map";
        this->goal_pub_.publish(goal);
    }
}

//void Frontier_Navigation::publishOutlineOfSearchRectangle(geometry_msgs::PoseStamped &center, int radius) {
//    nav_msgs::GridCells rectangle;
//    int startCell;
//    int iterations;
//    Helpers::setupSearchArea(center, radius, this->map_, startCell, iterations);
//    int height = this->map_->info.height;
//    for (int i = 0; i < iterations; i++) {
//        rectangle.cells.push_back(Helpers::gridToPoint(startCell + i, map_));
//        rectangle.cells.push_back(Helpers::gridToPoint(startCell+iterations*height + i, map_));
//        rectangle.cells.push_back(Helpers::gridToPoint(startCell + i*height, map_));
//        rectangle.cells.push_back(Helpers::gridToPoint(startCell + i*height + iterations, map_));
//    }
//    rectangle.cell_height = rectangle.cell_width = this->map_->info.resolution;
//    rectangle.header.frame_id = this->map_->header.frame_id;
//    this->rectangle_pub_.publish(rectangle);
//}

void Frontier_Navigation::publishOutlineOfSearchRectangle(geometry_msgs::PoseStamped &center, int radius) {
    nav_msgs::GridCells rectangle;
    MapOperations mapOps;
    geometry_msgs::Point startPoint;
    geometry_msgs::Point add;
    add.z = 0;
    int iterations;
    mapOps.setupSearchArea(center, radius, this->map_, startPoint, iterations);
    int height = this->map_->info.height;
    double resolution = this->map_->info.resolution;
    for (int i = 0; i < iterations; i++) {
        add.x = startPoint.x + i*resolution;
        add.y = startPoint.y;
        rectangle.cells.push_back(add);
        add.x = startPoint.x + i*resolution;
        add.y = startPoint.y + 2*radius;
        rectangle.cells.push_back(add);
        add.x = startPoint.x;
        add.y = startPoint.y + i*resolution;
        rectangle.cells.push_back(add);
        add.x = startPoint.x + 2*radius;
        add.y = startPoint.y + i*resolution;
        rectangle.cells.push_back(add);
    }
    rectangle.cell_height = rectangle.cell_width = this->map_->info.resolution;
    rectangle.header.frame_id = this->map_->header.frame_id;
    this->rectangle_pub_.publish(rectangle);
}

void Frontier_Navigation::publishCircle(int goalIndex) {
    nav_msgs::GridCells circle = Helpers::circle(goalIndex, 1.5, this->map_);
    circle.cell_height = circle.cell_width = this->map_->info.resolution;
    circle.header.frame_id = "/map";
    this->circle_pub_.publish(circle);
}

void Frontier_Navigation::publishLists() {

    nav_msgs::GridCells blacklist;
    nav_msgs::GridCells whitelistedGoals;
    nav_msgs::GridCells whitelistedFrontierRegions;
    blacklist.header = whitelistedGoals.header = whitelistedFrontierRegions.header = this->map_->header;
    blacklist.cell_height = blacklist.cell_width = 0.05;
    whitelistedGoals.cell_height = whitelistedGoals.cell_width = 0.05;
    whitelistedFrontierRegions.cell_height = whitelistedFrontierRegions.cell_width = 0.05;
    MapOperations mapOps;

    for (int i = 0; i < this->blackList_.size(); i++) {
        blacklist.cells.push_back(blackList_[i].pose.position);
    }

    for (int i = 0; i < this->whiteListedFrontierRegions_.size(); i++) {
        whitelistedGoals.cells.push_back(whiteListedGoals_[i].pose.position);
        for (int j = 0; j < whiteListedFrontierRegions_[i].size(); j++) {
            whitelistedFrontierRegions.cells.push_back(mapOps.cellToPoint(whiteListedFrontierRegions_[i][j], map_));
        }
    }

    blackList_pub_.publish(blacklist);
    whiteListedGoals_pub_.publish(whitelistedGoals);
    whiteListedFrontierRegions_pub_.publish(whitelistedFrontierRegions);
}
