#include "frontier_navigation.h"
#include "tf/transform_listener.h"
#include "neighbours.h"
#include <vector>

Frontier_Navigation::Frontier_Navigation(ros::NodeHandle* node_ptr)
{
    this->nodeHandle_ = node_ptr;
    this->rectangle_pub_ = this->nodeHandle_->advertise<nav_msgs::GridCells>("/searchRadius", 1, true);
    this->rawFrontiers_pub_ = this->nodeHandle_->advertise<nav_msgs::GridCells>("/rawFrontiers", 1, true);
//    this->filteredFrontiers_pub_ = this->nodeHandle_->advertise<nav_msgs::GridCells>("filteredFrontiers", 1, true);
    this->goal_pub_ = this->nodeHandle_->advertise<geometry_msgs::PoseStamped>("/sb_navigation/simple_goal", 1, true);

    node_ptr->param("/frontier_navigation/radius", radius_, 5.0);
    node_ptr->param("/frontier_navigation/attempts", attempts_, 4);
    node_ptr->param("/frontier_navigation/stepping", stepping_, 5.0);
    node_ptr->param("/frontier_navigation/threshold", threshold_, 250);
    node_ptr->param("/frontier_navigation/sleep", sleep_, 0);
    node_ptr->param("/frontier_navigation/minDistance", minDinstance_, 3.0);
    node_ptr->param("/frontier_navigation/timeout", timeout_, 5.0);
    node_ptr->param("/frontier_navigation/timeoutAttempts", timeoutAttempts_, 5);
}

void Frontier_Navigation::timerCallback(const ros::TimerEvent&) {
    ROS_WARN("Robot not moving!");
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "/map";
    goal.pose.position.x = this->robot_position_.x+2;
    goal.pose.position.y = this->robot_position_.y;
    this->goal_pub_.publish(goal);
}

void Frontier_Navigation::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr&  map) {

    ROS_INFO("Frontier_Navigation received map");
//    this->not_moving_timer_ = this->nodeHandle_->createTimer(ros::Duration(timeout_), &Frontier_Navigation::timerCallback, this, true);
//    ROS_INFO("Timer started to prevent dead locks");

    this->map_ = map;
    nav_msgs::GridCells indexedRawFrontiers;
    nav_msgs::GridCells rectangle;
//    nav_msgs::GridCells filteredFrontiers;

    double radius = radius_;
//    for (int i = 1; i <= attempts_; i++) {
        findConnectedIndexedFrontiersWithinRadius(radius);
//        this->rawFrontiers_pub_.publish(rawFrontiers);
        this->rectangle_pub_.publish(rectangle);
//        this->filteredFrontiers_pub_.publish(filteredFrontiers);



//        sleep(sleep_);
//        if (validateFrontiers(filteredFrontiers)) {
//            printf("Frontiers detected successfully. Waiting on next map update ...\n");
//            double minDistance = 99999999.9;
//            geometry_msgs::PoseStamped goal;
//            goal.header.frame_id = "/map";
//            for (int j = 0; j < filteredFrontiers.cells.size(); j++) {
//                double dist = distance(robot_position_, filteredFrontiers.cells[j]);
//                if (dist < minDistance && dist > minDinstance_) {
//                    minDistance = dist;
//                    goal.pose.position.x = filteredFrontiers.cells[j].x;
//                    goal.pose.position.y = filteredFrontiers.cells[j].y;
//                }
//            }
//            printf("x: %f\ty: %f\tdistance: %f\n", goal.pose.position.x, goal.pose.position.y, minDistance);
//            this->goal_pub_.publish(goal);
//            break;
//        }
//        else if (i == attempts_) {
//            printf("Frontier constraints couldn't be held. Robot is stuck (for now)\n");
//        } else {
//            radius += stepping_;
//            printf("Frontier constraints couldn't be held. Radius will be increased by %f to %f.\n", stepping_, radius);
//        }


//    }
}

void Frontier_Navigation::posCallback(const geometry_msgs::PoseStamped& robot_position) {
    this->robot_position_ = robot_position.pose.position;
}

void Frontier_Navigation::findConnectedIndexedFrontiersWithinRadius(int radius) {
    // 1. find raw frontier-indices themselves
    std::vector<unsigned int> indexedRawFrontiers = findIndexedRawFrontiersWithinRadius(radius);
    // 2. compute adjacency matrix out of found frontier-indices
    std::vector<std::vector<unsigned int> > adjacencyMatrixOfFrontiers = computeAdjacencyMatrixOfFrontiers(indexedRawFrontiers);
    // 3. find connected frontier components out of computed adjacency matrix
    std::vector<std::vector<unsigned int> > connectedIndexedFrontiers = findConnectedIndexedFrontiers(adjacencyMatrixOfFrontiers);
}

std::vector<unsigned int> Frontier_Navigation::findIndexedRawFrontiersWithinRadius(int radius)
{
    // Based on robots coordinate system
    // Make sure not to leave given coordinate system!

    geometry_msgs::Point pos = this->robot_position_;
    geometry_msgs::Point startPoint;
    startPoint.x = pos.x - radius;
    startPoint.y = pos.y - radius;
    startPoint.z = 0;

    int startIndex = pointToGrid(startPoint);

    Neighbours neighbours(map_->info.width, map_->info.height);
    int iterations = radius*2/map_->info.resolution;

    std::vector<unsigned int> indexedRawFrontiers;
    int8_t data = 0;
    int index;
    for (int i = 0; i < iterations; i++) {
        for (int j = 0; j < iterations; j++) {

            index = startIndex + j + i*map_->info.height;

            data = map_->data[index];
            // if free-space is next to unknown-space free-space-index will be added
            // to indexedRawFrontiers
            if (data == 0) {
                if (neighbours.getValLeft(index, map_) == -1 && validateFrontierPoint(index)) {
                    indexedRawFrontiers.push_back(index);
                }
                else if (neighbours.getValRight(index, map_) == -1 && validateFrontierPoint(index)) {
                    indexedRawFrontiers.push_back(index);
                }
                else if (neighbours.getValTop(index, map_) == -1 && validateFrontierPoint(index)) {
                    indexedRawFrontiers.push_back(index);
                }
                else if (neighbours.getValBottom(index, map_) == -1 && validateFrontierPoint(index)) {
                    indexedRawFrontiers.push_back(index);
                }
            }
        }
    }
    return indexedRawFrontiers;
}

std::vector<std::vector<unsigned int> > Frontier_Navigation::computeAdjacencyMatrixOfFrontiers(std::vector<unsigned int> &indexedRawFrontiers) {

    std::vector<std::vector<unsigned int> > adjacencyMatrixOfFrontiers;
    std::vector<unsigned int> neighbours;

    for (unsigned int position = 0; position < indexedRawFrontiers.size(); position++) {
        int frontierIndex = indexedRawFrontiers[position];
        neighbours.push_back(0);
        neighbours.push_back(frontierIndex);
        // bottom
        for (unsigned int i = 0; i < position; i++) {
            int currentElement = indexedRawFrontiers[i];
            if (currentElement == frontierIndex-4000-1) neighbours.push_back(currentElement);
            else if (currentElement == frontierIndex-4000) neighbours.push_back(currentElement);
            else if (currentElement == frontierIndex-4000+1) {neighbours.push_back(currentElement); break;}
        }
        // left
        if (indexedRawFrontiers[position-1] == frontierIndex-1) neighbours.push_back(indexedRawFrontiers[position-1]);
        // right
        if (indexedRawFrontiers[position+1] == frontierIndex+1) neighbours.push_back(indexedRawFrontiers[position+1]);
        // top
        for (unsigned int i = position+1; i < indexedRawFrontiers.size(); i++) {
            int currentElement = indexedRawFrontiers[i];
            if (currentElement == frontierIndex+4000-1) neighbours.push_back(currentElement);
            else if (currentElement == frontierIndex+4000) neighbours.push_back(currentElement);
            else if (currentElement == frontierIndex+4000+1) {neighbours.push_back(currentElement); break;}
        }
        adjacencyMatrixOfFrontiers.push_back(neighbours);
        neighbours.clear();
    }
    return adjacencyMatrixOfFrontiers;
}

std::vector<std::vector<unsigned int> > Frontier_Navigation::findConnectedIndexedFrontiers(std::vector<std::vector<unsigned int> > &adjacencyMatrixOfFrontiers) {
    std::vector<std::vector<unsigned int> > connectedIndexedFrontiers;
    std::vector<unsigned int> neighbours;
    int component = 1;
    for (unsigned int i = 0; i < adjacencyMatrixOfFrontiers.size(); i++) {
        if (adjacencyMatrixOfFrontiers[i][0] == 0) {
            recursivelyFindConnectedFrontiers(adjacencyMatrixOfFrontiers, neighbours, i, component);
            connectedIndexedFrontiers.push_back(neighbours);
            neighbours.clear();
            component++;
        }
    }
    return connectedIndexedFrontiers;
}

void Frontier_Navigation::recursivelyFindConnectedFrontiers(std::vector<std::vector<unsigned int> > &adjacencyMatrixOfFrontiers, std::vector<unsigned int> &neighbours, int index, int component) {
    // point not yet visited
    if (adjacencyMatrixOfFrontiers[index][0] == 0) {
        adjacencyMatrixOfFrontiers[index][0] = component;
        neighbours.push_back(adjacencyMatrixOfFrontiers[index][1]);
        for (unsigned int i = 2; i < adjacencyMatrixOfFrontiers[index].size(); i++) {
            int nextIndex = adjacencyMatrixOfFrontiers[index][i];
            for (unsigned int j = 0; j < adjacencyMatrixOfFrontiers.size(); j++) {
                if (adjacencyMatrixOfFrontiers[j][1] == nextIndex) {
                    recursivelyFindConnectedFrontiers(adjacencyMatrixOfFrontiers, neighbours, j, component);
                }
            }
        }
    }
}

geometry_msgs::Point Frontier_Navigation::nextGoal()
{
    geometry_msgs::Point point;
    point.x = point.y = point.z = 0;
    return point;
}

void Frontier_Navigation::publishOutlineOfSearchRectangle(int radius) {

    nav_msgs::GridCells rectangle;
    geometry_msgs::Point pos = this->robot_position_;
    geometry_msgs::Point startPoint;
    startPoint.x = pos.x - radius;
    startPoint.y = pos.y - radius;
    startPoint.z = 0;

    int startIndex = pointToGrid(startPoint);
    int iterations = radius*2/map_->info.resolution;
    for (int i = 0; i < iterations; i++) {
        rectangle.cells.push_back((gridToPoint(startIndex + i, map_)));
        rectangle.cells.push_back((gridToPoint(startIndex+iterations*map_->info.height + i, map_)));
        rectangle.cells.push_back(gridToPoint(startIndex + i*map_->info.height, map_));
        rectangle.cells.push_back(gridToPoint(startIndex + i*map_->info.height + iterations, map_));
    }
    rectangle.cell_height = rectangle.cell_width = this->map_->info.resolution;
    rectangle.header.frame_id = "/map";
    this->rectangle_pub_.publish(rectangle);
}



geometry_msgs::Point Frontier_Navigation::gridToPoint(int index, const nav_msgs::OccupancyGrid::ConstPtr& map) {

    geometry_msgs::Point point;
    double resolution = map->info.resolution;
    int height = map->info.height;
    int width = map->info.width;

    point.x = resolution * (index%width + 0.5 - width/2);
    point.y = resolution * (index/width + 0.5 - height/2);
    point.z = 0.0;

    return point;
}

int Frontier_Navigation::pointToGrid(geometry_msgs::Point point) {
    // find center of cell in which robot is located
    double x_cellCenter = map_->info.resolution * (floor(point.x/map_->info.resolution) + 0.5);
    double y_cellCenter = map_->info.resolution * (floor(point.y/map_->info.resolution) + 0.5);
    // calculate corresponding linear value
    // based on f(minX/minY) = 0
    return ((y_cellCenter)*map_->info.width + (x_cellCenter))/map_->info.resolution + (map_->info.height*map_->info.width - 1)/2.0;
}



double Frontier_Navigation::distance(geometry_msgs::Point A, geometry_msgs::Point B) {
    return sqrt(pow((A.x - B.x), 2) + pow((A.y - B.y), 2) + pow((A.z - B.z), 2));
}

double Frontier_Navigation::distanceToSetOfFrontiers(std::vector<geometry_msgs::Point> &frontierSet) {
    // simple approach to find distance form set of frontiers to robot
    // more advanced approaches might be necessaray and useful
    double dist = 0.0;
    for (unsigned int i = 0; i < frontierSet.size(); i++) {
        dist += distance(robot_position_, frontierSet[i]);
    }
    return dist / frontierSet.size();
}

double Frontier_Navigation::distanceToSetOfFrontiers(std::vector<unsigned int> &frontierSet) {
    return 0.0;
}


// Define constraints for every single found frontier point
// I.e. skip frontiers which are surounded by free-space (due to sensor resolution)
bool Frontier_Navigation::validateFrontierPoint(int index) {
    return true;
}

// Define constraints which are necassary for further processing of found frontiers
// I.e. set minimum amount of frontier points needed
bool Frontier_Navigation::validateFrontiers(nav_msgs::GridCells &frontiers) {
    return frontiers.cells.size() > 1000;
}



void Frontier_Navigation::print(std::vector<std::vector<unsigned int> > &adjacentMatrixOfFrontiers) {
    for (unsigned int i = 0; i < adjacentMatrixOfFrontiers.size(); i++) {
        for (unsigned int j = 0; j < adjacentMatrixOfFrontiers[i].size(); j++) {
            std::cout << adjacentMatrixOfFrontiers[i][j] << " - ";
        }
        printf("\n");
    }
    printf("\n");
}



void Frontier_Navigation::TEST_gridToPoint() {

}

void Frontier_Navigation::TEST_pointToGrid() {

    int height = 16;
    int width = 16;
    double resolution = 0.25;
    geometry_msgs::Point point;
    point.z = 0;
    int index_actual;

    double x_arr[] = {-0.75, -0.5, -0.75, -0.5, -0.625, -0.6, -0.6};
    double y_arr[] = {-1.0, -1.0, -0.75, -0.75, -0.875, -0.8, -1.0};
    int target[] = {69, 70, 85, 86, 69, 69, 69};

    for (int i = 0; i < 7; i++) {

        point.x = x_arr[i];
        point.y = y_arr[i];

        double x_cellCenter = resolution * (floor(point.x/resolution) + 0.5);
        double y_cellCenter = resolution * (floor(point.y/resolution) + 0.5);

        index_actual = ((y_cellCenter)*width + (x_cellCenter))/resolution + (height*width - 1)/2.0;

        printf("x: %f\t y: %f\t target: %d\t actual: %d\n", x_cellCenter, y_cellCenter, target[i], index_actual);

        point.x = resolution * (index_actual%width + 0.5 - width/2);
        point.y = resolution * (index_actual/width + 0.5 - height/2);

        printf("x: %f\t y: %f\n", point.x, point.y);

    }
}







