#include "nav_msgs/OccupancyGrid.h"
#include <tf/transform_listener.h>
#include "nav_msgs/GridCells.h"

class Frontier_Navigation {

public:

    Frontier_Navigation(ros::NodeHandle* node_ptr);

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map);
    void posCallback(const geometry_msgs::PoseStamped& robot_position);
    void timerCallback(const ros::TimerEvent&);

    void TEST_pointToGrid();
    void TEST_gridToPoint();

private:

    std::vector<std::vector<unsigned int> > findConnectedIndexedFrontiersWithinRadius(int radius);
    std::vector<unsigned int> findIndexedRawFrontiersWithinRadius(int radius);
    std::vector<std::vector<unsigned int> > computeAdjacencyMatrixOfFrontiers(std::vector<unsigned int> &indexedRawFrontiers);
    std::vector<std::vector<unsigned int> > findConnectedIndexedFrontiers(std::vector<std::vector<unsigned int> > &adjacencyMatrixOfFrontiers);
    void recursivelyFindConnectedFrontiers(std::vector<std::vector<unsigned int> > &adjacencyMatrixOfFrontiers, std::vector<unsigned int> &neighbours, int index, int component);
    geometry_msgs::PoseStamped nextGoal(std::vector<unsigned int> frontierSet);

    void publishOutlineOfSearchRectangle(int radius);
    void print(std::vector<std::vector<unsigned int> > &adjacentMatrixOfFrontiers);
    double distanceToSetOfFrontiers(std::vector<geometry_msgs::Point> &frontierSet);
    double distanceToSetOfFrontiers(std::vector<unsigned int> &frontierSet);
    double distance(geometry_msgs::Point A, geometry_msgs::Point B);

    int pointToGrid(geometry_msgs::Point point);
    geometry_msgs::Point gridToPoint(int index, const nav_msgs::OccupancyGrid::ConstPtr &map);

    bool validateFrontiers(std::vector<std::vector<unsigned int> > &connectedIndexedFrontiers);
    bool validateFrontierPoint(int index);

    ros::NodeHandle* nodeHandle_;
    ros::Publisher rectangle_pub_;
    ros::Publisher rawFrontiers_pub_;
    ros::Publisher filteredFrontiers_pub_;
    ros::Publisher goal_pub_;
    ros::Timer not_moving_timer_;
    geometry_msgs::Point robot_position_;
    nav_msgs::OccupancyGrid::ConstPtr map_;

    double radius_;
    int attempts_;
    double stepping_;
    int threshold_;
    int sleep_;
    double minDinstance_;
    double timeout_;
    int timeoutAttempts_;
    double worstCase_;

};
