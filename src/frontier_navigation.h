#include "nav_msgs/OccupancyGrid.h"
#include <tf/transform_listener.h>
#include "nav_msgs/GridCells.h"
#include "types.h"

class Frontier_Navigation {

public:

    Frontier_Navigation(ros::NodeHandle* node_ptr);

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map);
    void posCallback(const geometry_msgs::PoseStamped& robot_position);
    void timerCallback(const ros::TimerEvent&);

    void TEST_pointToGrid();
    void TEST_gridToPoint();

    void circle1(double radius);
    void circle2(double radius);
    void circle3(double radius);
    void circle4(double radius, geometry_msgs::Point pt);
private:

    void findAndPrepareFrontiersWithinRadius(int radius, vec_double &frontiers, vec_double &adjacencyMatrixOfFrontiers);
    std::vector<unsigned int> findFrontierIdxsWithinRadius(int radius);
    vec_double computeAdjacencyMatrixOfFrontiers(std::vector<unsigned int> &frontierIdxs);
    vec_double findFrontiers(vec_double &adjacencyMatrixOfFrontiers);
    void recursivelyFindFrontiers(std::vector<std::vector<unsigned int> > &adjacencyMatrixOfFrontiers, std::vector<unsigned int> &neighbours, int index, int component);

    int determineBestFrontier(vec_double &adjacencyMatrixOfFrontiers, vec_double &frontiers);
    std::vector<double> computeQualityOfFrontiers(vec_double &adjacencyMatrixOfFrontiers, vec_double &frontiers);

    geometry_msgs::PoseStamped nextGoal(vec_single frontier);

    void publishFrontierPts(vec_double frontiers);
    void publishFrontierPts(vec_double frontiers, int best_frontier);
    void publishOutlineOfSearchRectangle(int radius);
    void publishGoal(geometry_msgs::PoseStamped goal);

    void print(std::vector<std::vector<unsigned int> > &adjacentMatrixOfFrontiers);
    double distanceToSetOfFrontiers(std::vector<geometry_msgs::Point> &frontierSet);
    double distanceToSetOfFrontiers(std::vector<unsigned int> &frontierSet);
    double distance(geometry_msgs::Point A, geometry_msgs::Point B);

    int pointToGrid(geometry_msgs::Point point);
    geometry_msgs::Point gridToPoint(int index, const nav_msgs::OccupancyGrid::ConstPtr &map);

    bool frontierConstraints(vec_single &frontier);
    bool validateFrontierPoint(int index);

    ros::NodeHandle* nodeHandle_;
    ros::Publisher rectangle_pub_;
    ros::Publisher frontiers_pub_;
    ros::Publisher bestFrontier_pub_;
    ros::Publisher goal_pub_;
    ros::Publisher circle_pub_;
    ros::Timer not_moving_timer_;
    geometry_msgs::PoseStamped robot_position_;
    nav_msgs::OccupancyGrid::ConstPtr map_;

    double radius_;
    int attempts_;
    double stepping_;
    int threshold_;
    int sleep_;
    double minDinstance_;
    double timeout_;
    int timeoutAttempts_;
    double weightOfConnectivity_;
    double worstCaseOfConnectivity_;
    double weightOfSize_;
    double weightOfDistance_;
    double weightOfDirection_;

};
