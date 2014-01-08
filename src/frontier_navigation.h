#include "nav_msgs/OccupancyGrid.h"
#include <tf/transform_listener.h>
#include "nav_msgs/GridCells.h"
#include "types.h"
#include "actionlib_msgs/GoalStatus.h"

class Frontier_Navigation {

public:

    Frontier_Navigation(ros::NodeHandle* node_ptr);

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map);
    void posCallback(const geometry_msgs::PoseStamped& robot_position);
    void timerCallback(const ros::TimerEvent&);
    void cmdVelCallback(const geometry_msgs::Twist& cmd_vel);
    void goalStatusCallback(const actionlib_msgs::GoalStatus& goalStatus);

private:

    void processMap();
    void findAndPrepareFrontiersWithinRadius(int radius, vec_double &frontiers, vec_double &adjacencyMatrixOfFrontiers);
    std::vector<unsigned int> findFrontierIdxsWithinRadius(int radius);
    vec_double computeAdjacencyMatrixOfFrontiers(std::vector<unsigned int> &frontierIdxs);
    vec_double findFrontiers(vec_double &adjacencyMatrixOfFrontiers);
    void recursivelyFindFrontiers(std::vector<std::vector<unsigned int> > &adjacencyMatrixOfFrontiers, std::vector<unsigned int> &neighbours, int index, int component);

    std::vector<int> determineBestFrontier(vec_double &adjacencyMatrixOfFrontiers, vec_double &frontiers);
    std::vector<double> computeQualityOfFrontiers(vec_double &adjacencyMatrixOfFrontiers, vec_double &frontiers);

    geometry_msgs::PoseStamped nextGoal(vec_single frontier);

    void publishFrontierPts(vec_double frontiers);
    void publishFrontierPts(vec_double frontiers, int best_frontier);
    void publishOutlineOfSearchRectangle(int radius);
    void publishGoal(geometry_msgs::PoseStamped goal);
    void publishCircle(int goalIndex);

    void preFilterMap(int radius);





    bool frontierConstraints(vec_single &frontier, bool print = false);
    bool cmdVelConstraints(const geometry_msgs::Twist& cmd_vel, bool print = false);

    ros::NodeHandle* nodeHandle_;
    ros::Publisher rectangle_pub_;
    ros::Publisher frontiers_pub_;
    ros::Publisher bestFrontier_pub_;
    ros::Publisher goal_pub_;
    ros::Publisher circle_pub_;
    ros::Publisher pathTracker_pub_;
    ros::Publisher zeros_pub_;
    ros::Publisher min1_pub_;
    ros::Publisher min2_pub_;
    ros::Publisher min3_pub_;
    ros::Publisher min4_pub_;
    ros::Publisher map_2_pub_;
    ros::Timer not_moving_timer_;
    geometry_msgs::PoseStamped robot_position_;
    nav_msgs::OccupancyGrid::ConstPtr map_;
    nav_msgs::OccupancyGrid filteredMap_;
    geometry_msgs::PoseStamped activeGoal_;
    actionlib_msgs::GoalStatus goalStatus_;

    nav_msgs::GridCells pathTracker_;
    unsigned int pathCounter_;
    nav_msgs::GridCells goalTracker_;
    nav_msgs::GridCells unusedFrontierTracker_;

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
