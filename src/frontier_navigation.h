#include <vector>
#include <stdlib.h>

#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <nav_msgs/GridCells.h>
#include <actionlib_msgs/GoalStatus.h>
#include <tf/transform_listener.h>

#include "types.h"
#include "constants.h"
#include "map_operations.h"
#include "helpers.h"


enum processStates {INIT, PROCESSING_MAP_STARTED, PROCESSING_MAP_DONE};
enum strategies {NORMAL, NO_FRONTIER_REGIONS_FOUND, DRIVE_TO_GOAL_BEFORE_UPDATE, STUCK, GOAL_REJECTED, DUPLICATED_GOAL};

class Frontier_Navigation {

public:

    Frontier_Navigation(ros::NodeHandle* node_ptr);

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map);
    void posCallback(const geometry_msgs::PoseStamped& robot_position);
    void timerCallback(const ros::TimerEvent&);
    void cmdVelCallback(const geometry_msgs::Twist& cmd_vel);
    void goalStatusCallback(const actionlib_msgs::GoalStatus& goalStatus);

//    void preFilterMap_fi(int radius);
//    void preFilterMap_Fi(int radius);
//    void preFilterMap_FI(int radius);
//    void preFilterMap_FII(const geometry_msgs::PoseStamped &center, int radius);

    void check(std::vector<geometry_msgs::Point> test);


    void escapeStrategy(strategies strategy);
    bool evaluateGoal(geometry_msgs::PoseStamped &goal);
    bool evaluateWhitelist(geometry_msgs::PoseStamped &goal);
    bool evaluateFrontierRegion(vec_single &frontierRegion);
    void explore();
    bool findNextGoal(geometry_msgs::PoseStamped &center, double radius, geometry_msgs::PoseStamped &goal);
    void strategy_NO_FRONTIER_REGIONS_FOUND();
    void publishLists();
    bool evaluateBlackList(geometry_msgs::PoseStamped &goal);
    void clenupWhitelist();
    void clenupBlacklist();
    bool findWhiteListedGoal();

private:

//    void processMap(geometry_msgs::PoseStamped center);

//    // find frontierRegions
//    void findFrontierRegions(const geometry_msgs::PoseStamped &center, int radius, vec_double &frontiers, vec_double &adjacencyMatrixOfFrontiers);
//    std::vector<unsigned int> findFrontierCells(const geometry_msgs::PoseStamped &center, int radius);
//    vec_double computeAdjacencyMatrixOfFrontierCells(std::vector<unsigned int> &frontierCells);
//    vec_double findFrontierRegions(vec_double &adjacencyMatrixOfFrontierCells);
//    void recursivelyFindFrontierRegions(std::vector<std::vector<unsigned int> > &adjacencyMatrixOfFrontiers, std::vector<unsigned int> &neighbours, int index, int component);

    // quality of frontierRegions
    std::vector<int> determineBestFrontierRegions(vec_double &adjacencyMatrixOfFrontiers, vec_double &frontiers);
    std::vector<int> determineBestFrontierRegions(vec_double &frontierRegions);
    std::vector<double> computeQualityOfFrontierRegions(vec_double &adjacencyMatrixOfFrontiers, vec_double &frontiers);

    geometry_msgs::PoseStamped nextGoal(vec_single frontier);

    // publishers
    void publishFrontierPts(vec_double frontiers);
    void publishFrontierPts(vec_double frontiers, int best_frontier);
    void publishOutlineOfSearchRectangle(geometry_msgs::PoseStamped &center, int radius);
    void publishGoal(geometry_msgs::PoseStamped &goal, bool print = false);
    void publishCircle(int goalIndex);

    void preFilterMap(const geometry_msgs::PoseStamped &center, int radius);


//    Process_Machine processMachine;

    processStates processState_;
    strategies strategy_;

    MapOperations mapOps_;


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
    ros::Publisher filteredMap_pub_;
    ros::Publisher blackList_pub_;
    ros::Publisher whiteListedGoals_pub_;
    ros::Publisher whiteListedFrontierRegions_pub_;
    ros::Timer not_moving_timer_;
    geometry_msgs::PoseStamped robot_position_;
//    nav_msgs::OccupancyGrid::ConstPtr map_;
//    boost::shared_ptr<nav_msgs::OccupancyGrid> mapCopy_;
    boost::shared_ptr<nav_msgs::OccupancyGrid> map_;
//    nav_msgs::OccupancyGrid filteredMap_;
    geometry_msgs::PoseStamped activeGoal_;
    actionlib_msgs::GoalStatus goalStatus_;

    nav_msgs::GridCells pathTracker_;
    unsigned int pathCounter_;
    nav_msgs::GridCells goalTracker_;

    vec_double whiteListedFrontierRegions_;
    std::vector<geometry_msgs::PoseStamped> blackList_;
    std::vector<geometry_msgs::PoseStamped> whiteListedGoals_;
//    std::list<geometry_msgs::PoseStamped> whitelistedGoals_;
//    std::list<vec_single> whitelistedFrontierRegions_;


    unsigned int mapCallbackCnt_;
    int cmd_vel_cnt_;
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
    bool explore_;
    double removeWhitelistedGoalThreshold_;
    int duplicatedGoals_;

};
