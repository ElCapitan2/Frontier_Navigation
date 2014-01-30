#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <stdio.h>
#include "types.h"




class MapOperations {

public:

    void preFilterMap(boost::shared_ptr<nav_msgs::OccupancyGrid> &map, const geometry_msgs::PoseStamped &center, int radius);
    void findFrontierRegions(boost::shared_ptr<nav_msgs::OccupancyGrid> &map, const geometry_msgs::PoseStamped &center, int radius, vec_double &frontierRegions, vec_double &adjacencyMatrixOfFrontierCells);

    int pointToCell(geometry_msgs::Point point, int width, double resolution, double x_org, double y_org, bool print = false);
    geometry_msgs::Point cellToPoint(int index, int width, double resolution, double x_org, double y_org, bool print = false);
    int pointToCell(geometry_msgs::Point point, const nav_msgs::OccupancyGrid::ConstPtr &map, bool print = false);
    geometry_msgs::Point cellToPoint(int index, const nav_msgs::OccupancyGrid::ConstPtr &map, bool print = false);

private:
    // filter map
    void preFilterMap_fi(int radius);
    void preFilterMap_Fi(int radius);
    void preFilterMap_FI(int radius);
    void preFilterMap_FII(const geometry_msgs::PoseStamped &center, int radius);

    // find frontierRegions
    std::vector<unsigned int> findFrontierCells(const geometry_msgs::PoseStamped &center, int radius);
    vec_double computeAdjacencyMatrixOfFrontierCells(std::vector<unsigned int> &frontierCells);
    vec_double computeFrontierRegions(vec_double &adjacencyMatrixOfFrontierCells);
    void recursivelyComputeFrontierRegions(std::vector<std::vector<unsigned int> > &adjacencyMatrixOfFrontiers, std::vector<unsigned int> &neighbours, int index, int component);

    boost::shared_ptr<nav_msgs::OccupancyGrid> map_;

};
