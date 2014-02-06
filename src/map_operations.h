#ifndef MAP_OPERATIONS_H
#define MAP_OPERATIONS_H

#include <stdio.h>
#include <algorithm>
#include <math.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>

#include "types.h"
#include "helpers.h"
#include "constants.h"

class MapOperations {

public:

    // filter map
    void preFilterMap(boost::shared_ptr<nav_msgs::OccupancyGrid> &map, const geometry_msgs::PoseStamped &center, int radius);

    // find frontierRegions
    void findFrontierRegions(boost::shared_ptr<nav_msgs::OccupancyGrid> &map, const geometry_msgs::PoseStamped &center, int radius, vec_double &frontierRegions, vec_double &adjacencyMatrixOfFrontierCells);

    // conversions from cell to point and vice versa
    int pointToCell(geometry_msgs::Point point, int width, double resolution, double x_org, double y_org, bool print = false);
    geometry_msgs::Point cellToPoint(int index, int width, double resolution, double x_org, double y_org, bool print = false);
    int pointToCell(geometry_msgs::Point point, const nav_msgs::OccupancyGrid::ConstPtr &map, bool print = false);
    geometry_msgs::Point cellToPoint(int index, const nav_msgs::OccupancyGrid::ConstPtr &map, bool print = false);

    // neighbour stuff
    // ...

    // neighbouring cells
    unsigned int getLeftCell(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map);
    unsigned int getRightCell(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map);
    unsigned int getTopCell(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map);
    unsigned int getBottomCell(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map);
    unsigned int getLeftTopCell(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map);
    unsigned int getRightTopCell(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map);
    unsigned int getLeftBottomCell(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map);
    unsigned int getRightBottomCell(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map);

    // neighbouring points

    // neighbouring values
    int8_t getLeftVal(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map);
    int8_t getRightVal(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map);
    int8_t getTopVal(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map);
    int8_t getBottomVal(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map);
    int8_t getLeftTopVal(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map);
    int8_t getLeftBottomVal(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map);
    int8_t getRightTopVal(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map);
    int8_t getRightBottomVal(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map);

    // more specific neighbouring values
    bool isFSpace(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map);
    bool isUSpace(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map);
    bool isOSpace(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map);

    // 3x3 neighbourhood
    int neighbourhoodValue(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map);

    // misc
    void setupSearchArea(const geometry_msgs::PoseStamped &center, double radius, const nav_msgs::OccupancyGrid::ConstPtr &map, int &startCell, int &iterations);
    void setupSearchArea(const geometry_msgs::PoseStamped &center, double radius, const nav_msgs::OccupancyGrid::ConstPtr &map, geometry_msgs::Point &startPoint, int &iterations);


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

    // misc
    unsigned int computeStartCellOfRectangle(const geometry_msgs::PoseStamped &center, int radius, const nav_msgs::OccupancyGrid::ConstPtr &map);
    geometry_msgs::Point computeStartPointOfRectangle(const geometry_msgs::PoseStamped &center, int radius);


    boost::shared_ptr<nav_msgs::OccupancyGrid> map_;

};

#endif /*MAP_OPERATIONS_H*/
