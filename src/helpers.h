#ifndef HELPERS_H
#define HELPERS_H

#include <stdio.h>
#include <math.h>
#include <float.h>
#include <algorithm>
#include <iostream>
#include <fstream>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>

#include "types.h"
#include "map_operations.h"

class Helpers {
public:

    // interpolate linearly between two given points
    static double linearInterpolation(double start_x, double start_y, double end_x, double end_y, double x, bool print = false);

    // comparisons
    static bool areVecsEqual(geometry_msgs::Vector3 a, geometry_msgs::Vector3 b, bool print = false);

    // distance in point-space and index-space
    static double distance(unsigned int startIndex, unsigned int endIndex, int width, double resolution = 1.0, bool print = false);
    static double distance(geometry_msgs::Point A, geometry_msgs::Point B, bool print = false);
    static double distance(geometry_msgs::PoseStamped A, geometry_msgs::PoseStamped B, bool print = false);
    static double distance(geometry_msgs::Point A, geometry_msgs::PoseStamped B, bool print = false);
    // length of vector given as vector or point
    static double length(geometry_msgs::Vector3 vector);
    static double length(double x, double y, double z);

    // conversion between point-space and index-space
    static int pointToGrid(geometry_msgs::Point point, int width, int height, double resolution, double x_org, double y_org, bool print = false);
    static geometry_msgs::Point gridToPoint(int index, int width, int height, double resolution, double x_org, double y_org, bool print = false);
    static int pointToGrid(geometry_msgs::Point point, const nav_msgs::OccupancyGrid::ConstPtr &map, bool print = false);
    static geometry_msgs::Point gridToPoint(int index, const nav_msgs::OccupancyGrid::ConstPtr &map, bool print = false);

    static void print(std::vector<std::vector<unsigned int> > toPrint);

    // find closest and furthermost point of frontier to given point
    static int closestPoint(vec_single &frontier, int robotPosIdx, int width, bool print = false);
    static int furthermostPoint(vec_single &frontier, int robotPosIdx, int width, bool print = false);

    // determine circle around point in point-space and index-space
    static nav_msgs::GridCells circle(geometry_msgs::Point pt, double radius, const nav_msgs::OccupancyGrid::ConstPtr &map, bool print = false);
    static nav_msgs::GridCells circle(int index, double radius, const nav_msgs::OccupancyGrid::ConstPtr &map, bool print = false);
//    static nav_msgs::GridCells circleArea(geometry_msgs::Point pt, double radius, const nav_msgs::OccupancyGrid::ConstPtr &map, bool print = false);
//    static nav_msgs::GridCells circleArea(int index, double radius, const nav_msgs::OccupancyGrid::ConstPtr &map, bool print = false);

    // determine angle between two vectors
    // radian
    static double angleInRadian(geometry_msgs::Vector3 vecA, geometry_msgs::Vector3 vecB, bool print = false);
    static double angleInRadian(geometry_msgs::Point vecA, geometry_msgs::Point vecB, bool print = false);
    static double angleInRadian(double aX, double aY, double aZ, double bX, double bZ, double bY, bool print = false);
    // degree
    static double angleInDegree(geometry_msgs::Vector3 vecA, geometry_msgs::Vector3 vecB, bool print = false);

    // misc
    static vec_single sortAndRemoveEquals(vec_single v);
    static void writeToFile(char *file, char *msg, int value);

    // printers
    static void printPoint(geometry_msgs::Point point, char *name, int precison = 4);

    static void writeToFile(char *file, char *msg);
    static char *getOrdinal(unsigned int number);
    static void writeToFile(char *file, char *msg, double value);
    static void writeToFile(char* file, char* msg, int value1, int value2);
    static void writeToFile(char* file, char* msg, int value1, int value2, int value3);
    static void writeToFile(char *file, char *msg, vec_single data);
    static std::vector<geometry_msgs::Point> circleArea(unsigned int index, double radius, const nav_msgs::OccupancyGrid::ConstPtr &map);
    static std::vector<geometry_msgs::Point> circleArea(geometry_msgs::Point &center, double radius, const nav_msgs::OccupancyGrid::ConstPtr &map);
};

class PreFilterMap_1 {
public:
    PreFilterMap_1(int mapCnt, double radius, const geometry_msgs::PoseStamped &center, int startCell, int iterations);
    void printLog(int filteredCells, int ops, int addOps, int kernels, int isXSpace);
    vec_single cntOfFilteredCellsPerCycle;
    vec_single cntOfOpsPerCycle;
    vec_single cntOfAdditionalOpsPerCycle;
    vec_single cntOfKernelsInCycle;
    vec_single cntOfIsXSpaceInCycle;
private:
    int mapCnt_;
    double radius_;
    geometry_msgs::PoseStamped center_;
    int startCell_;
    int iterations_;
    int filterCycles_;
    int filteredCells_;
};

class PreFilterMap_2 {
public:
    PreFilterMap_2(int mapCnt, double radius, const geometry_msgs::PoseStamped &center, int startCell, int iterations, bool FI);
    void printLog(int filteredCells, int ops, int addOps, int distances, int kernels, int isXSpace, int getXVal, int getXCell);
    bool isFI;
    vec_single cntOfImpIdxsPerCycle;
    vec_single cntOfFilteredCellsPerCycle;
    vec_single cntOfPotentialImpIdxs;
    vec_single cntOfOpsPerCycle;
    vec_single cntOfAdditionalOpsPerCycle;
    vec_single cntOfDistancesInCycle;
    vec_single cntOfKernelsInCycle;
    vec_single cntOfIsXSpaceInCycle;
    vec_single cntOfGetXCellInCycle;
    vec_single cntOfGetXValInCycle;
    vec_single cntOfUSpaceCells;
    vec_single cntOfOSpaceCells;
    vec_single cntOfFSpaceCells;

private:

    int mapCnt_;
    double radius_;
    geometry_msgs::PoseStamped center_;
    int startCell_;
    int iterations_;
    int filterCycles_;
    int filteredCells_;
};


#endif  /* HELPERS_H */
