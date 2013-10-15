#ifndef NEIGHBOURS_H
#define NEIGHBOURS_H

#include <stdint.h>
#include <nav_msgs/OccupancyGrid.h>

class Neighbours
{
public:

    Neighbours(int width, int heigth);

    int8_t getValLeft(int index, const nav_msgs::OccupancyGrid::ConstPtr &map);
    int8_t getValRight(int index, const nav_msgs::OccupancyGrid::ConstPtr &map);
    int8_t getValTop(int index, const nav_msgs::OccupancyGrid::ConstPtr &map);
    int8_t getValBottom(int index, const nav_msgs::OccupancyGrid::ConstPtr &map);
    int8_t getValLeftTop(int index, const nav_msgs::OccupancyGrid::ConstPtr &map);
    int8_t getValLeftBottom(int index, const nav_msgs::OccupancyGrid::ConstPtr &map);
    int8_t getValRightTop(int index, const nav_msgs::OccupancyGrid::ConstPtr &map);
    int8_t getValRightBottom(int index, const nav_msgs::OccupancyGrid::ConstPtr &map);

     bool indexTest(bool printLog);

private:

    int maxIndex;
    int height;
    int width;

    int getLeft(int index);
    int getRight(int index);
    int getTop(int index);
    int getBottom(int index);
    int getLeftTop(int index);
    int getRightTop(int index);
    int getLeftBottom(int index);
    int getRightBottom(int index);

};

#endif // NEIGHBOURS_H
