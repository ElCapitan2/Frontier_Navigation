#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include "types.h"

class Helpers {
public:
    static double linearInterpolation(double start_x, double start_y, double end_x, double end_y, double x, bool print = false);
    static double distance(unsigned int startIndex, unsigned int endIndex, int width, double resolution = 1.0, bool print = false);
    static double distance(geometry_msgs::Point A, geometry_msgs::Point B, bool print = false);
    static int pointToGrid(geometry_msgs::Point point, int width, int height, double resolution, bool print = false);
    static geometry_msgs::Point gridToPoint(int index, int width, int height, double resolution, bool print = false);
    static int pointToGrid(geometry_msgs::Point point, const nav_msgs::OccupancyGrid::ConstPtr &map, bool print = false);
    static geometry_msgs::Point gridToPoint(int index, const nav_msgs::OccupancyGrid::ConstPtr &map, bool print = false);
    static void print(std::vector<std::vector<unsigned int> > toPrint);
    static int closestPoint(vec_single &frontier, int robotPosIdx, int width, bool print = false);
};
