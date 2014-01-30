#include <map_operations.h>

#include "helpers.h"
#include "neighbours.h"
#include <algorithm>
#include <math.h>
#include <constants.h>


int MapOperations::pointToCell(geometry_msgs::Point point, int width, double resolution, double x_org, double y_org, bool print) {
    // find center of cell in which robot is located
    double x_cellCenter = resolution * (floor(point.x/resolution) + 0.5);
    double y_cellCenter = resolution * (floor(point.y/resolution) + 0.5);
    // calculate corresponding linear value
    // based on f(minX/minY) = 0
    // double index = ((y_cellCenter)*width + (x_cellCenter))/resolution + (height*width - 1)/2.0;
    // based on any translation
    double index = ((y_cellCenter - y_org)/resolution - 0.5) * width + (x_cellCenter - x_org)/resolution - 0.5;
    if (print) printf("index: %d, width: %d, resolution: %f, x: %f, y: %f\n", index, width, resolution, point.x, point.y);
    return static_cast<int>(ceil(index));
}

geometry_msgs::Point MapOperations::cellToPoint(int index, int width, double resolution, double x_org, double y_org, bool print){
    geometry_msgs::Point point;
    point.x = resolution * (index%width + 0.5) + x_org;
    point.y = resolution * (index/width + 0.5) + y_org;
//    point.x = resolution * (index%width + 0.5 - width/2);
//    point.y = resolution * (index/width + 0.5 - height/2);
    point.z = 0.0;
    if (print) printf("index: %d, width: %d, resolution: %f, x: %f, y: %f\n", index, width, resolution, point.x, point.y);
    return point;
}

int MapOperations::pointToCell(geometry_msgs::Point point, const nav_msgs::OccupancyGrid::ConstPtr &map, bool print) {
    return pointToCell(point, map->info.width, map->info.resolution, map->info.origin.position.x, map->info.origin.position.y, print);
}

geometry_msgs::Point MapOperations::cellToPoint(int index, const nav_msgs::OccupancyGrid::ConstPtr &map, bool print) {
    return cellToPoint(index, map->info.width, map->info.resolution, map->info.origin.position.x, map->info.origin.position.y, print);
}
