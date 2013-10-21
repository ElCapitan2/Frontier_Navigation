#include "helpers.h"
#include <stdio.h>
#include <math.h>
#include <float.h>

double Helpers::linearInterpolation(double start_x, double start_y, double end_x, double end_y, double x, bool print)
{
    // f(x) = m*x+c
    // f(start_x) = start_y
    // f(end_x) = end_y
    // m = (end_y - start_y)/(end_x - start_x)
    // c = f(x) - m*x = start_y - m*start_x
    double m = (end_y - start_y)/(end_x - start_x);
    double c = (start_y - m*start_x);
    double result = m*x+c;
    if (print) printf("sx: %f\tsy: %f\tex: %f\tey: %f\tx: %f\tm: %f\tc: %f\tresult: %f\n", start_x, start_y, end_x, end_y, x, m, c, result);
    return result;
}

double Helpers::distance(unsigned int startIndex, unsigned int endIndex, int width, double resolution, bool print)
{
    int rowOfStartIndex = startIndex/width;
    int colOfStartIndex = startIndex - rowOfStartIndex*width;
    int rowOfEndIndex = endIndex/width;
    int colOfEndIndex = endIndex - rowOfEndIndex*width;
    return resolution * sqrt(pow((rowOfStartIndex-rowOfEndIndex), 2) + pow((colOfStartIndex-colOfEndIndex), 2));
}

double Helpers::distance(geometry_msgs::Point A, geometry_msgs::Point B, bool print) {
    // simplified since z=0
    return sqrt(pow((A.x - B.x), 2) + pow((A.y - B.y), 2));
}

int Helpers::closestPoint(vec_single &frontier, int robotPosIdx, int width, bool print) {
    int rowOfRobot = robotPosIdx/width;
    int colOfRobot = robotPosIdx - rowOfRobot*width;
    printf("%d - %d - %d - %d\n", rowOfRobot, colOfRobot, frontier.size(), robotPosIdx);
    double min = DBL_MAX;
    int index = 0;
    for (int i = 0; i < frontier.size(); i++) {
        int row = frontier[i]/width;
        int col = frontier[i] - row*width;
        double distance = sqrt(pow((rowOfRobot - row), 2) + pow((colOfRobot - col), 2));
        printf("%f\n", distance);
        if ((distance < min) && (distance > 25.0)) {
            min = distance;
            index = i;
        }
    }
    return frontier[index];
}

int Helpers::pointToGrid(geometry_msgs::Point point, int width, int height, double resolution, bool print) {
    // find center of cell in which robot is located
    double x_cellCenter = resolution * (floor(point.x/resolution) + 0.5);
    double y_cellCenter = resolution * (floor(point.y/resolution) + 0.5);
    // calculate corresponding linear value
    // based on f(minX/minY) = 0
    return ((y_cellCenter)*width + (x_cellCenter))/resolution + (height*width - 1)/2.0;
}
int Helpers::pointToGrid(geometry_msgs::Point point, const nav_msgs::OccupancyGrid::ConstPtr &map, bool print) {
    return pointToGrid(point, map->info.width, map->info.height, map->info.resolution, print);
}

geometry_msgs::Point Helpers::gridToPoint(int index, int width, int height, double resolution, bool print) {

    geometry_msgs::Point point;

    point.x = resolution * (index%width + 0.5 - width/2);
    point.y = resolution * (index/width + 0.5 - height/2);
    point.z = 0.0;

    if (print) printf("index: %d, width: %d, height: %d, resolution: %f, x: %f, y: %f\n", index, width, height, resolution, point.x, point.y);

    return point;
}
geometry_msgs::Point Helpers::gridToPoint(int index, const nav_msgs::OccupancyGrid::ConstPtr &map, bool print) {
    return gridToPoint(index, map->info.width, map->info.height, map->info.resolution, print);
}

void Helpers::print(std::vector<std::vector<unsigned int> > toPrint) {
    for (unsigned int i = 0; i < toPrint.size(); i++) {
        for (unsigned int j = 0; j < toPrint[i].size(); j++) {
            std::cout << toPrint[i][j] << " - ";
        }
        printf("\n");
    }
    printf("\n");
}
