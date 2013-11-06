#include "/home/u_private/ros_develop/frontier_navigation/src/helpers.h"
#include <stdio.h>

void test_linearInterpolation() {
    double value = Helpers::linearInterpolation(1.0, 10.0, 2.0, 0.0, 1.2);
    double target = 8.0;
    printf("test_linearInterpolation\ttarget: %f\actual: %f\tpassed? %d", target, value, target==value);
}

void test_gridToPoint() {

}

void test_pointToGrid() {

    int height = 16;
    int width = 16;
    double resolution = 0.25;
    geometry_msgs::Point point;
    point.z = 0;
    int index_actual;

    double x_arr[] = {-0.75, -0.5, -0.75, -0.5, -0.625, -0.6, -0.6};
    double y_arr[] = {-1.0, -1.0, -0.75, -0.75, -0.875, -0.8, -1.0};
    int target[] = {69, 70, 85, 86, 69, 69, 69};

    for (int i = 0; i < 7; i++) {

        point.x = x_arr[i];
        point.y = y_arr[i];

        double x_cellCenter = resolution * (floor(point.x/resolution) + 0.5);
        double y_cellCenter = resolution * (floor(point.y/resolution) + 0.5);

        index_actual = ((y_cellCenter)*width + (x_cellCenter))/resolution + (height*width - 1)/2.0;

        printf("x: %f\t y: %f\t target: %d\t actual: %d\n", x_cellCenter, y_cellCenter, target[i], index_actual);

        point.x = resolution * (index_actual%width + 0.5 - width/2);
        point.y = resolution * (index_actual/width + 0.5 - height/2);

        printf("x: %f\t y: %f\n", point.x, point.y);
    }
}
