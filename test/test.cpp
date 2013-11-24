#include <test.h>
#include <stdio.h>
//#include <ros/ros.h>
#include <helpers.h>

Test::Test() {
}

void printResultMessage(bool passed) {
    if (passed) printf("passed\n\n");
    else printf("FAILED!\n\n");
}

char* resultMsg(bool passed) {
    if (passed) return "passed";
    else return "FAILED!\n";
}

bool compareDoubles(double target, double actual) {
    double relativeError = fabs((actual - target) / target);
    if (relativeError <= 0.00001) return true;
    else return false;
}

bool Test::test_angleInX() {
    printf("test_angleInX\n");
    geometry_msgs::Vector3 a;
    geometry_msgs::Vector3 b;
    a.x = 2.3;
    a.y = 1.4;
    a.z = 2.1;
    b.x = 1.2;
    b.y = 0.1;
    b.z = 4.9;
    double actualAngleInDegree = Helpers::angleInDegree(a, b);
    double actualAngleInRadian = Helpers::angleInRadian(a, b);
    double targetAngleInDegree = 40.04518;
    double targetAngleInRadian = 0.698920;
    bool degree = compareDoubles(targetAngleInDegree, actualAngleInDegree);
    bool radian = compareDoubles(targetAngleInRadian, actualAngleInRadian);
    printf("\ta(%f, %f, %f); b(%f, %f, %f)\n", a.x, a.y, a.z, b.x, b.y, b.z);
    printf("\tangleInRadian: target: %f; actual: %f; %s\n", targetAngleInRadian, actualAngleInRadian, resultMsg(radian));
    printf("\tangleInDegree: target: %f; actual: %f; %s\n", targetAngleInDegree, actualAngleInDegree, resultMsg(degree));
    printf("\ttest_angleInX ");
    printResultMessage(degree & radian);
}

void Test::test_circleArea(int index, double radius) {
    nav_msgs::GridCells circle;
    double boxes = radius/0.05;
    int cnt = 0;
    for (int row = index-boxes*4000; row < index+boxes*4000; row+=4000) {
        for (int i = row-boxes; i < row+boxes; i++) {

            double distance = Helpers::distance(index, i, 4000, 0.05);
            if (distance < radius) {
                cnt++;
                circle.cells.push_back(Helpers::gridToPoint(i, 4000, 4000, 0.05));
            }
        }
    }
    printf("Radius: %f\tIterations: %d\tArea: %f\n", radius, cnt, pow(radius/0.05, 2)*M_PI);
}

