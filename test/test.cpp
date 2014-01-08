#include <test.h>
#include <stdio.h>
//#include <ros/ros.h>
#include <helpers.h>
#include <exception>

Test::Test() {
    good_ = 0;
    bad_ = 0;
}

void printResultMessage(bool passed, const char* name) {
    if (passed) printf("\t\e[1;32m-> %s passed\e[0m\n\n", name);
    else printf("\t\e[1;31m-> !!!!! %s FAILED !!!!!\e[0m\n\n", name);
}

char* resultMsg(bool passed) {
    if (passed) return "\e[1;32mpassed\e[0m";
    else return "\e[1;31mFAILED!\e[0m";
}

bool compareDoubles(double target, double actual) {
    double relativeError = fabs((actual - target) / target);
    if (relativeError <= 0.00001) return true;
    else return false;
}
void printIntro(const char* name) {
    printf("%s\n", name);
}

bool Test::test_linearInterpolation() {
    printIntro(__func__);
    double value;
    bool result;
    bool totalResult = true;
    for (double i = 0.5; i < 2.5; i+=0.5) {
        value = Helpers::linearInterpolation(0.0, 0.0, 1.0, i, 5.0);
        printf("\tA(%f, %f); B(%f, %f); C(%f, %c)\n", 0.0, 0.0, 1.0, i, 5.0, 'Y');
        result = compareDoubles(5.0*i, value);
        totalResult &= result ;
        printf("\ttarget: %f; actual: %f; %s\n", 5.0*i, value, resultMsg(result));
    }
    printResultMessage(totalResult, __func__);
    return totalResult;
}

bool Test::test_angleInX() {
    printIntro(__func__);
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
    printResultMessage(degree & radian, __func__);
    return degree & radian;
}

bool Test::test_printPoint() {
    printIntro(__func__);
    geometry_msgs::Point A;
    A.x = -1.2;
    A.y = 2.3;
    A.z = 0.003;
    for (int i = -1; i <= 7; i++) {
        printf("\t");
        Helpers::printPoint(A, "A", i);
    }
    printf("\t");
    Helpers::printPoint(A, "A");
    printResultMessage(true, __func__);
    return true;
}

bool Test::test_areVecsEqual() {
    printIntro(__func__);
    geometry_msgs::Vector3 a;
    geometry_msgs::Vector3 b;
    a.x = 1.2;
    a.y = 1.3;
    a.z = 0.0002;
    b.x = 1.2;
    b.y = 1.3;
    b.z = 0.0002;
    bool equal_1 = Helpers::areVecsEqual(a, b);
    b.z = 0.0001;
    bool equal_2 = Helpers::areVecsEqual(a, b);
    printf("\ta(%f, %f, %f); b(%f, %f, %f)\n", a.x, a.y, a.z, b.x, b.y, b.z);
    printf("\ttarget: 1; actual: %d; %s\n", equal_1, resultMsg(equal_1));
    printf("\ta(%f, %f, %f); b(%f, %f, %f)\n", a.x, a.y, a.z, b.x, b.y, b.z);
    printf("\ttarget: 0; actual: %d; %s\n", equal_2, resultMsg(!equal_2));
    printResultMessage(equal_1 && !equal_2, __func__);
    return equal_1 && !equal_2;
}

bool Test::test_linearInCoord()
{
    printIntro(__func__);
    geometry_msgs::Point pt = Helpers::gridToPoint(4352, 1600, 1184, 0.05, -50.8, -30.0);
    int index = Helpers::pointToGrid(pt, 1600, 1184, 0.05, -50.8, -30.0);

    printf("x: %f; y: %f; index: %d\n\n", pt.x, pt.y, index);
}

bool Test::test_sortAndRemoveEquals()
{
    printIntro(__func__);
    vec_single test;
    test.push_back(3);
    test.push_back(8);
    test.push_back(8);
    test.push_back(2);
    test.push_back(3);
    test = Helpers::sortAndRemoveEquals(test);
    bool result1 = true;
    for (int i = 0; i < test.size()-1; i++) {
        if (test[i] > test[i+1]) result1 = false;
    }
    printResultMessage(result1, __func__);
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
                circle.cells.push_back(Helpers::gridToPoint(i, 4000, 4000, 0.05, -100.0, -100.0));
            }
        }
    }
    printf("Radius: %f\tIterations: %d\tArea: %f\n", radius, cnt, pow(radius/0.05, 2)*M_PI);
}



