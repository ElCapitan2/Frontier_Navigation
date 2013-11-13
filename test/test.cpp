#include <test.h>
#include <stdio.h>
//#include <ros/ros.h>
#include <helpers.h>

Test::Test() {
    printf("instance set!\n");
    geometry_msgs::Vector3 a;
    geometry_msgs::Vector3 b;

    a.x = 2.3;
    a.y = 1.4;
    a.z = 2.1;

    b.x = 1.2;
    b.y = 0.1;
    b.z = 4.9;

//    Helpers::angleInDegree(a, b);

    double angleInDegree = Helpers::angleInDegree(a, b);
    double angleInRadian = Helpers::angleInRadian(a, b) * 180 / M_PI;


    printf("%f - %f\n", angleInDegree, angleInRadian);
}

void Test::hallo() {
    printf("HALLO neuer Mensch\n");
}
