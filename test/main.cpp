#include <stdio.h>
#include <test.h>

int main(int argc, char** argv)
{
    Test test;
    test.test_angleInX();
    for (double i = 0.5; i < 2.5; i+=0.5) {
        test.test_circleArea(7998000, i);
    }
    return 0;
}
