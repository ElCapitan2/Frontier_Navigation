#include <stdio.h>
#include <test.h>
#include <vector>
#include <algorithm>

int main(int argc, char** argv)
{
    int good = 0;
    int bad = 0;

    Test test;
    bool t1 = test.test_angleInX();
    bool t2 = test.test_linearInterpolation();
    bool t3 = test.test_areVecsEqual();
    bool t4 = test.test_sortAndRemoveEquals();
    bool t5 = test.test_printPoint();
    bool t6 = test.test_Map_Operations();
    bool t7 = test.test_getOrdinal();

    int success = t1 + t2 + t3 + t4 + t5 + t6 + t7;
    int tests = 7;

    if (success < tests) printf("\e[1;31m-> %d of %d tests passed\e[0m\n", success, tests);
    else printf("\e[1;32m-> %d of %d tests passed\e[0m\n", success, tests);

    return 0;
}
