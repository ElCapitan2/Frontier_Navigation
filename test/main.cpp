#include <stdio.h>
#include <test.h>
#include <vector>
#include <algorithm>

//bool myfunction1 (int i,int j) { return (i>j);}
//bool myfunction2 (std::vector<int> vec1, std::vector<int> vec2) {return (vec1[1] < vec2[1]);}

int main(int argc, char** argv)
{
    int good = 0;
    int bad = 0;
    Test test;
    if (test.test_angleInX()) good++;
    else bad++;
    test.test_linearInterpolation();
    test.test_areVecsEqual();
    test.test_printPoint();

//    std::vector<std::vector<int> > vec;

//    std::vector<int> insert;
//    insert.push_back(0);
//    insert.push_back(4);
//    vec.push_back(insert);
//    insert.clear();
//    insert.push_back(1);
//    insert.push_back(3);
//    vec.push_back(insert);
//    insert.clear();
//    insert.push_back(2);
//    insert.push_back(10);
//    vec.push_back(insert);
//    insert.clear();
//    insert.push_back(3);
//    insert.push_back(7);
//    vec.push_back(insert);
//    insert.clear();



//    for (int i = 0; i < vec.size(); i++) {
//        printf("%d-%d\t", vec[i][0], vec[i][1]);
//    }

//    printf("\n");
//    std::sort(vec.begin(), vec.end(), myfunction2);

//    for (int i = 0; i < vec.size(); i++) {
//        printf("%d-%d\t", vec[i][0], vec[i][1]);
//    }
//    printf("\n");

    return 0;
}
