#include <test.h>
#include <stdio.h>
//#include <ros/ros.h>
#include <helpers.h>
#include <exception>

Test::Test() {
    good_ = 0;
    bad_ = 0;
}

void printResultMessage(bool passed, const char* name, int indentationLevel = 0) {
    switch (indentationLevel) {
    case 0: {
        if (passed) printf("\t\e[1;32m-> %s passed\e[0m\n\n", name);
        else printf("\t\e[1;31m-> !!!!! %s FAILED !!!!!\e[0m\n\n", name);
        break;
    }
    case 1: {
        if (passed) printf("\t\t\e[1;32m-> %s passed\e[0m\n\n", name);
        else printf("\t\t\e[1;31m-> !!!!! %s FAILED !!!!!\e[0m\n\n", name);
        break;
    }
    case 2: {
        if (passed) printf("\t\t\e[1;32m-> %s passed\e[0m\n\n", name);
        else printf("\t\t\e[1;31m-> !!!!! %s FAILED !!!!!\e[0m\n\n", name);
        break;
    }
    case 3: {
        if (passed) printf("\t\t\e[1;32m-> %s passed\e[0m\n\n", name);
        else printf("\t\t\e[1;31m-> !!!!! %s FAILED !!!!!\e[0m\n\n", name);
        break;
    }
    default: {
        if (passed) printf("\t\t\e[1;32m-> %s passed\e[0m\n\n", name);
        else printf("\t\t\e[1;31m-> !!!!! %s FAILED !!!!!\e[0m\n\n", name);
    }
    }
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
void printIntro(const char* name, int indentationLevel = 0) {
    switch (indentationLevel) {
    case 0: printf("%s\n", name); break;
    case 1: printf("\t%s\n", name); break;
    case 2: printf("\t\t%s\n", name); break;
    case 3: printf("%\t\t\ts\n", name); break;
    default: printf("%s\n", name);
    }
}

void createOccupancyGrid(boost::shared_ptr<nav_msgs::OccupancyGrid> &occGrid, std::vector<double> &x, std::vector<double> &y) {
    occGrid->info.height = 10;
    occGrid->info.width = 6;
    occGrid->info.resolution = 0.5;
    occGrid->info.origin.position.x = -2.0;
    occGrid->info.origin.position.y = -1.0;
    for (unsigned int i = 0; i < occGrid->info.height * occGrid->info.width; i++) {
        switch (i % 3) {
        case 0: occGrid->data.push_back(F_SPACE); break;
        case 1: occGrid->data.push_back(U_SPACE); break;
        case 2: occGrid->data.push_back(O_SPACE); break;
        }
    }
    double xArr[6] = {-1.75, -1.25, -0.75, -0.25, 0.25, 0.75};
    for (int i = 0; i < 6; i++) x.push_back(xArr[i]);
    double yArr[10] = {-0.75, -0.25, 0.25, 0.75, 1.25, 1.75, 2.25, 2.75, 3.25, 3.75};
    for (int i = 0; i < 10; i++) y.push_back(yArr[i]);
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
    for (unsigned int i = 0; i < test.size()-1; i++) {
        if (test[i] > test[i+1]) result1 = false;
    }
    printResultMessage(result1, __func__);
    return result1;
}

bool Test::test_Map_Operations()
{
    printIntro(__func__);
    bool result;
    MapOperations mapOps;
    boost::shared_ptr<nav_msgs::OccupancyGrid> occGrid(new nav_msgs::OccupancyGrid);
    std::vector<double> x;
    std::vector<double> y;
    createOccupancyGrid(occGrid, x, y);
    bool t1 = test_cellToPoint(mapOps, occGrid, x, y);
    bool t2 = test_pointToCell(mapOps, occGrid, x, y);
    bool t3 = test_getXCell(mapOps, occGrid);
    bool t4 = test_getXValue(mapOps, occGrid);
    bool t5 = test_isXSpace(mapOps, occGrid);
    bool t6 = test_neighbourhoodValue(mapOps, occGrid);
    bool t7 = test_setupSearchArea(mapOps, occGrid);
    result = t1 && t2 && t3 && t4 && t5 && t6 && t7;
    printResultMessage(result, __func__);
    return result;
}

bool Test::test_cellToPoint(MapOperations &mapOps, boost::shared_ptr<nav_msgs::OccupancyGrid> &occGrid, std::vector<double> x, std::vector<double> y) {
    printIntro(__func__, 1);
    geometry_msgs::Point pt1;
    geometry_msgs::Point pt2;
    int index = 0;
    bool result = true;
    bool temp = true;
    for (unsigned int i = 0; i < occGrid->info.height; i++) {
        for (unsigned int j = 0; j < occGrid->info.width; j++) {
            index = j + i*occGrid->info.width;
            pt1 = mapOps.cellToPoint(index, occGrid);
            pt2 = mapOps.cellToPoint(index, occGrid->info.width, occGrid->info.resolution, occGrid->info.origin.position.x, occGrid->info.origin.position.y);
            if (pt1.x != pt2.x || pt1.x != x[j]) {
                temp = false;
                result = false;
            }
            printf("\t\tpt1.x: %f pt2.x: %f x: %f %s\n", pt1.x, pt2.x, x[j], resultMsg(temp));
            temp = true;
            if (pt1.y != pt2.y || pt1.y != y[i]) {
                temp = false;
                result = false;
            }
            printf("\t\tpt1.y: %f pt2.y: %f y: %f %s\n", pt1.y, pt2.y, y[i], resultMsg(temp));
            temp = true;
        }
    }
    printResultMessage(result, __func__, 1);
    return result;
}

bool Test::test_pointToCell(MapOperations &mapOps, boost::shared_ptr<nav_msgs::OccupancyGrid> &occGrid, std::vector<double> x, std::vector<double> y) {
    printIntro(__func__, 1);
    geometry_msgs::Point pt;
    pt.z = 0.0;
    unsigned int index1;
    unsigned int index2;
    bool result = true;
    bool temp = true;
    for (unsigned int i = 0; i < occGrid->info.height; i++) {
        for (unsigned int j = 0; j < occGrid->info.width; j++) {
            pt.x = x[j];
            pt.y = y[i];
            index1 = mapOps.pointToCell(pt, occGrid);
            index2 = mapOps.pointToCell(pt, occGrid->info.width, occGrid->info.resolution, occGrid->info.origin.position.x, occGrid->info.origin.position.y);
            if (index1 != index2 || index1 != j + i*occGrid->info.width) {
                temp = false;
                result = false;
            }
            printf("\t\tindex1: %d index2: %d index: %d %s\n", index1, index2, j + i*occGrid->info.width, resultMsg(temp));
            temp = true;
        }
    }
    printResultMessage(result, __func__, 1);
    return result;
}

bool Test::test_getXCell(MapOperations &mapOps, boost::shared_ptr<nav_msgs::OccupancyGrid> &occGrid)
{
    printIntro(__func__, 1);
    bool result = true;

    bool resultLeft = true;
    bool resultRight = true;
    bool resultTop = true;
    bool resultBottom = true;
    bool resultLeftTop = true;
    bool resultLeftBottom = true;
    bool resultRightTop = true;
    bool resultRightBottom = true;

    unsigned int left;
    unsigned int right;
    unsigned int top;
    unsigned int bottom;
    unsigned int leftTop;
    unsigned int leftBottom;
    unsigned int rightTop;
    unsigned int rightBottom;

    for (unsigned int row = 0; row < occGrid->info.height; row++) {
        for (unsigned int col = 0; col < occGrid->info.width; col++) {
            unsigned int index = col + row*occGrid->info.width;
            left = mapOps.getLeftCell(index, occGrid);
            right = mapOps.getRightCell(index, occGrid);
            top = mapOps.getTopCell(index, occGrid);
            bottom = mapOps.getBottomCell(index, occGrid);
            leftTop = mapOps.getLeftTopCell(index, occGrid);
            leftBottom = mapOps.getLeftBottomCell(index, occGrid);
            rightTop = mapOps.getRightTopCell(index, occGrid);
            rightBottom = mapOps.getRightBottomCell(index, occGrid);
            // left
            if (col == 0) {
                if (left != -1) resultLeft =  false;
            } else {
                if (left != index-1) resultLeft = false;
            }
            // right
            if (col == occGrid->info.width - 1) {
                if (right != -1) resultRight = false;
            } else {
                if (right != index+1) resultRight = false;
            }
            // top
            if (row == occGrid->info.height - 1) {
                if (top != -1) resultTop = false;
            } else {
                if (top != index + occGrid->info.width) resultTop = false;
            }
            // bottom
            if (row ==  0) {
                if (bottom != -1) resultBottom = false;
            } else {
                if (bottom != index - occGrid->info.width) resultBottom = false;
            }
            // left top
            if (col == 0 || row == occGrid->info.height - 1) {
                if (leftTop != -1) resultLeftTop = false;
            } else {
                if (leftTop != index + occGrid->info.width - 1) resultLeftTop = false;
            }
            // left bottom
            if (col == 0 || row == 0) {
                if (leftBottom != -1) resultLeftBottom = false;
            } else {
                if (leftBottom != index - occGrid->info.width - 1) resultLeftBottom = false;
            }
            // right top
            if (col == occGrid->info.width - 1 || row == occGrid->info.height - 1) {
                if (rightTop != -1) resultRightTop = false;
            } else {
                if (rightTop != index + occGrid->info.width + 1) resultRightTop = false;
            }
            // right bottom
            if (col == occGrid->info.width - 1 || row == 0) {
                if (rightBottom != -1) resultRightBottom = false;
            } else {
                if (rightBottom != index - occGrid->info.width + 1) resultRightBottom = false;
            }
        }
    }
    printf("\t\tLeft: %s\n", resultMsg(resultLeft));
    printf("\t\tRight: %s\n", resultMsg(resultRight));
    printf("\t\tTop: %s\n", resultMsg(resultTop));
    printf("\t\tBottom: %s\n", resultMsg(resultBottom));
    printf("\t\tLeftTop: %s\n", resultMsg(resultLeftTop));
    printf("\t\tLeftBottom: %s\n", resultMsg(resultLeftBottom));
    printf("\t\tRightTop: %s\n", resultMsg(resultRightTop));
    printf("\t\tRightBottom: %s\n", resultMsg(resultRightBottom));

    result = resultLeft && resultRight && resultTop && resultBottom && resultLeftTop &&
            resultLeftBottom && resultRightTop && resultRightBottom;

    printResultMessage(result, __func__, 1);
    return result;
}

bool Test::test_getXValue(MapOperations &mapOps, boost::shared_ptr<nav_msgs::OccupancyGrid> &occGrid)
{
    printIntro(__func__, 1);
    bool result;

    bool resultLeft = true;
    bool resultRight = true;
    bool resultTop = true;
    bool resultBottom = true;
    bool resultLeftTop = true;
    bool resultLeftBottom = true;
    bool resultRightTop = true;
    bool resultRightBottom = true;

    int8_t left;
    int8_t right;
    int8_t top;
    int8_t bottom;
    int8_t leftTop;
    int8_t leftBottom;
    int8_t rightTop;
    int8_t rightBottom;

    int8_t val[3] = {0, -1, 100};

    for (unsigned int row = 0; row < occGrid->info.height; row++) {
        for (unsigned int col = 0; col < occGrid->info.width; col++) {
            unsigned int index = col + row*occGrid->info.width;
            left = mapOps.getLeftVal(index, occGrid);
            right = mapOps.getRightVal(index, occGrid);
            top = mapOps.getTopVal(index, occGrid);
            bottom = mapOps.getBottomVal(index, occGrid);
            leftTop = mapOps.getLeftTopVal(index, occGrid);
            leftBottom = mapOps.getLeftBottomVal(index, occGrid);
            rightTop = mapOps.getRightTopVal(index, occGrid);
            rightBottom = mapOps.getRightBottomVal(index, occGrid);
            // left
            if (col == 0) {
                if (left != -1) resultLeft =  false;
            } else {
                if (left != val[mapOps.getLeftCell(index, occGrid) % 3]) resultLeft =  false;
            }
            // right
            if (col == occGrid->info.width - 1) {
                if (right != -1) resultRight = false;
            } else {
                if (right != val[mapOps.getRightCell(index, occGrid) % 3]) resultRight = false;
            }
            // top
            if (row == occGrid->info.height - 1) {
                if (top != -1) resultTop = false;
            } else {
                if (top != val[mapOps.getTopCell(index, occGrid) % 3]) resultTop = false;
            }
            // bottom
            if (row ==  0) {
                if (bottom != -1) resultBottom = false;
            } else {
                if (bottom != val[mapOps.getBottomCell(index, occGrid) % 3]) resultBottom = false;
            }
            // left top
            if (col == 0 || row == occGrid->info.height - 1) {
                if (leftTop != -1) resultLeftTop = false;
            } else {
                if (leftTop != val[mapOps.getLeftTopCell(index, occGrid) % 3]) resultLeftTop = false;
            }
            // left bottom
            if (col == 0 || row == 0) {
                if (leftBottom != -1) resultLeftBottom = false;
            } else {
                if (leftBottom != val[mapOps.getLeftBottomCell(index, occGrid) % 3]) resultLeftBottom = false;
            }
            // right top
            if (col == occGrid->info.width - 1 || row == occGrid->info.height - 1) {
                if (rightTop != -1) resultRightTop = false;
            } else {
                if (rightTop != val[mapOps.getRightTopCell(index, occGrid) % 3]) resultRightTop = false;
            }
            // right bottom
            if (col == occGrid->info.width - 1 || row == 0) {
                if (rightBottom != -1) resultRightBottom = false;
            } else {
                if (rightBottom != val[mapOps.getRightBottomCell(index, occGrid) % 3]) resultRightBottom = false;
            }
        }
    }

    printf("\t\tLeft: %s\n", resultMsg(resultLeft));
    printf("\t\tRight: %s\n", resultMsg(resultRight));
    printf("\t\tTop: %s\n", resultMsg(resultTop));
    printf("\t\tBottom: %s\n", resultMsg(resultBottom));
    printf("\t\tLeftTop: %s\n", resultMsg(resultLeftTop));
    printf("\t\tLeftBottom: %s\n", resultMsg(resultLeftBottom));
    printf("\t\tRightTop: %s\n", resultMsg(resultRightTop));
    printf("\t\tRightBottom: %s\n", resultMsg(resultRightBottom));

    result = resultLeft && resultRight && resultTop && resultBottom && resultLeftTop &&
            resultLeftBottom && resultRightTop && resultRightBottom;

    printResultMessage(result, __func__, 1);
    return result;
}

bool Test::test_isXSpace(MapOperations &mapOps, boost::shared_ptr<nav_msgs::OccupancyGrid> &occGrid)
{
    printIntro(__func__, 1);

    bool fSpace = true;
    bool uSpace = true;
    bool oSpace = true;

    bool result;
    for (int i = 0; i < occGrid->data.size(); i++) {
        switch(i % 3) {
        case 0: if(!mapOps.isFSpace(i, occGrid)) fSpace = false; break;
        case 1: if(!mapOps.isUSpace(i, occGrid)) uSpace = false; break;
        case 2: if(!mapOps.isOSpace(i, occGrid)) oSpace = false; break;
        }
    }
    printf("\t\tfSpace: %s\n", resultMsg(fSpace));
    printf("\t\tuSpace: %s\n", resultMsg(uSpace));
    printf("\t\toSpace: %s\n", resultMsg(oSpace));
    result = fSpace && uSpace && oSpace;
    printResultMessage(result, __func__, 1);
    return result;
}

bool Test::test_neighbourhoodValue(MapOperations &mapOps, boost::shared_ptr<nav_msgs::OccupancyGrid> &occGrid)
{
    printIntro(__func__, 1);
    bool result = true;
    int value;
    for (unsigned int i = 0; i < occGrid->data.size(); i++) {
        value = 0;
        value += mapOps.getLeftVal(i, occGrid) + mapOps.getRightVal(i, occGrid);
        value += mapOps.getTopVal(i, occGrid) + mapOps.getBottomVal(i, occGrid);
        value += mapOps.getLeftTopVal(i, occGrid) + mapOps.getLeftBottomVal(i, occGrid);
        value += mapOps.getRightTopVal(i, occGrid) + mapOps.getRightBottomVal(i, occGrid);
        if (value != mapOps.neighbourhoodValue(i, occGrid)) result = false;
    }
    printResultMessage(result, __func__, 1);
    return result;
}

bool Test::test_setupSearchArea(MapOperations &mapOps, boost::shared_ptr<nav_msgs::OccupancyGrid> &occGrid)
{
    printIntro(__func__, 1);
    bool result = true;

    printf("\t\tw: %d, h: %d, r: %f ", occGrid->info.width, occGrid->info.height, occGrid->info.resolution);
    Helpers::printPoint(occGrid->info.origin.position, "zero", 2);
    printf("\t\t");
    geometry_msgs::PoseStamped center;
    center.pose.position = mapOps.cellToPoint(26, occGrid);
    Helpers::printPoint(center.pose.position, "center", 2);

    unsigned int startCell;
    int iterations;
    int idxs_1[] = {12,13,14,15,16,18,19,20,21,22,24,25,26,27,28,30,31,32,33,34,36,37,38,39,40};
    int idxs_2[] = {19, 20, 21, 25, 26, 27, 31, 32, 33};

    mapOps.setupSearchArea(center.pose.position, 1.0, occGrid, startCell, iterations);
    printf("\t\tradius: %f startCell: %d iterations:  %d\n", 1.0, startCell, iterations);
    int index;
    int cnt = 0;
    for (int i = 0; i < iterations; i++) {
        for (int j = 0; j < iterations; j++, cnt++) {
            index = startCell + j + i*occGrid->info.width;
            if (index != idxs_1[cnt]) result = false;
            printf("\t\ttarget: %d actual: %d\t%s\n", idxs_1[cnt], index, resultMsg(index == idxs_1[cnt]));
        }
    }
    mapOps.setupSearchArea(26, 1.0, occGrid, startCell, iterations);
    printf("\t\tradius: %f startCell: %d iterations:  %d\n", 1.0, startCell, iterations);
    cnt = 0;
    for (int i = 0; i < iterations; i++) {
        for (int j = 0; j < iterations; j++, cnt++) {
            index = startCell + j + i*occGrid->info.width;
            if (index != idxs_1[cnt]) result = false;
            printf("\t\ttarget: %d actual: %d\t%s\n", idxs_1[cnt], index, resultMsg(index == idxs_1[cnt]));
        }
    }

    mapOps.setupSearchArea(center.pose.position, 0.5, occGrid, startCell, iterations);
    printf("\t\tradius: %f startCell: %d iterations:  %d\n", 0.5, startCell, iterations);
    cnt = 0;
    for (int i = 0; i < iterations; i++) {
        for (int j = 0; j < iterations; j++, cnt++) {
            index = startCell + j + i*occGrid->info.width;
            if (index != idxs_2[cnt]) result = false;
            printf("\t\ttarget: %d actual: %d\t%s\n", idxs_2[cnt], index, resultMsg(index == idxs_2[cnt]));
        }
    }
    mapOps.setupSearchArea(26, 0.5, occGrid, startCell, iterations);
    printf("\t\tradius: %f startCell: %d iterations:  %d\n", 0.5, startCell, iterations);
    cnt = 0;
    for (int i = 0; i < iterations; i++) {
        for (int j = 0; j < iterations; j++, cnt++) {
            index = startCell + j + i*occGrid->info.width;
            if (index != idxs_2[cnt]) result = false;
            printf("\t\ttarget: %d actual: %d\t%s\n", idxs_2[cnt], index, resultMsg(index == idxs_2[cnt]));
        }
    }

    printResultMessage(result, __func__, 1);
    return result;
}

bool Test::test_getOrdinal()
{
    printIntro(__func__);
    bool result = true;
    bool tempResult;
    char* ordinals[32] = {"0th", "1st", "2nd", "3rd", "4th", "5th", "6th", "7th", "8th", "9th", "10th", "11th", "12th", "13th",
                          "14th", "15th", "16th", "17th", "18th", "19th", "20th", "21st", "22nd", "23rd", "24th", "25th", "26th",
                          "27th", "28th", "29th", "30th", "31st"};
    char* actual;
    for (unsigned int i = 0; i < 32; i++) {
        actual = Helpers::getOrdinal(i);
        if (strcmp(ordinals[i], actual) == 0) tempResult = true;
        else {
            tempResult = false;
            result = false;
        }
        printf("\tnumber: %d target: %s actual: %s\t%s\n", i, ordinals[i], actual, resultMsg(tempResult));

    }
    printResultMessage(result, __func__);
    return result;
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



