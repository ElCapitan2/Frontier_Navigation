#include "neighbours.h"
#include "nav_msgs/OccupancyGrid.h"
#include <stdio.h>

Neighbours::Neighbours(int width, int height)
{
    this->width = width;
    this->height = height;
    this->maxIndex = height*width-1;
}

int8_t Neighbours::getValLeft(int index, const nav_msgs::OccupancyGrid::ConstPtr& map) {
    int left = getLeft(index);
    if (left != -1) return map->data[left];
    else return -1;
}

int8_t Neighbours::getValRight(int index, const nav_msgs::OccupancyGrid::ConstPtr& map) {
    int right = getRight(index);
    if (right != -1) return map->data[right];
    else return -1;
}

int8_t Neighbours::getValTop(int index, const nav_msgs::OccupancyGrid::ConstPtr& map) {
    int top = getTop(index);
    if (top != -1) return map->data[top];
    else return -1;
}

int8_t Neighbours::getValBottom(int index, const nav_msgs::OccupancyGrid::ConstPtr& map) {
    int bottom = getBottom(index);
    if (bottom != -1) return map->data[bottom];
    else return -1;
}

int8_t Neighbours::getValLeftTop(int index, const nav_msgs::OccupancyGrid::ConstPtr& map) {
    int leftTop = getLeftTop(index);
    if (leftTop != -1) return map->data[leftTop];
    else return -1;
}

int8_t Neighbours::getValLeftBottom(int index, const nav_msgs::OccupancyGrid::ConstPtr& map) {
    int leftBottom = getLeftBottom(index);
    if (leftBottom != -1) return map->data[leftBottom];
    else return -1;
}

int8_t Neighbours::getValRightTop(int index, const nav_msgs::OccupancyGrid::ConstPtr& map) {
    int rightTop = getRightTop(index);
    if (rightTop != -1) return map->data[rightTop];
    else return -1;
}

int8_t Neighbours::getValRightBottom(int index, const nav_msgs::OccupancyGrid::ConstPtr& map) {
    int rightBottom = getRightBottom(index);
    if (rightBottom != -1) return map->data[rightBottom];
    else return -1;
}

int Neighbours::getLeft(int index)
{
    // check if index is within available range
    // makes use of lazy evaluation
    if (index < 0 || index > maxIndex) return -1;
    // check if index is left border of map
    if (index % width == 0) return -1;
    else return index-1;
}

int Neighbours::getRight(int index)
{
    if (index < 0 || index > maxIndex) return -1;
    // check if index is right border of map
    if (index % width == width-1) return -1;
    else return index+1;
}

// we start on left bottom
int Neighbours::getTop(int index)
{
    if (index < 0 || index > maxIndex) return -1;
    // check if index is top of map
    if ((index+=width) > maxIndex) return -1;
    else return index;
}

// we start on left bottom
int Neighbours::getBottom(int index)
{
    if (index < 0 || index > maxIndex) return -1;
    // check if index is bottom of map
    if ((index-=width) < 0) return -1;
    else return index;
}

int Neighbours::getLeftTop(int index)
{
    if (index < 0 || index > maxIndex) return -1;
    return getTop(getLeft(index));
}

int Neighbours::getRightTop(int index)
{
    if (index < 0 || index > maxIndex) return -1;
    return getTop(getRight(index));
}

int Neighbours::getLeftBottom(int index)
{
    if (index < 0 || index > maxIndex) return -1;
    return getBottom((getLeft(index)));
}

int Neighbours::getRightBottom(int index)
{
    if (index < 0 || index > maxIndex) return -1;
    return getBottom(getRight(index));
}

bool Neighbours::indexTest(bool printLog)
{
    bool result = true;
    int indices[9] = {0, width/2, width-1, (height/2)*width, (width/2)*(1+height), width-1+(height*width)/2, (height-1)*width, width*(height-0.5), width*height-1};
    int left[9] = {-1, indices[1]-1, indices[2]-1, -1, indices[4]-1, indices[5]-1, -1, indices[7]-1, indices[8]-1};
    int right[9] = {indices[0]+1, indices[1]+1, -1, indices[3]+1, indices[4]+1, -1, indices[6]+1, indices[7]+1, -1};
    int top[9] = {indices[0]+width, indices[1]+width, indices[2]+width, indices[3]+width, indices[4]+width, indices[5]+width, -1, -1, -1};
    int bottom[9] = {-1, -1, -1, indices[3]-width, indices[4]-width, indices[5]-width, indices[6]-width, indices[7]-width, indices[8]-width};
    int leftTop[9] = {-1, indices[1]-1+width, indices[2]-1+width, -1, indices[4]-1+width, indices[5]-1+width, -1, -1, -1};
    int leftBottom[9] = {-1, -1, -1, -1, indices[4]-1-width, indices[5]-1-width, -1, indices[7]-1-width, indices[8]-1-width};
    int rightTop[9] = {indices[0]+1+width, indices[1]+1+width, -1, indices[3]+1+width, indices[4]+1+width, -1, -1, -1, -1};
    int rightBottom[9] = {-1, -1, -1, indices[3]+1-width, indices[4]+1-width, -1, indices[6]+1-width, indices[7]+1-width, -1};
    for (int i = 0; i < 9; i++) {
        int idx = indices[i];
        if (printLog) {
            printf("Index: %d\n", idx);
            printf("Left\t\tTarget: %d\tActual: %d\tPassed: %d\n", left[i], getLeft(idx), getLeft(idx) == left[i]);
            printf("Right\t\tTarget: %d\tActual: %d\tPassed: %d\n", right[i], getRight(idx), getRight(idx) == right[i]);
            printf("Top\t\tTarget: %d\tActual: %d\tPassed: %d\n", top[i], getTop(idx), getTop(idx) == top[i]);
            printf("Bottom\t\tTarget: %d\tActual: %d\tPassed: %d\n", bottom[i], getBottom(idx), getBottom(idx) == bottom[i]);
            printf("LeftTop\t\tTarget: %d\tActual: %d\tPassed: %d\n", leftTop[i], getLeftTop(idx), getLeftTop(idx) == leftTop[i]);
            printf("RightTop\tTarget: %d\tActual: %d\tPassed: %d\n", rightTop[i], getRightTop(idx), getRightTop(idx) == rightTop[i]);
            printf("LeftBottom\tTarget: %d\tActual: %d\tPassed: %d\n", leftBottom[i], getLeftBottom(idx), getLeftBottom(idx) == leftBottom[i]);
            printf("RightBottom\tTarget: %d\tActual: %d\tPassed: %d\n\n", rightBottom[i], getRightBottom(idx), getRightBottom(idx) == rightBottom[i]);
        }
        if (getLeft(idx) != left[i]) result = false;
        if (getRight(idx) != right[i]) result = false;
        if (getTop(idx) != top[i]) result = false;
        if (getBottom(idx) != bottom[i]) result = false;
        if (getLeftTop(idx) != leftTop[i]) result = false;
        if (getRightTop(idx) != rightTop[i]) result = false;
        if (getLeftBottom(idx) != leftBottom[i]) result = false;
        if (getRightBottom(idx) != rightBottom[i]) result = false;
    }
    if (result) {
        printf("indexTest passed successfully\n");
    } else {
        printf("indexTest FAILED\n");
    }
    return result;
}
