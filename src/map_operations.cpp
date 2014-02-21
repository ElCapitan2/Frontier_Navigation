#include "map_operations.h"

int maxIndex(const nav_msgs::OccupancyGrid::ConstPtr &map) {
    return map->info.height * map->info.width - 1;
}

bool isIndexWithinRange(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map) {
    if (index < 0 || index > maxIndex(map)) return false;
    else return true;
}

int MapOperations::pointToCell(geometry_msgs::Point point, int width, double resolution, double x_org, double y_org, bool print) {
    // find center of cell in which robot is located
    double x_cellCenter = resolution * (floor(point.x/resolution) + 0.5);
    double y_cellCenter = resolution * (floor(point.y/resolution) + 0.5);
    // calculate corresponding linear value
    // based on f(minX/minY) = 0
    // double index = ((y_cellCenter)*width + (x_cellCenter))/resolution + (height*width - 1)/2.0;
    // based on any translation
    double index = ((y_cellCenter - y_org)/resolution - 0.5) * width + (x_cellCenter - x_org)/resolution - 0.5;
    if (print) printf("index: %d, width: %d, resolution: %f, x: %f, y: %f\n", index, width, resolution, point.x, point.y);
    return static_cast<int>(ceil(index));
}

geometry_msgs::Point MapOperations::cellToPoint(int index, int width, double resolution, double x_org, double y_org, bool print){
    geometry_msgs::Point point;
    point.x = resolution * (index%width + 0.5) + x_org;
    point.y = resolution * (index/width + 0.5) + y_org;
//    point.x = resolution * (index%width + 0.5 - width/2);
//    point.y = resolution * (index/width + 0.5 - height/2);
    point.z = 0.0;
    if (print) printf("index: %d, width: %d, resolution: %f, x: %f, y: %f\n", index, width, resolution, point.x, point.y);
    return point;
}

int MapOperations::pointToCell(geometry_msgs::Point point, const nav_msgs::OccupancyGrid::ConstPtr &map, bool print) {
    return pointToCell(point, map->info.width, map->info.resolution, map->info.origin.position.x, map->info.origin.position.y, print);
}

geometry_msgs::Point MapOperations::cellToPoint(int index, const nav_msgs::OccupancyGrid::ConstPtr &map, bool print) {
    return cellToPoint(index, map->info.width, map->info.resolution, map->info.origin.position.x, map->info.origin.position.y, print);
}

unsigned int MapOperations::getLeftCell(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    // check if index is left border of map
    if (index % map->info.width == 0) return -1;
    // check if index is wihthin available range
    if (!isIndexWithinRange(index, map)) return -1;
    return index - 1;
}
unsigned int MapOperations::getRightCell(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    if (index % map->info.width == map->info.width - 1) return -1;
    if (!isIndexWithinRange(index, map)) return -1;
    return index + 1;
}
unsigned int MapOperations::getTopCell(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    if (index + map->info.width > maxIndex(map)) return -1;
    if (!isIndexWithinRange(index, map)) return -1;
    return index + map->info.width;
}
unsigned int MapOperations::getBottomCell(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    if (index >= 0 && index < map->info.width) return -1;
    if (!isIndexWithinRange(index, map)) return -1;
    return index - map->info.width;
}
unsigned int MapOperations::getLeftTopCell(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    return getTopCell(getLeftCell(index, map), map);
}
unsigned int MapOperations::getRightTopCell(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    return getTopCell(getRightCell(index, map), map);
}
unsigned int MapOperations::getLeftBottomCell(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    return getBottomCell(getLeftCell(index, map), map);
}
unsigned int MapOperations::getRightBottomCell(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    return getBottomCell(getRightCell(index, map), map);
}

int8_t MapOperations::getLeftVal(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    unsigned int left = getLeftCell(index, map);
    if (left == -1) return -1;
    else return map->data[left];
}

int8_t MapOperations::getRightVal(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    unsigned int right = getRightCell(index, map);
    if (right == -1) return -1;
    else return map->data[right];
}

int8_t MapOperations::getTopVal(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    unsigned int top = getTopCell(index, map);
    if (top == -1) return -1;
    else return map->data[top];
}

int8_t MapOperations::getBottomVal(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    unsigned int bottom = getBottomCell(index, map);
    if (bottom == -1) return -1;
    else return map->data[bottom];
}

int8_t MapOperations::getLeftTopVal(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    unsigned int leftTop = getLeftTopCell(index, map);
    if (leftTop == -1) return -1;
    else return map->data[leftTop];
}

int8_t MapOperations::getLeftBottomVal(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    unsigned int leftBottom = getLeftBottomCell(index, map);
    if (leftBottom == -1) return -1;
    else return map->data[leftBottom];
}

int8_t MapOperations::getRightTopVal(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    unsigned int rightTop = getRightTopCell(index, map);
    if (rightTop == -1) return -1;
    else return map->data[rightTop];
}

int8_t MapOperations::getRightBottomVal(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    unsigned int rightBottom = getRightBottomCell(index, map);
    if (rightBottom == -1) return -1;
    else return map->data[rightBottom];
}

bool MapOperations::isFSpace(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    if (map->data[index] == F_SPACE) return true;
    else return false;
}

bool MapOperations::isUSpace(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    if (map->data[index] == U_SPACE) return true;
    else return false;
}

bool MapOperations::isOSpace(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    if (map->data[index] == O_SPACE) return true;
    else return false;
}

int MapOperations::neighbourhoodValue(unsigned int index, const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    int value = 0;
    value += getLeftVal(index, map);
    value += getRightVal(index, map);
    value += getTopVal(index, map);
    value += getBottomVal(index, map);
    value += getLeftTopVal(index, map);
    value += getLeftBottomVal(index, map);
    value += getRightTopVal(index, map);
    value += getRightBottomVal(index, map);
    return value;
}

void MapOperations::setupSearchArea(const geometry_msgs::Point &center, double radius, const nav_msgs::OccupancyGrid::ConstPtr &map, unsigned int &startCell, int &iterations) {
    geometry_msgs::Point startPoint;
    startPoint.x = center.x - radius;
    startPoint.y = center.y - radius;
    startPoint.z = 0;
    int sc = pointToCell(startPoint, map);
    startCell = sc;
    iterations = radius*2/map->info.resolution + 1;
    Helpers::writeToFile("searchArea.xlsx", "centerPoint-startCell");
    Helpers::writeToFile("searchArea.xlsx", ";centerPoint.x", center.x);
    Helpers::writeToFile("searchArea.xlsx", ";centerPoint.y", center.y);
    Helpers::writeToFile("searchArea.xlsx", ";radius", radius);
    Helpers::writeToFile("searchArea.xlsx", ";startPoint.x", startPoint.x);
    Helpers::writeToFile("searchArea.xlsx", ";startPoint.y", startPoint.y);
    Helpers::writeToFile("searchArea.xlsx", ";startCell", sc);
    Helpers::writeToFile("searchArea.xlsx", ";iterations", iterations);
}
void MapOperations::setupSearchArea(const geometry_msgs::Point &center, double radius, const nav_msgs::OccupancyGrid::ConstPtr &map, geometry_msgs::Point &startPoint, int &iterations) {
    startPoint.x = center.x - radius;
    startPoint.y = center.y - radius;
    startPoint.z = 0;
    iterations = radius*2/map->info.resolution + 1;
    Helpers::writeToFile("searchArea.xlsx", "centerPoint-startPoint");
    Helpers::writeToFile("searchArea.xlsx", ";centerPoint.x", center.x);
    Helpers::writeToFile("searchArea.xlsx", ";centerPoint.y", center.y);
    Helpers::writeToFile("searchArea.xlsx", ";radius", radius);
    Helpers::writeToFile("searchArea.xlsx", ";startPoint.x", startPoint.x);
    Helpers::writeToFile("searchArea.xlsx", ";startPoint.y", startPoint.y);
    Helpers::writeToFile("searchArea.xlsx", ";iterations", iterations);

}

void MapOperations::setupSearchArea(unsigned int center, double radius, const nav_msgs::OccupancyGrid::ConstPtr &map, unsigned int &startCell, int &iterations) {
    double conversion = radius/map->info.resolution;
    int sc = center - conversion - map->info.width*conversion;
    startCell = sc;
    iterations = radius*2/map->info.resolution + 1;
    Helpers::writeToFile("searchArea.xlsx", "centerCell-startCell");
    Helpers::writeToFile("searchArea.xlsx", ";radius", radius);
    Helpers::writeToFile("searchArea.xlsx", ";conversion", conversion);
    Helpers::writeToFile("searchArea.xlsx", ";startCell", sc);
    Helpers::writeToFile("searchArea.xlsx", ";iterations", iterations);
}
void MapOperations::setupSearchArea(unsigned int center, double radius, const nav_msgs::OccupancyGrid::ConstPtr &map, geometry_msgs::Point &startPoint, int &iterations) {
    double conversion = radius/map->info.resolution;
    int startCell = center - conversion - map->info.width*conversion;
    startPoint = cellToPoint(startCell, map);
    iterations = radius*2/map->info.resolution + 1;
    Helpers::writeToFile("searchArea.xlsx", "centerCell-startPoint");
    Helpers::writeToFile("searchArea.xlsx", ";radius", radius);
    Helpers::writeToFile("searchArea.xlsx", ";conversion", conversion);
    Helpers::writeToFile("searchArea.xlsx", ";startCell", startCell);
    Helpers::writeToFile("searchArea.xlsx", ";startPoint.x", startPoint.x);
    Helpers::writeToFile("searchArea.xlsx", ";startPoint.y", startPoint.y);
    Helpers::writeToFile("searchArea.xlsx", ";iterations", iterations);
}


