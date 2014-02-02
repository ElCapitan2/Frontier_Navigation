#include "map_operations.h"

class Test {
public:
    Test();
    int good_;
    int bad_;
    bool test_linearInterpolation();
    bool test_angleInX();
    bool test_printPoint();
    bool test_areVecsEqual();
    bool test_sortAndRemoveEquals();

    // map_operations
    bool test_Map_Operations();
    bool test_cellToPoint(MapOperations &mapOps, boost::shared_ptr<nav_msgs::OccupancyGrid> &occGrid, std::vector<double> x, std::vector<double> y);
    bool test_pointToCell(MapOperations &mapOps, boost::shared_ptr<nav_msgs::OccupancyGrid> &occGrid, std::vector<double> x, std::vector<double> y);
    bool test_getXCell(MapOperations &mapOps, boost::shared_ptr<nav_msgs::OccupancyGrid> &occGrid);
    bool test_getXValue(MapOperations &mapOps, boost::shared_ptr<nav_msgs::OccupancyGrid> &occGrid);
    bool test_isXSpace(MapOperations &mapOps, boost::shared_ptr<nav_msgs::OccupancyGrid> &occGrid);
    bool test_neighbourhoodValue(MapOperations &mapOps, boost::shared_ptr<nav_msgs::OccupancyGrid> &occGrid);

    void test_circleArea(int index, double radius);

};
