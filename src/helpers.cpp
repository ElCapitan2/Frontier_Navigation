#include "helpers.h"


double Helpers::linearInterpolation(double start_x, double start_y, double end_x, double end_y, double x, bool print)
{
    // f(x) = m*x+c
    // f(start_x) = start_y
    // f(end_x) = end_y
    // m = (end_y - start_y)/(end_x - start_x)
    // c = f(x) - m*x = start_y - m*start_x
    double m = (end_y - start_y)/(end_x - start_x);
    double c = (start_y - m*start_x);
    double result = m*x+c;
    if (print) printf("sx: %f\tsy: %f\tex: %f\tey: %f\tx: %f\tm: %f\tc: %f\tresult: %f\n", start_x, start_y, end_x, end_y, x, m, c, result);
    return result;
}

bool Helpers::areVecsEqual(geometry_msgs::Vector3 a, geometry_msgs::Vector3 b, bool print)
{
    double E = 0.00001;
    bool equal = (fabs(a.x - b.x) <= E) && (fabs(a.y - b.y) <= E) && (fabs(a.z - b.z) <= E);
    if (print) printf("");
    return equal;
}

double Helpers::distance(unsigned int startIndex, unsigned int endIndex, int width, double resolution, bool print)
{
    int rowOfStartIndex = startIndex/width;
    int colOfStartIndex = startIndex - rowOfStartIndex*width;
    int rowOfEndIndex = endIndex/width;
    int colOfEndIndex = endIndex - rowOfEndIndex*width;
    return resolution * sqrt(pow((rowOfStartIndex-rowOfEndIndex), 2) + pow((colOfStartIndex-colOfEndIndex), 2));
}

double Helpers::distance(geometry_msgs::Point A, geometry_msgs::Point B, bool print) {
    double distance = sqrt(pow((A.x - B.x), 2) + pow((A.y - B.y), 2) + pow((A.z - B.z), 2));
    if (print) {
        printf("Distance between A(%f/%f/%f) and B(%f/%f/%f): %f\n", A.x, A.y, A.z, B.x, B.y, B.z, distance);
    }
    return distance;
}

double Helpers::distance(geometry_msgs::PoseStamped A, geometry_msgs::PoseStamped B, bool print)
{
    return distance(A.pose.position, B.pose.position, print);
}

double Helpers::distance(geometry_msgs::Point A, geometry_msgs::PoseStamped B, bool print)
{
    return distance(A, B.pose.position, print);
}

double Helpers::length(geometry_msgs::Vector3 vector) {
    return length(vector.x, vector.y, vector.z);
}

double Helpers::length(double x, double y, double z) {
    return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
}

int Helpers::closestPoint(vec_single &frontier, int robotPosIdx, int width, bool print) {
    int rowOfRobot = robotPosIdx/width;
    int colOfRobot = robotPosIdx - rowOfRobot*width;
//    printf("%d - %d - %d - %d\n", rowOfRobot, colOfRobot, frontier.size(), robotPosIdx);
    double min = DBL_MAX;
    int index = 0;
    for (int i = 0; i < frontier.size(); i++) {
        int row = frontier[i]/width;
        int col = frontier[i] - row*width;
        double distance = sqrt(pow((rowOfRobot - row), 2) + pow((colOfRobot - col), 2));
//        printf("%f\n", distance);
        if ((distance < min) && (distance > 25.0)) {
            min = distance;
            index = i;
        }
    }
    return frontier[index];
}

int Helpers::furthermostPoint(vec_single &frontier, int robotPosIdx, int width, bool print) {
    int rowOfRobot = robotPosIdx/width;
    int colOfRobot = robotPosIdx - rowOfRobot*width;
//    printf("%d - %d - %d - %d\n", rowOfRobot, colOfRobot, frontier.size(), robotPosIdx);
    double max = 0.0;
    int index = 0;
    for (int i = 0; i < frontier.size(); i++) {
        int row = frontier[i]/width;
        int col = frontier[i] - row*width;
        double distance = sqrt(pow((rowOfRobot - row), 2) + pow((colOfRobot - col), 2));
//        printf("%f\n", distance);
        if (distance > max) {
            max = distance;
            index = i;
        }
    }
    return frontier[index];
}

int Helpers::pointToGrid(geometry_msgs::Point point, int width, int height, double resolution, double x_org, double y_org, bool print) {
    // find center of cell in which robot is located
    double x_cellCenter = resolution * (floor(point.x/resolution) + 0.5);
    double y_cellCenter = resolution * (floor(point.y/resolution) + 0.5);
    // calculate corresponding linear value
    // based on f(minX/minY) = 0
    // double index = ((y_cellCenter)*width + (x_cellCenter))/resolution + (height*width - 1)/2.0;
    // based on any translation
    double index = ((y_cellCenter - y_org)/resolution - 0.5) * width + (x_cellCenter - x_org)/resolution - 0.5;
    return static_cast<int>(ceil(index));
}
int Helpers::pointToGrid(geometry_msgs::Point point, const nav_msgs::OccupancyGrid::ConstPtr &map, bool print) {
    return pointToGrid(point, map->info.width, map->info.height, map->info.resolution, map->info.origin.position.x, map->info.origin.position.y, print);
}

geometry_msgs::Point Helpers::gridToPoint(int index, int width, int height, double resolution, double x_org, double y_org, bool print) {

    geometry_msgs::Point point;

    point.x = resolution * (index%width + 0.5) + x_org;
    point.y = resolution * (index/width + 0.5) + y_org;
//    point.x = resolution * (index%width + 0.5 - width/2);
//    point.y = resolution * (index/width + 0.5 - height/2);
    point.z = 0.0;

    if (print) printf("index: %d, width: %d, height: %d, resolution: %f, x: %f, y: %f\n", index, width, height, resolution, point.x, point.y);

    return point;
}
geometry_msgs::Point Helpers::gridToPoint(int index, const nav_msgs::OccupancyGrid::ConstPtr &map, bool print) {
    return gridToPoint(index, map->info.width, map->info.height, map->info.resolution, map->info.origin.position.x, map->info.origin.position.y, print);
}

nav_msgs::GridCells Helpers::circle(geometry_msgs::Point pt, double radius, const nav_msgs::OccupancyGrid::ConstPtr &map, bool print) {
    int index = Helpers::pointToGrid(pt, map);
    return circle(index, radius, map, print);
}

nav_msgs::GridCells Helpers::circle(int index, double radius, const nav_msgs::OccupancyGrid::ConstPtr &map, bool print) {
    nav_msgs::GridCells circle;
    double boxes = radius/map->info.resolution;
    int cnt = 0;
    int8_t free = 0;
    for (int row = index-boxes*map->info.width; row < index+boxes*map->info.width; row+=map->info.width) {
        for (int i = row-boxes; i < row+boxes; i++) {
            cnt++;
            double distance = Helpers::distance(index, i, map->info.width, map->info.resolution);
            if ((distance <= radius+0.05) && (distance >= radius-0.05) && (map->data[i] == free)) {
                circle.cells.push_back(Helpers::gridToPoint(i, map));
            }
        }
    }
    if (print) printf("circle - radius: %f, points: %d, cycles: %d\n", radius, circle.cells.size(), cnt);
    return circle;
}

std::vector<geometry_msgs::Point> Helpers::circleArea(geometry_msgs::Point &center, double radius, const nav_msgs::OccupancyGrid::ConstPtr &map) {
    MapOperations mapOps;
    mapOps.pointToCell(center, map);
    return circleArea(mapOps.pointToCell(center, map), radius, map);
}

std::vector<geometry_msgs::Point> Helpers::circleArea(unsigned int center, double radius, const nav_msgs::OccupancyGrid::ConstPtr &map) {
    MapOperations mapOps;
    std::vector<geometry_msgs::Point> circle;
    unsigned int startCell;
    int iterations;
    mapOps.setupSearchArea(center, radius, map, startCell, iterations);
    int cell;
    double dist;
    int start = 0;
    int end = 0;
    int inserts = 0;
    for (int i = 0; i < iterations; i++) {
        for (int j = 0; j < iterations; j++) {
            cell = startCell + j + i*map->info.width;
            dist = distance(center, cell, map->info.width, map->info.resolution);
            if (dist <= radius) {
                start = j;
                end = iterations-j;
                inserts = end-start;
                for (int k = 0; k <= inserts; k++) {
                    if (mapOps.isFSpace(cell, map)) circle.push_back(mapOps.cellToPoint(cell, map));
                    cell++;
                }
                break;
            }
        }
    }
    return circle;
}

double Helpers::angleInRadian(geometry_msgs::Vector3 vecA, geometry_msgs::Vector3 vecB, bool print) {
    return acos((vecA.x*vecB.x + vecA.y*vecB.y + vecA.z*vecB.z) / (Helpers::length(vecA)*Helpers::length(vecB)));
}

double Helpers::angleInDegree(geometry_msgs::Vector3 vecA, geometry_msgs::Vector3 vecB, bool print) {
    double angle = angleInRadian(vecA, vecB, print)/M_PI * 180;
    if (print) printf("angle between (%f/%f/%f) and (%f/%f/%f): %f°\n", vecA.x, vecA.y, vecA.z, vecB.x, vecB.y, vecB.z, angle);
    return angle;
}

vec_single Helpers::sortAndRemoveEquals(vec_single v)
{
    if (v.size() == 0) return v;
    // O(log(n)*n)
    std::sort(v.begin(), v.end());
    // O(n)
    vec_single sortedAndUnique;
    for (int i = 0; i < v.size() - 1; i++) {
        if (v[i] != v[i+1]) sortedAndUnique.push_back(v[i]);
    }
    sortedAndUnique.push_back(v[v.size()-1]);
    return sortedAndUnique;
}

void Helpers::print(std::vector<std::vector<unsigned int> > toPrint) {
    for (unsigned int i = 0; i < toPrint.size(); i++) {
        for (unsigned int j = 0; j < toPrint[i].size(); j++) {
            std::cout << toPrint[i][j] << " - ";
        }
        printf("\n");
    }
    printf("\n");
}

void Helpers::printPoint(geometry_msgs::Point point, char* name, int precision) {
    switch (precision) {
    case 0: printf("%s(%.0f/%.0f/%.0f)\n", name, point.x, point.y, point.z); break;
    case 1: printf("%s(%.1f/%.1f/%.1f)\n", name, point.x, point.y, point.z); break;
    case 2: printf("%s(%.2f/%.2f/%.2f)\n", name, point.x, point.y, point.z); break;
    case 3: printf("%s(%.3f/%.3f/%.3f)\n", name, point.x, point.y, point.z); break;
    case 4: printf("%s(%.4f/%.4f/%.4f)\n", name, point.x, point.y, point.z); break;
    case 5: printf("%s(%.5f/%.5f/%.5f)\n", name, point.x, point.y, point.z); break;
    case 6: printf("%s(%.6f/%.6f/%.6f)\n", name, point.x, point.y, point.z); break;
    default: printf("%s(%.4f/%.4f/%.4f)\n", name, point.x, point.y, point.z);
    }
}

void Helpers::writeToFile(char* file, char* msg, int value1, int value2) {

    std::stringstream strs;
    strs << "/home/u_private/ros_develop/frontier_navigation/logs/";
    strs << file;

    std::string temp_str = strs.str();
    char* char_type = (char*) temp_str.c_str();

    std::ofstream stream;
    stream.open(char_type, std::ios::app);
    if (stream.is_open()) {
        stream << msg << "; " << value1 << ";" << value2 << std::endl;
        stream.close();
    } else printf("File NOT open\n");
}

void Helpers::writeToFile(char* file, char* msg, int value1, int value2, int value3) {

    std::stringstream strs;
    strs << "/home/u_private/ros_develop/frontier_navigation/logs/";
    strs << file;

    std::string temp_str = strs.str();
    char* char_type = (char*) temp_str.c_str();

    std::ofstream stream;
    stream.open(char_type, std::ios::app);
    if (stream.is_open()) {
        stream << msg << "; " << value1 << ";" << value2 << ";" << value3 << std::endl;
        stream.close();
    } else printf("File NOT open\n");
}

void Helpers::writeToFile(char *file, char *msg, vec_single data) {

    std::stringstream strs;
    strs << "/home/u_private/ros_develop/frontier_navigation/logs/";
    strs << file;

    std::string temp_str = strs.str();
    char* char_type = (char*) temp_str.c_str();

    std::ofstream stream;
    stream.open(char_type, std::ios::app);
    if (stream.is_open()) {
        stream << msg;
        for (int i = 0; i < data.size(); i++) stream << ";" << data[i];
        stream << std::endl;
        stream.close();
    } else printf("File NOT open\n");
}

void Helpers::writeToFile(char* file, char* msg, int value) {

    std::stringstream strs;
    strs << "/home/u_private/ros_develop/frontier_navigation/logs/";
    strs << file;

    std::string temp_str = strs.str();
    char* char_type = (char*) temp_str.c_str();

    std::ofstream stream;
    stream.open(char_type, std::ios::app);
    if (stream.is_open()) {
        stream << msg << "; " << value << std::endl;
        stream.close();
    } else printf("File NOT open\n");
}

void Helpers::writeToFile(char* file, char* msg, double value) {

    std::stringstream strs;
    strs << "/home/u_private/ros_develop/frontier_navigation/logs/";
    strs << file;

    std::string temp_str = strs.str();
    char* char_type = (char*) temp_str.c_str();

    std::ofstream stream;
    stream.open(char_type, std::ios::app);
    if (stream.is_open()) {
        stream << msg << "; " << value << std::endl;
        stream.close();
    } else printf("File NOT open\n");
}

void Helpers::writeToFile(char *file, char *msg) {

    std::stringstream strs;
    strs << "/home/u_private/ros_develop/frontier_navigation/logs/";
    strs << file;

    std::string temp_str = strs.str();
    char* char_type = (char*) temp_str.c_str();

    std::ofstream stream;
    stream.open(char_type, std::ios::app);
    if (stream.is_open()) {
        stream << msg << std::endl;
        stream.close();
    } else printf("File NOT open\n");
}

char* Helpers::getOrdinal(unsigned int number) {

    std::stringstream strs;
    strs << number;

    if (number % 10 > 3) strs << "th";
    else if (number == 11 || number == 12 || number == 13) strs << "th";
    else if (number % 10 == 1) strs << "st";
    else if (number % 10 == 2) strs << "nd";
    else if (number % 10 == 3) strs << "rd";
    else strs << "th";

    std::string temp_str = strs.str();
    char* char_type = (char*) temp_str.c_str();

    return char_type;
}

PreFilterMap_1::PreFilterMap_1(int mapCnt, double radius, const geometry_msgs::PoseStamped &center, int startCell, int iterations) {
    this->mapCnt_ = mapCnt;
    this->radius_ = radius;
    this->center_ = center;
    this->startCell_ = startCell;
    this->iterations_ = iterations;
}
void PreFilterMap_1::printLog(int filteredCells, int ops, int addOps) {
    char* file = "filter.xlsx";
    int cycles = this->cntOfOpsPerCycle.size();
    Helpers::writeToFile(file, "Filtering map using fi...");
    Helpers::writeToFile(file, "mapCnt", this->mapCnt_);
    Helpers::writeToFile(file, "radius", this->radius_);
    Helpers::writeToFile(file, "center.x", this->center_.pose.position.x);
    Helpers::writeToFile(file, "center.y", this->center_.pose.position.y);
    Helpers::writeToFile(file, "startCell", this->startCell_);
    Helpers::writeToFile(file, "iterations", this->iterations_);
    Helpers::writeToFile(file, "cycles", cycles);
    Helpers::writeToFile(file, "filteredCells", filteredCells);
    Helpers::writeToFile(file, "ops", ops);
    Helpers::writeToFile(file, "addOps", addOps);
    Helpers::writeToFile(file, "totalOps", ops + addOps);
    Helpers::writeToFile(file, ";cycle;filteredCells;ops;addOps");
    vec_single data;
    for (int i = 0; i < this->cntOfOpsPerCycle.size(); i++) {
        data.push_back(i+1);
        data.push_back(this->cntOfFilteredCellsPerCycle[i]);
        data.push_back(this->cntOfOpsPerCycle[i]);
        data.push_back(this->cntOfAdditionalOpsPerCycle[i]);
        Helpers::writeToFile(file, "", data);
        data.clear();
    }
}

PreFilterMap_2::PreFilterMap_2(int mapCnt, double radius, const geometry_msgs::PoseStamped &center, int startCell, int iterations) {
    this->mapCnt_ = mapCnt;
    this->radius_ = radius;
    this->center_ = center;
    this->startCell_ = startCell;
    this->iterations_ = iterations;
}
void PreFilterMap_2::printLog(int filteredCells, int ops, int addOps) {
    char* file = "filter.xlsx";
    int cycles = this->cntOfImpIdxsPerCycle.size();
    Helpers::writeToFile(file, "Filtering map using FII...");
    Helpers::writeToFile(file, "mapCnt", this->mapCnt_);
    Helpers::writeToFile(file, "radius", this->radius_);
    Helpers::writeToFile(file, "center.x", this->center_.pose.position.x);
    Helpers::writeToFile(file, "center.y", this->center_.pose.position.y);
    Helpers::writeToFile(file, "startCell", this->startCell_);
    Helpers::writeToFile(file, "iterations", this->iterations_);
    Helpers::writeToFile(file, "cycles", cycles);
    Helpers::writeToFile(file, "filteredCells", filteredCells);
    Helpers::writeToFile(file, "ops", ops);
    Helpers::writeToFile(file, "addOps", addOps);
    Helpers::writeToFile(file, "totalOps", ops + addOps);
    Helpers::writeToFile(file, ";cycle;importantIdxs;filteredCells;potentials;ops;addOps;...");
    vec_single data;
    for (int i = 0; i < this->cntOfImpIdxsPerCycle.size(); i++) {
        data.push_back(i+1);
        data.push_back(this->cntOfImpIdxsPerCycle[i]);
        data.push_back(this->cntOfFilteredCellsPerCycle[i]);
        data.push_back(this->cntOfPotentialImpIdxs[i]);
        data.push_back(this->cntOfOpsPerCycle[i]);
        data.push_back(this->cntOfAdditionalOpsPerCycle[i]);
        data.push_back(0);
        Helpers::writeToFile(file, "", data);
        data.clear();
    }
}




