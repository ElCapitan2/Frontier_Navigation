#include "frontier_navigation.h"
#include "ros/ros.h"

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "frontier_navigation");
    ros::NodeHandle nh;
    ros::NodeHandle* node_ptr = &nh;
    ros::Rate rate(10);

    Frontier_Navigation f_navigation(node_ptr);
    ros::Subscriber map = nh.subscribe("map", 1000, &Frontier_Navigation::mapCallback, &f_navigation);
    ros::Subscriber robot_position = nh.subscribe("sb_navigation/robot_position", 1000, &Frontier_Navigation::posCallback, &f_navigation);
    ros::Subscriber cmd_vel = nh.subscribe("cmd_vel", 1000, &Frontier_Navigation::cmdVelCallback, &f_navigation);
    ros::Subscriber goalStatus = nh.subscribe("sb_navigation/status", 1000, &Frontier_Navigation::goalStatusCallback, &f_navigation);
//    ros::Subscriber filteredMap = nh.subscribe("filteredMap", 1000, &Frontier_Navigation::fileredMapCallback, &f_navigation);

    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
