//#include "neighbours.h"
#include "frontier_navigation.h"
#include "stdio.h"
#include "ros/ros.h"
#include "tf/transform_listener.h"
//#include "costmap_2d/costmap_2d_ros.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "frontier_navigation");
    ros::NodeHandle nh;
    ros::NodeHandle* node_ptr = &nh;
    ros::Rate rate(10);

    Frontier_Navigation f_navigation(node_ptr);
    ros::Subscriber map = nh.subscribe("map", 1000, &Frontier_Navigation::mapCallback, &f_navigation);
    ros::Subscriber robot_position = nh.subscribe("sb_navigation/robot_position", 1000, &Frontier_Navigation::posCallback, &f_navigation);

//    f_navigation.TEST_pointToGrid();

    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

//    step 1:
//    get position of robot and translate it into index based coordinate system




//    neighbours n(4000, 4000);
//    n.indexTest(true);

//    ros::init(argc, argv, "frontier_navigation");
//    ros::NodeHandle nh;
//    ros::NodeHandle* node_ptr = &nh;
//    ros::Rate rate(10);
//    ros::Subscriber sub_map = nh.subscribe("map", 1000, mapCallback);

//    costmap_2d::Costmap2DROS* local_costmap;
////    costmap_2d::Costmap2DROS* global_costmap;

//    tf::TransformListener* tf;
//    tf = new tf::TransformListener(ros::Duration(10));

//    local_costmap = new costmap_2d::Costmap2DROS("local", *tf);
////    global_costmap = new costmap_2d::Costmap2DROS("global", *tf);

//    printf("fs: %d\n", costmap_2d::FREE_SPACE);
//    printf("lo: %d\n", costmap_2d::LETHAL_OBSTACLE);
//    printf("no: %d\n", costmap_2d::NO_INFORMATION);

//    printf("%f\n", local_costmap->getInflationRadius());
//    printf("%f\n", local_costmap->getInscribedRadius());
//    printf("%f\n", local_costmap->getResolution());
//    printf("%f\n", local_costmap->getSizeInCellsX());
//    printf("%f\n", local_costmap->getSizeInCellsY());
//    tf::Stamped<tf::Pose> robotPos;
//    local_costmap->getRobotPose(robotPos);
//    geometry_msgs::PoseStamped robotPos_stamped;
//    tf::poseStampedTFToMsg(robotPos, robotPos_stamped);
//    printf("%f\n", robotPos_stamped.pose.position.x);
//    printf("%f\n", robotPos_stamped.pose.position.y);

//    costmap_2d::Costmap2D cm;
//    local_costmap->getCostmapCopy(cm);
//    printf("index: %d\n", cm.getIndex(639, 1760)); //7040639
//    unsigned int x, y;
//    cm.indexToCells(7040639, x, y);
//    printf("%d - %d\n", x, y);
//    printf("%f - %f\n", cm.getOriginX(), cm.getOriginY());
//    printf("%f\n", cm.getSizeInMetersX());
//    printf("cost: %d\n", cm.getCost(639, 1760));
//    const unsigned char* a = cm.getCharMap();

//    int fs = 0;
//    int lo = 0;
//    int ni = 0;
//    int ka = 0;
//    int data = 0;
//    nav_msgs::GridCells f;
//    f.header.frame_id = "/map";
//    f.cell_height = 0.05;
//    f.cell_width = 0.05;
//    geometry_msgs::Point point;
//    point.z = 0;
//    ros::Publisher pub = nh.advertise<nav_msgs::GridCells>("versuch", 1, true);
//    for (int i = 0; i < 16000000; i++) {
//        data = a[i];
//        switch (data) {
//        case costmap_2d::FREE_SPACE: fs++; break;
//        case costmap_2d::LETHAL_OBSTACLE: lo++; break;
//        case costmap_2d::NO_INFORMATION: ni++; break;
//        default: {
//            point.x = (i%4000 - 2000 + 0.5)*0.05;
//            point.y = (i/4000 - 2000 + 0.5)*0.05;
//            f.cells.push_back(point);
//            ka++;
//        }
//        }
//    }
//    printf("fs: %d\tlo: %d\tni: %d\tka: %d\n", fs, lo, ni, ka);

//    ROS_INFO("frontier_navigation is running ...");

//    while(ros::ok()) {
//        pub.publish(f);
//        ros::spinOnce();
//        rate.sleep();
//    }

