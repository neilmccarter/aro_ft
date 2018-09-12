#include <iostream>
#include "vehicle_side.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_collect_data");
    ros::NodeHandle nh;
    CollectData collect_data(nh);
    ros::spin();
    return 0;
}
