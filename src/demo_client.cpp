#include "ros/ros.h"
#include "demo_test/pos_data.h"


void udpcallback(const demo_test::pos_data& pos)
{
    unsigned int num = pos.num;
    ROS_INFO("num is: %d", num);
    for(int i=0; i<num; i++)
    {
        ROS_INFO("rigidbody_%d: x=%f, y=%f, z=%f", i+1, pos.pos[i*3], pos.pos[i*3+1], pos.pos[i*3+2]);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "demo_client");
    ros::NodeHandle n;
    demo_test::pos_data pos;
    ros::Subscriber sub = n.subscribe("demo_udp", 1000, udpcallback);
    ros::spin();
    return 0;
}

