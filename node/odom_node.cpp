//
// Created by liuhang on 20190528.
//

#include "odom.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom");
    ros::NodeHandle node;
    ros::NodeHandle private_node("~");

    odom od;

    if(od.init(node, private_node))
    {
        ros::spin();
    }
    
    return 0;
}