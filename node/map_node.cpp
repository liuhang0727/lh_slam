//
// Created by liuhang on 20190528.
//

#include "map.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map");
    ros::NodeHandle node;
    ros::NodeHandle private_node("~");

    map m;

    if(m.init(node, private_node))
    {
        ros::spin();
    }
    
    return 0;
}