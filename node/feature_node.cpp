//
// Created by liuhang on 20190404.
//

#include "feature.h"

int main(int argc, char** argv)
{
    // initModule_nonfree();

    ros::init(argc, argv, "feature");
    ros::NodeHandle node;
    ros::NodeHandle private_node("~");

    feature f;

    if(f.init(node, private_node))
    {
        ros::spin();
    }
    
    return 0;
}