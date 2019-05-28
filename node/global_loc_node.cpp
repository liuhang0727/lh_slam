//
// Created by liuhang on 20190528.
//

#include "global_loc.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_loc");
    ros::NodeHandle node;
    ros::NodeHandle private_node("~");

    global_loc gl;

    if(gl.init(node, private_node))
    {
        ros::spin();
    }
    
    return 0;
}