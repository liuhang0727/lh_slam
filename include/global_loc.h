// 
// Created by liuhang on 20190528.
// 

#ifndef GLOBAL_LOC_H
#define GLOBAL_LOC_H

#include "common/common_headers.h"
#include "common/cloud_utils.h"
#include "common/ros_utils.h"
#include "common/transform_utils.h"
#include "plane.h"

class global_loc
{
    public:
    global_loc();
    ~global_loc();
    
    // set the params, subscribe and publish msgs
    bool init(ros::NodeHandle &node, ros::NodeHandle &private_node);


    private:

};

#endif // GLOBAL_LOC_H