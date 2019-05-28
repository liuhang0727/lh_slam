// 
// Created by liuhang on 20190528.
// 

#ifndef ODOM_H
#define ODOM_H

#include "common/common_headers.h"
#include "common/cloud_utils.h"
#include "common/ros_utils.h"
#include "common/transform_utils.h"
#include "plane.h"

class odom
{
    public:
    odom();
    ~odom();
    
    // set the params, subscribe and publish msgs
    bool init(ros::NodeHandle &node, ros::NodeHandle &private_node);


    private:

};

#endif // ODOM_H