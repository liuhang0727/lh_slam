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

    // the sub-thread of this node
    void spin();

    // the process of this node
    void process(); 

    // msgs handler
    inline void rgb_handler(const sensor_msgs::CompressedImage::ConstPtr &rgb_msg);
    inline void depth_handler(const sensor_msgs::CompressedImage::ConstPtr &depth_msg);
    inline void cloud_handler(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg);

    // ensure the all data are from the same frame, and each frame only be processed once
    inline bool if_new_data();

    // set the msgs flag to false, prapare for next frame
    inline void reset();

    // pre-process for kinect cloud: creat cloud, delete noise, pre-transform 
    inline void pre_process();

    // load faro map data
    void load_faro();

    // plane association between kinect and faro
    bool plane_match();

    // solve pose
    bool pose_solve(const std::vector<std::pair<Eigen::Vector4f, Eigen::Vector4f> > matches, Eigen::Matrix4f &T);

    // compute score of each transform
    double score(Eigen::Matrix4f T);



    private:
    std::thread _sub_thread;  // sub-thread
    std::mutex _mutex;  // thread mutex lock

    ros::Subscriber _sub_rgb;  // subscribe msgs
    ros::Subscriber _sub_depth;
    ros::Subscriber _sub_cloud; 
    ros::Publisher _pub_kinect_cloud;  // publish msgs
    ros::Publisher _pub_pk_cloud;
    ros::Publisher _pub_faro_cloud;
    ros::Time _rgb_time;  // time stamp 
    ros::Time _depth_time;
    ros::Time _cloud_time;

    cv::Mat _rgb_image;  //image
    cv::Mat _depth_image;

    CloudA::Ptr _kinect_cloud;  // cloud
    CloudA::Ptr _pk_cloud;
    CloudA::Ptr _faro_cloud;
    CloudA::Ptr _pf_cloud;

    bool _if_rgb;  // the flag of whether a new msg received
    bool _if_depth;
    bool _if_cloud;

    Eigen::Vector4f _cam_params;  // cam params

    pcl::KdTreeFLANN<PointA> _faro_tree;  // kd-tree of faro cloud

    std::unordered_map<int, std::vector<Eigen::Vector4f> > _kinect_params;  // plane params
    std::unordered_map<int, std::vector<Eigen::Vector4f> > _faro_params;

    // params
    int _skip_count;  // skip some frame to reduce the compution
    long _msgs_index; // the index of total msgs 
    std::string _cam_params_path;  // path of cam params
    std::string _faro_path;  // path of faro map

};

#endif // GLOBAL_LOC_H