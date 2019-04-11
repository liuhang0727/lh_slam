//
// Created by liuhang on 20190404.
//

#ifndef FEATURE_H
#define FEATURE_H

#include "common_headers.h"

class feature
{
    public:
        feature();
        ~feature();
        
        //set the params, subscribe and publish msgs
        bool init(ros::NodeHandle &node, ros::NodeHandle &private_node);

        //the sub-thread of this node
        void spin();

        //the process of this node
        void process(); 

        //the recall handler of qhd_cloud msgs
        inline void qhd_cloud_handler(const sensor_msgs::PointCloud2::ConstPtr &qhd_cloud_msg);

        //the recall handler of qhd_rgb msgs
        inline void qhd_rgb_handler(const sensor_msgs::Image::ConstPtr &qhd_rgb_msg);

        //the recall handler of qhd_depth msgs
        inline void qhd_depth_handler(const sensor_msgs::Image::ConstPtr &qhd_depth_msg);

        //ensure the all data are from the same frame, and each frame only be processed once
        inline bool if_new_data();

        //set the msgs flag to false, prapare for next frame
        inline void reset();

        //search corresponding frame in rgb_image_map for cloud frame 
        inline bool data_match();

        //feature detect in rgb image
        void feature_detect();

        //publish cloud msg
        inline void publish_cloud_msg(ros::Publisher &publisher, 
                                      const CloudT &cloud, const ros::Time &time, std::string frame_id);

        //publish image msg
        inline void publish_image_msg(ros::Publisher &publisher, 
                                      const cv::Mat &image_mat, const ros::Time &time, std::string frame_id);

        //publish msgs
        inline void publish_msgs();


    private:
        std::thread _process_thread;  //the process thread
        std::mutex _mutex;  //thread mutex lock

        ros::Subscriber _sub_qhd_cloud;  //subscribe qhd_cloud
        ros::Subscriber _sub_qhd_rgb;  //subscribe qhd_rgb image
        ros::Subscriber _sub_qhd_depth;  //subscribe qhd_depth_image
        ros::Publisher _pub_image;  //publish image
        
        ros::Time _qhd_cloud_time;  //time stamp of qhd_cloud
        // ros::Time _qhd_rgb_time;  //time stamp of qhd_rgb image
        ros::Time _qhd_depth_time;  //time stamp of qhd_depth image

        CloudT::Ptr _qhd_cloud;  //qhd_cloud in pcl format
        cv::Mat _qhd_rgb_image;  //qhd_rgb cv image
        // cv_bridge::CvImagePtr _qhd_rgb_image;  //qhd_rgb cv image
        cv_bridge::CvImagePtr _qhd_depth_image;  //qhd_depth cv image

        // std::deque<sensor_msgs::Image> _qhd_rgb_image_que;  //the que of qhd_rgb msgs
        std::map<ros::Time, cv_bridge::CvImagePtr> _qhd_rgb_image_map;  //the map of qhd_rgb_image

        bool _if_qhd_cloud;  //the flag of whether a new sd_cloud msg received
        bool _if_qhd_rgb_image;  //the flag of whether a new qhd_rgb_image msg received
        bool _if_qhd_depth_image;  //the flag of whether a new qhd_depth_image msg received

        int _msgs_index;  //the total msgs index
        int _skip_count;  //skip some frame to reduce the compution

        cv::Mat keypoints_image;
        
};

#endif //FEATURE_H