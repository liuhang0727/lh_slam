//
// Created by liuhang on 20190404.
//

#ifndef FEATURE_H
#define FEATURE_H

#include "common/common_headers.h"
#include "common/downsample_utils.h"
#include "subimage_feature.h"
#include "plane.h"

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

        //the recall handler of sd_ir msgs
        inline void sd_ir_handler(const sensor_msgs::Image::ConstPtr &sd_ir_msg);

        //the recall handler of qhd_depth msgs
        inline void qhd_depth_handler(const sensor_msgs::Image::ConstPtr &qhd_depth_msg);

        //ensure the all data are from the same frame, and each frame only be processed once
        inline bool if_new_data();

        //set the msgs flag to false, prapare for next frame
        inline void reset();

        //publish cloud msg
        inline void publish_cloud_msg(ros::Publisher &publisher, 
                                      const CloudA::Ptr &cloud, const ros::Time &time, std::string frame_id);

        //publish image msg
        inline void publish_image_msg(ros::Publisher &publisher, const cv::Mat &image_mat,
                                      const ros::Time &time, std::string frame_id, std::string code_type);

        //publish msgs
        inline void publish_msgs();

        //search corresponding frame in rgb_image_map for cloud frame 
        inline bool data_match();

        //detect and match features in images
        bool feature_detect(cv::Mat &last_image, cv::Mat &image, 
                            std::vector<cv::KeyPoint> &last_keypoints, std::vector<cv::KeyPoint> &keypoints,
                            cv::Mat &last_descriptor, cv::Mat &descriptor,
                            std::vector<cv::DMatch> &matches, cv::Mat &match_image,
                            bool &if_first_frame);

        //stretch the gray scale of ir image to ensure feature extract process
        void gray_stretch();

        //extract planes from point cloud
        void plane_extract();

        //down sample
        inline void down_sample(CloudA::Ptr &cloud_in, CloudA::Ptr &cloud_out, double leaf_size);



    private:
        std::thread _process_thread;  //the process thread
        std::mutex _mutex;  //thread mutex lock

        ros::Subscriber _sub_qhd_cloud;  //subscriber of qhd_cloud
        ros::Subscriber _sub_qhd_rgb;  //subscriber of qhd_rgb image
        ros::Subscriber _sub_qhd_depth;  //subscriber of qhd_depth image
        ros::Subscriber _sub_sd_ir;  //subscriber of sd_ir image
        ros::Publisher _pub_image;  //publisher of image
        ros::Publisher _pub_cloud;  //publisher of cloud
        
        ros::Time _qhd_cloud_time;  //time stamp of qhd_cloud
        // ros::Time _qhd_rgb_time;  //time stamp of qhd_rgb image
        ros::Time _qhd_depth_time;  //time stamp of qhd_depth image
        ros::Time _sd_ir_time;  //time stamp of sd_ir image

        CloudA::Ptr _qhd_cloud;  //qhd_cloud in pcl format
        CloudA::Ptr _plane_cloud;  //plane set in point cloud

        cv::Mat _qhd_rgb_image;  //qhd_rgb cv image
        cv::Mat _sd_ir_image;  //sd_ir cv image
        cv::Mat _last_qhd_rgb_image;  //the last frame qhd_rgb cv image
        cv::Mat _last_sd_ir_image;  //the last frame sd_ir cv image
        std::vector<cv::KeyPoint> _qhd_rgb_keypoints;  //the keypoints of the current rgb frame
        std::vector<cv::KeyPoint> _sd_ir_keypoints;  //the keypoints of the current ir frame
        std::vector<cv::KeyPoint> _last_qhd_rgb_keypoints;  //the keypoints of the last rgb frame
        std::vector<cv::KeyPoint> _last_sd_ir_keypoints;  //the keypoints of the last ir frame
        cv::Mat _qhd_rgb_descriptor;  //the feature descriptor of current rgb frame
        cv::Mat _sd_ir_descriptor;  //the feature descriptor of current ir frame 
        cv::Mat _last_qhd_rgb_descriptor;  //the feature descriptor of last rgb frame
        cv::Mat _last_sd_ir_descriptor;  //the feature descriptor of last ir frame
        std::vector<cv::DMatch> _qhd_rgb_matches;  //qhd_rgb matches of two frames
        std::vector<cv::DMatch> _sd_ir_matches;  //sd_ir matches of two frames
        cv::Mat _keypoints_image;
        cv::Mat _match_image;

        cv_bridge::CvImagePtr _qhd_depth_bridge;  //qhd_depth cv bridge
        cv_bridge::CvImagePtr _sd_ir_bridge;  //sd_ir cv bridge
        std::map<ros::Time, cv_bridge::CvImagePtr> _qhd_rgb_bridge_map;  //the map of qhd_rgb cv bridge

        bool _if_qhd_cloud;  //the flag of whether a new sd_cloud msg received
        bool _if_qhd_rgb_image;  //the flag of whether a new qhd_rgb_image msg received
        bool _if_qhd_depth_image;  //the flag of whether a new qhd_depth_image msg received
        bool _if_sd_ir_image;  //the flag of whether a new sd_ir_image msg received

        bool _if_first_frame;  //the flag of whether it's the first process frame, to 2D feature match 

        long _msgs_index;  //the total msgs index
        int _skip_count;  //skip some frame to reduce the compution
};

#endif //FEATURE_H