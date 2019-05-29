// 
// Created by liuhang on 20190404.
// 

#ifndef FEATURE_H
#define FEATURE_H

#include "common/common_headers.h"
#include "common/cloud_utils.h"
#include "common/ros_utils.h"
#include "common/transform_utils.h"
#include "subimage_feature.h"
#include "plane.h"

class feature
{
    public:
        feature();
        ~feature();
        
        // set the params, subscribe and publish msgs
        bool init(ros::NodeHandle &node, ros::NodeHandle &private_node);

        // the sub-thread of this node
        void spin();

        // the process of this node
        void process(); 

        // the recall handler of cloud msgs
        inline void cloud_handler(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg);

        // the recall handler of rgb msgs
        inline void rgb_handler(const sensor_msgs::CompressedImage::ConstPtr &rgb_msg);

        // the recall handler of ir msgs
        inline void ir_handler(const sensor_msgs::CompressedImage::ConstPtr &ir_msg);

        // the recall handler of depth msgs
        inline void depth_handler(const sensor_msgs::CompressedImage::ConstPtr &depth_msg);

        // ensure the all data are from the same frame, and each frame only be processed once
        inline bool if_new_data();

        // set the msgs flag to false, prapare for next frame
        inline void reset();

        // publish msgs
        inline void publish_msgs();

        // search corresponding frame in rgb_image_map for cloud frame 
        inline bool data_match();

        // detect and match features in images
        bool feature_detect(cv::Mat &last_image, cv::Mat &image, 
                            std::vector<cv::KeyPoint> &last_keypoints, std::vector<cv::KeyPoint> &keypoints,
                            cv::Mat &last_descriptor, cv::Mat &descriptor,
                            std::vector<cv::DMatch> &matches, cv::Mat &match_image,
                            bool &if_first_frame);

        // stretch the gray scale of ir image to ensure feature extract process
        void gray_stretch();

        // load faro cloud 
        void faro();

        // plane association between kinect and faro
        bool plane_match();

        // solve pose
        bool pose_solve(std::vector<std::pair<Eigen::Vector4f, Eigen::Vector4f> > matches, Eigen::Matrix4f &T);

        // compute score of each transform
        double score(Eigen::Matrix4f T);

        // match kinect and faro
        void match();

    private:
        std::thread _process_thread;  // the process thread
        std::mutex _mutex;  // thread mutex lock

        ros::Subscriber _sub_cloud;  // subscriber of cloud
        ros::Subscriber _sub_rgb;  // subscriber of rgb image
        ros::Subscriber _sub_depth;  // subscriber of depth image
        ros::Subscriber _sub_ir;  // subscriber of ir image
        ros::Publisher _pub_image;  // publisher of image
        ros::Publisher _pub_cloud;  // publisher of cloud
        ros::Publisher _pub_kinect;
        ros::Publisher _pub_faro;  // publisher of faro cloud
        
        ros::Time _cloud_time;  // time stamp of cloud
        ros::Time _rgb_time;  // time stamp of rgb image
        ros::Time _depth_time;  // time stamp of depth image
        ros::Time _ir_time;  // time stamp of ir image

        CloudA::Ptr _cloud;  // cloud in pcl format
        CloudA::Ptr _ds_cloud;  // down sample cloud
        CloudA::Ptr _plane_cloud;  // plane cloud of kinect
        CloudA::Ptr _T_plane_cloud;
        CloudA::Ptr _faro_cloud;  //faro cloud
        CloudA::Ptr _pf_cloud;  //plane cloud of faro

        cv::Mat _rgb_image;  // rgb cv image
        cv::Mat _ir_image;  // ir cv image
        cv::Mat _last_rgb_image;  // the last frame rgb cv image
        cv::Mat _last_ir_image;  // the last frame ir cv image
        std::vector<cv::KeyPoint> _rgb_keypoints;  // the keypoints of the current rgb frame
        std::vector<cv::KeyPoint> _ir_keypoints;  // the keypoints of the current ir frame
        std::vector<cv::KeyPoint> _last_rgb_keypoints;  // the keypoints of the last rgb frame
        std::vector<cv::KeyPoint> _last_ir_keypoints;  // the keypoints of the last ir frame
        cv::Mat _rgb_descriptor;  // the feature descriptor of current rgb frame
        cv::Mat _ir_descriptor;  // the feature descriptor of current ir frame 
        cv::Mat _last_rgb_descriptor;  // the feature descriptor of last rgb frame
        cv::Mat _last_ir_descriptor;  // the feature descriptor of last ir frame
        std::vector<cv::DMatch> _rgb_matches;  // rgb matches of two frames
        std::vector<cv::DMatch> _ir_matches;  // ir matches of two frames
        cv::Mat _keypoints_image;
        cv::Mat _match_image;

        cv_bridge::CvImagePtr _depth_bridge;  // depth cv bridge
        cv_bridge::CvImagePtr _ir_bridge;  // ir cv bridge
        std::map<ros::Time, cv_bridge::CvImagePtr> _rgb_bridge_map;  // the map of rgb cv bridge

        bool _if_cloud;  // the flag of whether a new cloud msg received
        bool _if_rgb_image;  // the flag of whether a new rgb_image msg received
        bool _if_depth_image;  // the flag of whether a new depth_image msg received
        bool _if_ir_image;  // the flag of whether a new ir_image msg received

        bool _if_first_frame;  // the flag of whether it's the first process frame, to 2D feature match 

        long _msgs_index;  // the total msgs index
        int _skip_count;  // skip some frame to reduce the compution



        std::unordered_map<int, std::vector<Eigen::Vector4f> > _kinect_gp;
        std::unordered_map<int, std::vector<Eigen::Vector4f> > _faro_gp;
        pcl::KdTreeFLANN<PointA> _tree;
        std::map<double, Eigen::Matrix4f> _scores;
};

#endif // FEATURE_H