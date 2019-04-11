//
// Created by liuhang on 20190404.
//

#include "feature.h"

feature::feature()
    :_qhd_cloud(new CloudT())
{
    _if_qhd_cloud = false;
    _if_qhd_rgb_image = false;
    _if_qhd_depth_image = false;
    
    _msgs_index = 0;
}

feature::~feature()
{
    
}

//set the params, subscribe and publish msgs
bool feature::init(ros::NodeHandle &node, ros::NodeHandle &private_node)
{
    //load params
    private_node.getParam("skip_count", _skip_count);

    //subscribe and publish msgs
    _sub_qhd_cloud = node.subscribe<sensor_msgs::PointCloud2>("/kinect2/sd/points", 10, &feature::qhd_cloud_handler, this);
    _sub_qhd_rgb = node.subscribe<sensor_msgs::Image>("/kinect2/qhd/image_color_rect", 10, &feature::qhd_rgb_handler, this);
    // _sub_qhd_depth = node.subscribe<sensor_msgs::Image>("/kinect2/qhd/image_depth_rect", 10, &feature::qhd_depth_handler, this);
    _pub_image = node.advertise<sensor_msgs::Image>("/keypoints_image", 10);

    //start the process thread
    _process_thread = std::thread(&feature::spin, this);

    return true;
}

//the process of this node
void feature::spin()
{
    //set the max process rate
    ros::Rate rate(100);

    //fall into the process loop
    bool status  = ros::ok();
    while(status)
    {
        ros::spinOnce();
        _mutex.lock();
        process();
        _mutex.unlock();
        status = ros::ok();
        rate.sleep();
    }
}

//the process of this node
void feature::process()
{
    //if there are no new data, return 
    if(!if_new_data())
    { return; }

    //set the msgs flag to false, prapare for next frame
    reset();

    //skip some frames to reduce compution
    _msgs_index++;
    if(_skip_count > 0)
    {
        if(_msgs_index % (_skip_count+1) != 1)
        { return; }
    }

    //find the corrspondence frame in rgb_image_map for cloud frame
    if(!data_match())
    { return; }

    //detect 2D features in rgb image
    feature_detect();

    //publish msgs
    publish_msgs();
}

//the recall handler of qhd_cloud msgs
inline void feature::qhd_cloud_handler(const sensor_msgs::PointCloud2::ConstPtr &qhd_cloud_msg)
{
    ros::Time time = ros::Time::now();
    _qhd_cloud_time = qhd_cloud_msg->header.stamp;
    _qhd_cloud->clear();
    pcl::fromROSMsg(*qhd_cloud_msg, *_qhd_cloud);
    _if_qhd_cloud = true;
   
    // std::cout<<"点云: "<<_qhd_cloud_time<<std::endl;
    // std::cout<<"点云 now: "<<time<<std::endl;
    // std::cout<<"received qhd_cloud msgs..."<<std::endl;
}

//the recall handler of qhd_rgb msgs
inline void feature::qhd_rgb_handler(const sensor_msgs::Image::ConstPtr &qhd_rgb_msg)
{
    _mutex.lock();

    try
    { 
        // //for rbg image "bgr8" or "8UC3"; for depth image "16UC1"; for ir image "16UC1" or "8UC1"
        // //"toCvCopy" or "toCvShare"
        ros::Time qhd_rgb_time = qhd_rgb_msg->header.stamp;
        cv_bridge::CvImagePtr qhd_rgb_image = cv_bridge::toCvCopy(qhd_rgb_msg, "bgr8");

        _qhd_rgb_image_map[qhd_rgb_time] = qhd_rgb_image;
        if(_qhd_rgb_image_map.size() > 8)
        { _qhd_rgb_image_map.erase(_qhd_rgb_image_map.begin()); }

        // std::cout<<"彩色图像: "<<qhd_rgb_time<<std::endl;
        // std::cout<<"彩色图像 now: "<<time<<std::endl;
        // // std::cout<<fabs((_qhd_rgb_time - _qhd_depth_time).toSec())<<std::endl;
        // // std::cout<<"rows: "<<_qhd_rgb_image->image.rows<<"  cols: "<<_qhd_rgb_image->image.cols<<std::endl;
        // // std::cout<<"received qhd_rgb msgs ..."<<std::endl;
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    _mutex.unlock();
}

//the recall handler of qhd_depth msgs
inline void feature::qhd_depth_handler(const sensor_msgs::Image::ConstPtr &qhd_depth_msg)
{
    _mutex.lock();

    try
    { 
        //for rbg image "bgr8" or "8UC3"; for depth image "16UC1"; for ir image "16UC1" or "8UC1"
        //"toCvCopy" or "toCvShare"
        _qhd_depth_time = qhd_depth_msg->header.stamp;
        _qhd_depth_image = cv_bridge::toCvCopy(qhd_depth_msg, "16UC1");
        _if_qhd_depth_image = true;

        // std::cout<<"深度图像: "<<_qhd_depth_time<<std::endl;
        // std::cout<<"深度图像 now: "<<time<<std::endl<<std::endl;
        // std::cout<<"rows: "<<_qhd_rgb_image->image.rows<<"  cols: "<<_qhd_rgb_image->image.cols<<std::endl;
        // std::cout<<"received qhd_depth msgs ..."<<std::endl;
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    _mutex.unlock();
}

//ensure the all data are from the same frame, and each frame only be processed once
inline bool feature::if_new_data()
{
    // // std::cout<<fabs((_qhd_depth_time - _qhd_rgb_time).toSec())<<std::endl;
    // return _if_qhd_rgb_image && _if_qhd_depth_image &&
    //     //    fabs((_qhd_cloud_time - _qhd_rgb_time).toSec()) < 0.05;  //0.3
    //        fabs((_qhd_depth_time - _qhd_rgb_time).toSec()) <0.05;

    return _if_qhd_cloud;
}

//set the msgs flag to false, prapare for next frame
inline void feature::reset()
{
    _if_qhd_cloud = false;
    _if_qhd_rgb_image = false;
    _if_qhd_depth_image = false;
}

//search corresponding frame in rgb_image_map for cloud frame 
inline bool feature::data_match()
{
    auto it = _qhd_rgb_image_map.find(_qhd_cloud_time);
    if(it == _qhd_rgb_image_map.end())
    { return false; }

    _qhd_rgb_image = it->second->image;
    return true;
}

//feature detect in rgb image
void feature::feature_detect()
{
    //feature detect in OpenCV3.X, the contrib is needed for SIFT and SURF
    cv::Ptr<cv::FeatureDetector> detector = cv::xfeatures2d::SIFT::create();
    // cv::Ptr<cv::Feature2D> detector = cv::xfeatures2d::SURF::create();
    // cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();

    //feature deatect in OpenCV2.x
    // cv::Ptr<cv::FeatureDetector> detector = cv::FeatureDetector::create("ORB");

    std::vector<cv::KeyPoint> keypoints;
    detector->detect(_qhd_rgb_image, keypoints);
    cv::drawKeypoints(_qhd_rgb_image, keypoints, keypoints_image, 
                      cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    std::cout<<"feature detected ..."<<std::endl;            

    // std::string path = "/home/liuhang/Documents/" + std::to_string(_msgs_index) + ".png";
    // cv::imwrite(path, keypoints_image);
}

//publish cloud msg
inline void feature::publish_cloud_msg(ros::Publisher &publisher, 
                                       const CloudT &cloud, const ros::Time &time, std::string frame_id)
{
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.stamp = time;
    msg.header.frame_id = frame_id;
    publisher.publish(msg);
}

//publish image msg
inline void feature::publish_image_msg(ros::Publisher &publisher, 
                                       const cv::Mat &image_mat, const ros::Time &time, std::string frame_id)
{
    sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_mat).toImageMsg();
    image_msg->header.stamp = time;
    image_msg->header.frame_id = frame_id;
    publisher.publish(image_msg);
}

//publish msgs
inline void feature::publish_msgs()
{
    publish_image_msg(_pub_image, keypoints_image, _qhd_cloud_time, "kinect2_rgb_optical_frame");
}