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
    _if_sd_ir_image = false;
    _if_first_frame = true;
    
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
    _sub_qhd_cloud = node.subscribe<sensor_msgs::PointCloud2>("/kinect2/qhd/points", 10, &feature::qhd_cloud_handler, this);
    _sub_qhd_rgb = node.subscribe<sensor_msgs::Image>("/kinect2/qhd/image_color", 10, &feature::qhd_rgb_handler, this);
    _sub_sd_ir = node.subscribe<sensor_msgs::Image>("/kinect2/sd/image_ir", 10, &feature::sd_ir_handler, this);
    // _sub_qhd_depth = node.subscribe<sensor_msgs::Image>("/kinect2/qhd/image_depth_rect", 10, &feature::qhd_depth_handler, this);
    _pub_image = node.advertise<sensor_msgs::Image>("/match_image", 10);

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

    // detect 2D features in rgb image, and macth two images
    // bool b = feature_detect(_last_qhd_rgb_image, _qhd_rgb_image, _last_qhd_rgb_keypoints, _qhd_rgb_keypoints,
    //                         _last_qhd_rgb_descriptor, _qhd_rgb_descriptor,
    //                         _qhd_rgb_matches, _match_image, _if_first_frame);

    //for ir image
    gray_stretch();
    bool b = feature_detect(_last_sd_ir_image, _sd_ir_image, _last_sd_ir_keypoints, _sd_ir_keypoints, 
                            _last_sd_ir_descriptor, _sd_ir_descriptor,
                            _sd_ir_matches, _match_image, _if_first_frame);

    

    if(!b)
    { return; }  

    // publish msgs
    publish_msgs();
}

//the recall handler of qhd_cloud msgs
inline void feature::qhd_cloud_handler(const sensor_msgs::PointCloud2::ConstPtr &qhd_cloud_msg)
{
    _mutex.lock();
    
    ros::Time time = ros::Time::now();
    _qhd_cloud_time = qhd_cloud_msg->header.stamp;
    _qhd_cloud->clear();
    pcl::fromROSMsg(*qhd_cloud_msg, *_qhd_cloud);
    _if_qhd_cloud = true;

    _mutex.unlock();
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
        cv_bridge::CvImagePtr qhd_rgb_image = cv_bridge::toCvCopy(qhd_rgb_msg, "8UC3");

        _qhd_rgb_bridge_map[qhd_rgb_time] = qhd_rgb_image;
        if(_qhd_rgb_bridge_map.size() > 8)
        { _qhd_rgb_bridge_map.erase(_qhd_rgb_bridge_map.begin()); }
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
        _qhd_depth_bridge = cv_bridge::toCvCopy(qhd_depth_msg, "16UC1");
        _if_qhd_depth_image = true;
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    _mutex.unlock();
}

//the recall handler of sd_ir msgs
inline void feature::sd_ir_handler(const sensor_msgs::Image::ConstPtr &sd_ir_msg)
{
    _mutex.lock();

    try
    {
        _sd_ir_time = sd_ir_msg->header.stamp;
        _sd_ir_bridge = cv_bridge::toCvCopy(sd_ir_msg, "8UC1");
        _if_sd_ir_image = true;
    }
    catch(const std::exception& e)
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
            //   fabs((_qhd_cloud_time - _qhd_rgb_time).toSec()) < 0.05;  //0.3
    //        fabs((_qhd_depth_time - _qhd_rgb_time).toSec()) <0.05;

    return _if_qhd_cloud && _if_sd_ir_image;
}

//set the msgs flag to false, prapare for next frame
inline void feature::reset()
{
    _if_qhd_cloud = false;
    _if_qhd_rgb_image = false;
    _if_qhd_depth_image = false;
    _if_sd_ir_image = false;

    _last_qhd_rgb_image = _qhd_rgb_image.clone();
    _last_qhd_rgb_keypoints = _qhd_rgb_keypoints;
    _last_qhd_rgb_descriptor = _qhd_rgb_descriptor.clone();

    _last_sd_ir_image = _sd_ir_image.clone();
    _last_sd_ir_keypoints = _sd_ir_keypoints;
    _last_sd_ir_descriptor = _sd_ir_descriptor.clone();
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
inline void feature::publish_image_msg(ros::Publisher &publisher, const cv::Mat &image_mat, 
                                       const ros::Time &time, std::string frame_id,  std::string code_type)
{
    sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), code_type, image_mat).toImageMsg();
    image_msg->header.stamp = time;
    image_msg->header.frame_id = frame_id;
    publisher.publish(image_msg);
}

//publish msgs
inline void feature::publish_msgs()
{
    publish_image_msg(_pub_image, _match_image, _qhd_cloud_time, "kinect2_rgb_optical_frame", "8UC3");
}

//search corresponding frame in rgb_image_map for cloud frame 
inline bool feature::data_match()
{
    auto it = _qhd_rgb_bridge_map.find(_qhd_cloud_time);
    if(it == _qhd_rgb_bridge_map.end())
    { return false; }

    _qhd_rgb_image = it->second->image;
    return true;
}

//feature detect
bool feature::feature_detect(cv::Mat &last_image, cv::Mat &image, 
                            std::vector<cv::KeyPoint> &last_keypoints, std::vector<cv::KeyPoint> &keypoints,
                            cv::Mat &last_descriptor, cv::Mat &descriptor,
                            std::vector<cv::DMatch> &matches, cv::Mat &match_image,
                            bool &if_first_frame)
{
     matches.clear();

    //feature deatect in OpenCV2.x
    // cv::Ptr<cv::FeatureDetector> detector = cv::FeatureDetector::create("ORB");

    //feature detect in OpenCV3.X, the contrib is needed for SIFT and SURF
    // cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create();  //180ms for qhd size
    // cv::Ptr<cv::xfeatures2d::SIFT> detector = cv::xfeatures2d::SIFT::create();  //300ms for qhd size
    cv::Ptr<cv::ORB> detector = cv::ORB::create();  //30ms for qhd size

    if(if_first_frame)
    {
        last_image = image.clone();
        detector->detect(last_image, last_keypoints);
        detector->compute(last_image, last_keypoints, last_descriptor);
        if_first_frame = false;
        return false;
    }

    detector->detect(image, keypoints);
    detector->compute(image, keypoints, descriptor);

    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<cv::DMatch> temp_matches;
    matcher.match(last_descriptor, descriptor, temp_matches);

    double min_dist = 1000, max_dist = 0;
    for(int i=0; i<temp_matches.size(); i++)
    {
        double dist = temp_matches[i].distance;
        if(dist < min_dist) min_dist = dist;
        if(dist > max_dist) max_dist = dist;
    }
    for(int i=0; i<temp_matches.size(); i++)
    {
        if(temp_matches[i].distance <= std::max(2*min_dist, 30.0))
        { matches.push_back(temp_matches[i]); }  
    }
    cv::drawMatches(last_image, last_keypoints, image, keypoints,
                    matches, match_image);

    // cv::drawKeypoints(image, keypoints, _keypoints_image, 
                //   cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);    

    // std::string path = "/home/liuhang/Documents/" + std::to_string(_msgs_index) + ".png";
    // cv::imwrite(path, image);

    return true;
}

//stretch the gray scale of ir image to ensure feature extract process
void feature::gray_stretch()
{
    //statistic the gray value
    _sd_ir_image = _sd_ir_bridge->image.clone();
    std::map<int, int> sta;  
    int rows = _sd_ir_image.rows;
    int cols = _sd_ir_image.cols;
    for(int i=0; i<rows; i++)
    {
        for(int j=0; j<cols; j++)
        {
            //uchar/8UC1, uchar/16UC1
            int v = (int)_sd_ir_image.at<uchar>(i,j);
            sta[v]++;
        }
    }

    //calculate the min and max of 0.5%
    double persent = 0;
    int min=0, max=0;
    bool if_min=false, if_max=false;
    for(auto it : sta)
    {
        double persent_temp = (it.second*1.0) / (rows*cols) * 100;
        persent += persent_temp;

        if(!if_min && persent>0.8)
        {
            min = it.first;
            if_min = true;
            // std::cout<<"min "<<min<<std::endl;
        }
        else if(!if_max && persent>99.2)
        {
            max = it.first;
            if_max = true;
            // std::cout<<"max "<<max<<std::endl;
            break;
        }
    }

    //strectch the gray scale
    for(int i=0; i<rows; i++)
    {
        for(int j=0; j<cols; j++)
        {
            int v = (int)_sd_ir_image.at<uchar>(i,j);
            if(v <= min)
            { _sd_ir_image.at<uchar>(i,j) = 0; }
            else if(v >= max)
            { _sd_ir_image.at<uchar>(i,j) = 255; }
            else
            { _sd_ir_image.at<uchar>(i,j) = (255/(max-min))*(_sd_ir_image.at<uchar>(i,j)-min); } 
        }
    }
}
