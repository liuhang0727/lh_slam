#ifndef COMMON_HEADERS_H
#define COMMON_HEADERS_H

//The headers that maybe used

//C++
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <deque>
#include <thread>
#include <mutex>
#include <math.h>
// using namespace std;

//Eigen 
#include <Eigen/Core>
#include <Eigen/Geometry>
// using namespace Eigen;

//PCL
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
typedef pcl::PointXYZRGBA PointT;  //point type
typedef pcl::PointCloud<PointT> CloudT;  //cloud type
// using namespace pcl;

//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//feature detect in OpenCV3.x
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
//feature detect in OpenCV2.x
// #include <opencv2/nonfree/features2d.hpp>
// #include <opencv2/nonfree/nonfree.hpp>
// #include <opencv2/nonfree>
// using namespace cv;

//ROS
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
// using namespace ros;


#endif //COMMON_HEADERS_H
