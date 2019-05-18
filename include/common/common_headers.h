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
#include <unordered_set>
#include <list>
#include <deque>
#include <thread>
#include <mutex>
#include <math.h>
#include <numeric>
#include <time.h>
#include <stdlib.h>
// using namespace std;

//Eigen 
#include <Eigen/Core>
#include <Eigen/Geometry>
// using namespace Eigen;

//PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/boundary.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/registration/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
typedef pcl::PointXYZRGBA PointA;  //point type
typedef pcl::PointXYZI PointI;
// typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointA> CloudA;  //cloud type
typedef pcl::PointCloud<PointI> CloudI;
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
