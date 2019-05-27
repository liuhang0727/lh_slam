#ifndef ROS_UTILS_H
#define ROS_UTILS_H

#include "common_headers.h"

//publish cloud msg
template<typename PointT>
inline void publish_cloud_msg(ros::Publisher &publisher, 
                              const typename pcl::PointCloud<PointT>::Ptr cloud,
                              const ros::Time &time, 
                              std::string frame_id)
{
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = time;
    msg.header.frame_id = frame_id;
    publisher.publish(msg);
}

//publish image msg
inline void publish_image_msg(ros::Publisher &publisher, 
                              const cv::Mat &image_mat,
                              const ros::Time &time, 
                              std::string frame_id, 
                              std::string code_type)
{
    sensor_msgs::CompressedImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), code_type, image_mat).toCompressedImageMsg();
    image_msg->header.stamp = time;
    image_msg->header.frame_id = frame_id;
    publisher.publish(image_msg);
}


#endif //ROS_UTILS_H