// FUCK!! BBBBBUG FILE!!!!!!!!!

#ifndef CLOUD_UTILS_H
#define CLOUD_UTILS_H

#include "common_headers.h"

// down sample point cloud by VoxelGrid provied in PCL
// but the organization will be destoryed
inline void voxel_grid(CloudA::Ptr cloud_in, CloudA::Ptr &cloud_out, double leaf_size)
{
    cloud_out->clear();

    CloudA::Ptr cloud_temp(new CloudA());
    pcl::VoxelGrid<PointA> voxel;
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel.setInputCloud(cloud_in);
    voxel.filter(*cloud_temp);

    // VoxelGrid降采样过后的点云,使用ROS发布消息时,RGB信息出错!!!!!!
    for(int i=0; i<cloud_temp->points.size(); i++)
    {
        PointA p;
        p.x = cloud_temp->points[i].x;
        p.y = cloud_temp->points[i].y;
        p.z = cloud_temp->points[i].z;
        p.r = cloud_temp->points[i].r;
        p.g = cloud_temp->points[i].g;
        p.b = cloud_temp->points[i].b;
        cloud_out->points.push_back(p);
    }
    cloud_out->height = 1;
    cloud_out->width = cloud_out->points.size();
}

// down sample uniformly
// 点云中含有NAN,导致创建kd-tree时出现错误!!!!
inline void uniform_downsample(CloudA::Ptr cloud_in, CloudA::Ptr &cloud_out, int interval) 
{
    cloud_out->clear();
    
    for(int i=0; i<cloud_in->points.size(); i++)
    {
        int row = i / cloud_in->width;
        int col = i % cloud_in->width;

        if(row % interval == 0)
        {
            if(col % interval == 0)
            {
                PointA p = cloud_in->points[i];
                cloud_out->points.push_back(p);
            }
        }
    }

    cloud_out->height = std::ceil(1.0 * cloud_in->height / interval);
    cloud_out->width = std::ceil(1.0 * cloud_in->width / interval);
    // std::cout<<"init: "<<cloud_in->height<<"  "<<cloud_in->width<<"   "<<cloud_in->points.size()<<std::endl;
    // std::cout<<"after: "<<cloud_out->height<<"   "<<cloud_out->width<<"   "<<cloud_out->points.size()<<std::endl<<std::endl;
}

// filter the noise
inline void filter(CloudA::Ptr cloud_in, CloudA::Ptr &cloud_out)
{
    // StatisticalOutlierRemoval 
    pcl::StatisticalOutlierRemoval<PointA> sor;
    sor.setInputCloud(cloud_in);
    sor.setMeanK(20);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_out);
}

// load camera params form yaml file
inline void load_params(std::string path, Eigen::Vector4f &params)
{
    YAML::Node y = YAML::LoadFile(path);
    params[0] = y[0]["params"][0].as<float>();
    params[1] = y[0]["params"][1].as<float>();
    params[2] = y[0]["params"][2].as<float>();
    params[3] = y[0]["params"][3].as<float>();
}

// transform depth image to cloud
inline void create_cloud(cv::Mat rgb, cv::Mat depth, Eigen::Vector4f &params, CloudA::Ptr &cloud)
{
    for(int i=0; i<depth.rows; i++)
    {
        for(int j=0; j<depth.cols; j++)
        {
            PointA p;
            ushort d = depth.ptr<ushort>(i)[j];
            p.z = double(d) / 1000;
            p.x = (j - params[2]) * p.z / params[0];
            p.y = (i - params[3]) * p.z / params[1];
            p.b = rgb.ptr<uchar>(i)[j*3];
            p.g = rgb.ptr<uchar>(i)[j*3+1];
            p.r = rgb.ptr<uchar>(i)[j*3+2];
            cloud->points.push_back(p);
        }
    }
}

// save cloud
template<typename PointT>
inline void save_cloud(std::string path, pcl::PointCloud<PointT> &cloud)
{
    cloud.height = 1;
    cloud.width = cloud.points.size();
    pcl::io::savePCDFile(path, cloud);
    std::cout<<"cloud saved"<<std::endl;
}

// transform plane params in std::vector<Eigen::Vector4f> to pcl::PointCloud<pcl::pointXYZI>
inline void eigen2pcl(std::vector<Eigen::Vector4f> eig, CloudI::Ptr &cloud)
{
    cloud->clear();

    for(int i=0; i<eig.size(); i++)
    {
        PointI p;
        p.x = eig[i][0];
        p.y = eig[i][1];
        p.z = eig[i][2];
        p.intensity = eig[i][3];
        cloud->points.push_back(p);
    }
}

#endif //CLOUD_UTILS_H