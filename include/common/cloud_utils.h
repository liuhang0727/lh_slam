// FUCK!! BBBBBUG FILE!!!!!!!!!

#ifndef CLOUD_UTILS_H
#define CLOUD_UTILS_H

#include "common_headers.h"

// down sample point cloud by VoxelGrid provied in PCL
// but the organization will be destoryed
inline void voxel_grid(const CloudA::Ptr cloud_in, CloudA::Ptr &cloud_out, double leaf_size)
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
inline void uniform_downsample(const CloudA::Ptr cloud_in, CloudA::Ptr &cloud_out, int interval) 
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
inline void filter(const CloudA::Ptr &cloud_in, CloudA::Ptr &cloud_out)
{
    // StatisticalOutlierRemoval 
    pcl::StatisticalOutlierRemoval<PointA> sor;
    sor.setInputCloud(cloud_in);
    sor.setMeanK(20);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_out);
}

// filter the noise of kinect in long range
inline void delete_noise(const CloudA::Ptr &cloud_in, CloudA::Ptr &cloud_out)
{
    cloud_out->clear();

    for(int i=0; i<cloud_in->points.size(); i++)
    {
        PointA p = cloud_in->points[i];
        if(p.z < 8)
        {
            cloud_out->points.push_back(p);
        }
    }
    cloud_out->height = 1;
    cloud_out->width = cloud_out->points.size();
}

// load camera params form yaml file
inline void load_params(const std::string path, Eigen::Vector4f &params)
{
    YAML::Node y = YAML::LoadFile(path);
    params[0] = y[0]["params"][0].as<float>();
    params[1] = y[0]["params"][1].as<float>();
    params[2] = y[0]["params"][2].as<float>();
    params[3] = y[0]["params"][3].as<float>();
}

// transform depth image to cloud
inline void create_cloud(const cv::Mat &rgb, const cv::Mat &depth, Eigen::Vector4f &params, CloudA::Ptr &cloud)
{
    cloud->clear();

    for(int i=0; i<depth.rows; i++)
    {
        for(int j=0; j<depth.cols; j++)
        {
            PointA p;
            ushort d = depth.ptr<ushort>(i)[j];
            p.z = double(d) / 1000.0;
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
inline void save_cloud(const std::string path, pcl::PointCloud<PointT> &cloud)
{
    cloud.height = 1;
    cloud.width = cloud.points.size();
    pcl::io::savePCDFile(path, cloud);
    std::cout<<"cloud saved"<<std::endl;
}

// transform plane params in std::vector<Eigen::Vector4f> to pcl::PointCloud<pcl::pointXYZI>
inline void eigen2pcl(const std::vector<Eigen::Vector4f> eig, CloudI::Ptr &cloud)
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

/*** TODO: cannot work ? ***/
// transform cloud based on the provided r and t
template<typename PointT>
inline void transform_cloud(const pcl::PointCloud<PointT> cloud_in, pcl::PointCloud<PointT> &cloud_out,
                            const Eigen::Matrix3f &r, const Eigen::Vector3f &t)
{
    cloud_out = cloud_in;
    for(int i=0; i<cloud_out.points.size(); i++)
    {
        PointT p = cloud_out.points[i];
        Eigen::Vector3f pei(p.x, p.y, p.z);
        Eigen::Vector3f per;
        per = pei * r + t;

        cloud_out.points[i].x = per[0];
        cloud_out.points[i].y = per[1];
        cloud_out.points[i].z = per[2];
    }
}
// transform cloud based on the provided T
template<typename PointT>
inline void transform_cloud(const pcl::PointCloud<PointT> cloud_in, pcl::PointCloud<PointT> &cloud_out, 
                            const Eigen::Matrix4f &T)
{
    Eigen::Vector3f t(T(0,3), T(1,3), T(2,3));
    Eigen::Matrix3f r;
    r << T(0,0), T(0,1), T(0,2), 
         T(1,0), T(1,1), T(1,2), 
         T(2,0), T(2,1), T(2,2);

    transform_cloud(cloud_in, cloud_out, r, t);     
}



#endif //CLOUD_UTILS_H