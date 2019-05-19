// FUCK!! BBBBBUG FILE!!!!!!!!!

#ifndef DOWNSAMPLE_UTILS_H
#define DOWNSAMPLE_UTILS_H

#include "common_headers.h"

// down sample point cloud by VoxelGrid provied in PCL
// but the organization will be destoryed
inline void voxel_grid(CloudA::Ptr cloud_in, 
                       CloudA::Ptr &cloud_out, 
                       double leaf_size)
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
inline void uniform_downsample(CloudA::Ptr cloud_in, 
                               CloudA::Ptr &cloud_out, 
                               int interval) 
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

#endif //DOWNSAMPLE_UTILS_H