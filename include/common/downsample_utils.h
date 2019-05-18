#ifndef COMMON_H
#define COMMON_H

#include "common_headers.h"

//down sample point cloud by VoxelGrid provied in PCL
//the organization and color will be destoryed 
// template <typename PointT>
inline void voxel_grid(pcl::PointCloud<PointA>::Ptr cloud_in, 
                       pcl::PointCloud<PointA>::Ptr &cloud_out, 
                       double leaf_size)
{
    pcl::VoxelGrid<PointA> voxel;
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel.setInputCloud(cloud_in);
    voxel.filter(*cloud_out);
}

//down sample uniformly
// template <typename PointT>
inline void uniform_downsample(pcl::PointCloud<PointA>::Ptr cloud_in, 
                               pcl::PointCloud<PointA>::Ptr &cloud_out, 
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
    // std::cout<<"after: "<<cloud_out->height<<"   "<<cloud_out->width<<"   "<<cloud_out->points.size()<<std::endl;
}

#endif //COMMON_H