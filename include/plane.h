//
// Created by liuhang on 20190501.
//

#ifndef PLANE_H
#define PLANE_H

#include "common/common_headers.h"
#include "common/downsample_utils.h"

class plane
{
    public:
    plane(CloudA::Ptr &cloud);

    void process(CloudA::Ptr plane_cloud);
    void region_grow(CloudA::Ptr cloud_in, CloudA::Ptr &cloud_out, std::vector<Eigen::Vector4f> &params);
    void plane_fitting(CloudA::Ptr cloud_in, Eigen::Vector4f &param);



    void plane_detect(CloudA::Ptr cloud_in, CloudA::Ptr &cloud_out);                    
    void compute_area(CloudA::Ptr cloud_in, float &area);    
    void multi_plane_fitting(CloudA::Ptr cloud_in, std::vector<int> index_in, int points_count, 
                             CloudA::Ptr &cloud_out, std::vector<int> &index_out );  

    
    private:
    CloudA::Ptr _init_cloud;
    CloudA::Ptr _ds_cloud;
    CloudA::Ptr _ca_cloud;
    std::vector<Eigen::Vector4f> _plane_params;
};

#endif //PLANE_H