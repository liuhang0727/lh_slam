//
// Created by liuhang on 20190501.
//

#ifndef PLANE_H
#define PLANE_H

#include "common/common_headers.h"
#include "common/cloud_utils.h"

class plane
{
    public:
    plane(CloudA::Ptr &cloud);

    void process(CloudA::Ptr &plane_cloud, std::unordered_map<int, std::vector<Eigen::Vector4f> > &gp);
    void region_grow(CloudA::Ptr cloud_in, CloudA::Ptr &cloud_out, std::unordered_map<int, std::vector<Eigen::Vector4f> > &gp);
    void plane_fitting(CloudA::Ptr cloud_in, Eigen::Vector4f &param);
    void group(Eigen::Vector4f param, std::unordered_map<int, std::vector<Eigen::Vector4f> > &gp);
    void merge(std::unordered_map<int, std::vector<Eigen::Vector4f> > gp_in, 
               std::unordered_map<int, std::vector<Eigen::Vector4f> > &gp_out);



    void plane_detect(CloudA::Ptr cloud_in, CloudA::Ptr &cloud_out);                    
    void compute_area(CloudA::Ptr cloud_in, float &area);    
    void multi_plane_fitting(CloudA::Ptr cloud_in, std::vector<int> index_in, int points_count, 
                             CloudA::Ptr &cloud_out, std::vector<int> &index_out );  

    
    private:
    CloudA::Ptr _init_cloud;
    CloudA::Ptr _ds_cloud;
    CloudA::Ptr _ca_cloud;
};

#endif //PLANE_H