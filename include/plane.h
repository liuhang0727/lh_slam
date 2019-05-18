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

    void plane_detect(CloudA::Ptr cloud_in, CloudA::Ptr &cloud_out);

    void plane_fitting(CloudA::Ptr cloud_in, std::vector<int> index_in, int points_count,
                       CloudA::Ptr &cloud_out, std::vector<int> &index_out );

    void compute_area(CloudA::Ptr cloud_in, float &area);

    void region_grow(CloudA::Ptr cloud_in, CloudA::Ptr &cloud_out);     


    public:
    CloudA::Ptr _init_cloud;
    CloudA::Ptr _ds_cloud;
    CloudA::Ptr _ca_cloud;
};

#endif //PLANE_H