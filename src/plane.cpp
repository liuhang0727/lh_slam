//
// Created by liuhang on 20190501.
//

#include "plane.h"

plane::plane(CloudA::Ptr &cloud)
    :_init_cloud(cloud), _ds_cloud(new CloudA()), _ca_cloud(new CloudA())
{ }

void plane::process(CloudA::Ptr plane_cloud)
{
    plane_cloud->clear();

    uniform_downsample(_init_cloud, _ds_cloud, 5);  //6
    plane_detect(_ds_cloud, _ca_cloud);

    //down sample param 6
    std::vector<int> in, out;
    for(int i=0; i<_ca_cloud->points.size(); i++)
    { in.push_back(i); }
    plane_fitting(_ca_cloud, in, 100, plane_cloud, out);

    // region_grow(_ds_cloud, _plane_cloud);
}

//
void plane::plane_detect(CloudA::Ptr cloud_in, CloudA::Ptr &cloud_out)
{
    for(int i=0; i<cloud_in->points.size(); i++)
    {
        int row = i / cloud_in->width;
        int col = i % cloud_in->width;

        std::vector<int> indexes;
        for(int m=-2; m<=2; m++)
        {
            int c = col + m;
            if(c>=0 && c<=cloud_in->width)
            { 
                int index = c + row * cloud_in->width;
                indexes.push_back(index);
            }
        }
        
        double sum_x=0, sum_y=0, sum_z=0;
        for(int k=0; k<indexes.size();k++)
        {
            sum_x += cloud_in->points[indexes[k]].x;
            sum_y += cloud_in->points[indexes[k]].y;
            sum_z += cloud_in->points[indexes[k]].z;
        }
        double diff_x = sum_x / indexes.size() - cloud_in->points[i].x;
        double diff_y = sum_y / indexes.size() - cloud_in->points[i].y;
        double diff_z = sum_z / indexes.size() - cloud_in->points[i].z;

        double max = 0.01;
        if(fabs(diff_x)<max && fabs(diff_y)<max && fabs(diff_z)<max)
        {
            cloud_out->points.push_back(cloud_in->points[i]);
        }
    }
}       

//plane fitting, extract all planes
void plane::plane_fitting(CloudA::Ptr cloud_in, std::vector<int> index_in, int points_count,
                          CloudA::Ptr &cloud_out, std::vector<int> &index_out)
{
    pcl::PointIndices::Ptr plane_pointindices(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::SACSegmentation<PointA> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.05);
    seg.setInputCloud(cloud_in);

    boost::shared_ptr<std::vector<int>> index_in_indicesptr = boost::make_shared<std::vector<int>>(index_in);
    int max_iteration_count = 4;
    for(int i=0; i<max_iteration_count; i++)
    {
        seg.setIndices(index_in_indicesptr);
        seg.segment(*plane_pointindices, *coefficients);

        if(plane_pointindices->indices.size() < points_count)
        { break; }

        //TODO 计算平面的面积,如果太小,说明是近处的物体,丢弃掉
        CloudA::Ptr cloud_temp(new CloudA());
        pcl::ExtractIndices<PointA> ext;
        ext.setInputCloud(cloud_in);
        ext.setIndices(plane_pointindices);
        ext.setNegative(false);
        ext.filter(*cloud_temp);
        std::cout<<cloud_temp->points.size()<<std::endl;

        float area;
        compute_area(cloud_temp, area);

        std::unordered_set<int> temp_index_in_indicesptr;
        for(int m=0; m<index_in_indicesptr->size(); m++)
        { temp_index_in_indicesptr.emplace(index_in_indicesptr->at(m)); }

        //random number
        int r = std::rand()%255;
        int g = std::rand()%255;
        int b = std::rand()%255;

        for(int j=0; j<plane_pointindices->indices.size(); j++)
        {
            temp_index_in_indicesptr.erase(plane_pointindices->indices[j]);

            PointA p = cloud_in->points[plane_pointindices->indices[j]];
            p.r = r;
            p.g = g;
            p.b = b;

            if(area > 6)
            {
                cloud_out->points.push_back(p);
                index_out.push_back(plane_pointindices->indices[j]);
            } 
        }

        plane_pointindices->indices.clear();
        index_in_indicesptr->clear();
        for(auto it : temp_index_in_indicesptr)
        { index_in_indicesptr->push_back(it); }

        //std:cout<<coefficients->values[0]<<"  "<<coefficients->values[1]<<"  "<<coefficients->values[2]
        //    <<"  "<<coefficients->values[3]<<"  "<<coefficients->values[4];
    }
}

//
void plane::compute_area(CloudA::Ptr cloud_in, float &area)
{
    //OBB,
    PointA min_p, max_p, obb_p;
    Eigen::Matrix3f obb_r;
    pcl::MomentOfInertiaEstimation<PointA> obb;
    obb.setInputCloud(cloud_in);
    obb.compute();
    obb.getOBB(min_p, max_p, obb_p, obb_r);
    // obb.getAABB(min_p, max_p);

    double diff_x = fabs(max_p.x - min_p.x);
    double diff_y = fabs(max_p.y - min_p.y);
    double diff_z = fabs(max_p.z - min_p.z);

    area = diff_x * diff_y;

    // std::cout<<std::endl<<std::endl;
    // std::cout<<min_p.x<<"   "<<min_p.y<<"   "<<min_p.z<<std::endl;
    // std::cout<<max_p.x<<"   "<<max_p.y<<"   "<<max_p.z<<std::endl;
    std::cout<<diff_x<<"  "<<diff_y<<"   "<<diff_z<<std::endl;
}

//使用区域生长的方法 region growing segmentation，进行平面提取
void plane::region_grow(CloudA::Ptr cloud_in, CloudA::Ptr &cloud_out)
{
    //计算法向量
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimationOMP<PointA, pcl::Normal> n;
    pcl::search::KdTree<PointA>::Ptr tree(new pcl::search::KdTree<PointA>);
    tree->setInputCloud(cloud_in);
    n.setInputCloud(cloud_in);
    n.setSearchMethod(tree);
    n.setKSearch(70);
    n.compute(*normals);

    //区域生长
    std::vector<pcl::PointIndices> clusters;
    pcl::RegionGrowing<PointA, pcl::Normal> reg;
    reg.setMinClusterSize(400);
    reg.setMaxClusterSize(100000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(6);
    reg.setInputCloud(cloud_in);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(10/180.0*M_PI);
    reg.setCurvatureThreshold(0.15);
    reg.extract(clusters);

    for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
    {
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            cloud_out->points.push_back(cloud_in->points[*pit]);
        }
    }
}