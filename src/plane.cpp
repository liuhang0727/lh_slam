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

    // voxel_grid(_init_cloud, _ds_cloud, 0.2);  //for faro 0.1~0.2
    uniform_downsample(_init_cloud, _ds_cloud, 5);  //for kinect 5

    /* detect planes by region_grow, the performance is better than plan_fitting */
    region_grow(_ds_cloud, plane_cloud, _plane_params);
    




    /* detect planes by plane_fitting */
    // plane_detect(_ds_cloud, _ca_cloud);
    // std::vector<int> in, out;
    // for(int i=0; i<_ca_cloud->points.size(); i++)
    // { in.push_back(i); }
    // multi_plane_fitting(_ca_cloud, in, 100, plane_cloud, out);
}

// plane segmentation by region growing
void plane::region_grow(CloudA::Ptr cloud_in, CloudA::Ptr &cloud_out, std::vector<Eigen::Vector4f> &params)
{
    // delete NAN points
    CloudA::Ptr cloud_nonan(new CloudA());
    for(int i=0; i<cloud_in->points.size(); i++)
    {
        float d = cloud_in->points[i].z;
        if(d == d)
        { cloud_nonan->points.push_back(cloud_in->points[i]); }
    }

    // build kd-tree
    pcl::search::KdTree<PointA>::Ptr tree(new pcl::search::KdTree<PointA>);
    tree->setInputCloud(cloud_nonan);

    // compute normal vector
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimationOMP<PointA, pcl::Normal> n;
    n.setSearchMethod(tree);
    n.setKSearch(10);  //10
    n.setInputCloud(cloud_nonan);
    n.compute(*normals);

    // region growing
    std::vector<pcl::PointIndices> clusters;
    pcl::RegionGrowing<PointA, pcl::Normal> reg;
    reg.setSearchMethod(tree);
    reg.setMinClusterSize(50);
    reg.setMaxClusterSize(100000);
    reg.setNumberOfNeighbours(4);  //4
    reg.setSmoothnessThreshold(8/180.0*M_PI);  //8
    reg.setCurvatureThreshold(0.1);  //0.1
    reg.setInputCloud(cloud_nonan);
    reg.setInputNormals(normals);
    reg.extract(clusters);

    // compute plane parmas for each cluster
    for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
    {
        //random number
        int r = std::rand()%255;
        int g = std::rand()%255;
        int b = std::rand()%255;

        CloudA::Ptr cloud_temp(new CloudA());
        Eigen::Vector4f param_temp;
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            PointA p = cloud_nonan->points[*pit];
            p.r = r;
            p.g = g;
            p.b = b;

            cloud_temp->points.push_back(p);
            cloud_out->points.push_back(p);
        }
        plane_fitting(cloud_temp, param_temp);
        params.push_back(param_temp);
    }
    // std::cout<<std::endl<<std::endl;
}

// compute param of one plane by RANSAC
void plane::plane_fitting(CloudA::Ptr cloud_in, Eigen::Vector4f &param)
{
    pcl::PointIndices::Ptr ind(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coef(new pcl::ModelCoefficients);
    pcl::SACSegmentation<PointA> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.05);  //0.05
    seg.setInputCloud(cloud_in);
    seg.segment(*ind, *coef);

    for(int i=0; i<4; i++)
    { 
        param[i] = coef->values[i];
        // std::cout<<param[i]<<" ";
    }
    // std::cout<<std::endl;
}















// try to find plane by the organization character, but it does not work...
void plane::plane_detect(CloudA::Ptr cloud_in, CloudA::Ptr &cloud_out)
{
    cloud_out->clear();

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
void plane::multi_plane_fitting(CloudA::Ptr cloud_in, std::vector<int> index_in, int points_count,
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
    int max_iteration_count = 6;
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

            // if(area > 6)
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

// compute plane area
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