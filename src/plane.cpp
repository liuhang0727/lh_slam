//
// Created by liuhang on 20190501.
//

#include "plane.h"

plane::plane(CloudA::Ptr &cloud)
    :_init_cloud(cloud), _ds_cloud(new CloudA()), _ca_cloud(new CloudA())
{ }

void plane::process(CloudA::Ptr &plane_cloud, std::unordered_map<int, std::vector<Eigen::Vector4f> > &gp)
{
    plane_cloud->clear();
    gp.clear();

    /* detect planes by region_grow, the performance is better than plan_fitting */
    // automatic recognition of kinect and faro
    if(_init_cloud->height == 1)
    {
        // faro
        voxel_grid(_init_cloud, _ds_cloud, 0.10); //0.12
        region_grow(_ds_cloud, plane_cloud, gp);
    }
    else
    {
        // kinect
        // uniform_downsample(_init_cloud, temp_cloud, 12);  //8

        voxel_grid(_init_cloud, _ds_cloud, 0.10);
        region_grow(_ds_cloud, plane_cloud, gp);
    }



    /* detect planes by plane_fitting */
    // plane_detect(_ds_cloud, _ca_cloud);
    // std::vector<int> in, out;
    // for(int i=0; i<_ca_cloud->points.size(); i++)
    // { in.push_back(i); }
    // multi_plane_fitting(_ca_cloud, in, 100, plane_cloud, out);
}

// plane segmentation by region growing
void plane::region_grow(CloudA::Ptr cloud_in, CloudA::Ptr &cloud_out, 
                        std::unordered_map<int, std::vector<Eigen::Vector4f> > &gp)
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
    reg.setMinClusterSize(30);  //50
    reg.setMaxClusterSize(100000);  //100000
    reg.setNumberOfNeighbours(10);  //4
    reg.setSmoothnessThreshold(4/180.0*M_PI);  //8
    reg.setCurvatureThreshold(0.02);  //0.1
    reg.setInputCloud(cloud_nonan);
    reg.setInputNormals(normals);
    reg.extract(clusters);

    // compute plane parmas for each cluster
    std::unordered_map<int, std::vector<Eigen::Vector4f> > gp_temp;
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
        group(param_temp, gp_temp);
    }

    // delete the redundant same planes
    merge(gp_temp, gp);

    std::cout<<std::endl<<std::endl;
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
        // ensuring the consistency of normal vector direction
        param[i] = coef->values[i];
        // if(coef->values[0] < 0)
        // { param[i] = -1.0*param[i]; }

        std::cout<<param[i]<<" ";
    }
    std::cout<<std::endl;
}

// divide the plans into three groups
void plane::group(Eigen::Vector4f param, std::unordered_map<int, std::vector<Eigen::Vector4f> > &gp)
{
    // compare which is max(fabs) among param[0] [1] [2]
    float x = fabs(param[0]);
    float y = fabs(param[1]);
    float z = fabs(param[2]);

    if(x>=y && x>=z)
    { gp[0].push_back(param); }
    else if(y>=x && y>=z)
    { gp[1].push_back(param); }
    else if(z>=x && z>=y)
    { gp[2].push_back(param); }
}

// delete the redundant same planes
void plane::merge(std::unordered_map<int, std::vector<Eigen::Vector4f> > gp_in, 
           std::unordered_map<int, std::vector<Eigen::Vector4f> > &gp_out)
{
    for(int i=0; i<3; i++)
    {
        // x y z
        for(int j=0; j<gp_in[i].size(); j++)
        {
            if(gp_out[i].size() == 0)
            {
                gp_out[i].push_back(gp_in[i][0]);
                continue;
            }

            bool same = false;
            for(int k=0; k<gp_out[i].size(); k++)
            {
                float x = gp_in[i][j][0] - gp_out[i][k][0];
                float y = gp_in[i][j][1] - gp_out[i][k][1];
                float z = gp_in[i][j][2] - gp_out[i][k][2];
                float d = gp_in[i][j][3] - gp_out[i][k][3];
                if (fabs(x)<0.2 && fabs(y)<0.2 && fabs(z)<0.2 && fabs(d)<0.2)
                {
                    same = true;
                    continue;
                }
            }
            if(!same)
            {
                gp_out[i].push_back(gp_in[i][j]);
            }
        }
    }

    // cout
    std::cout<<std::endl<<"合并之前平面参数: "<<std::endl;
    for(int m=0; m<3; m++)
    {
        for(int i=0; i<gp_in[m].size(); i++)
        {
            std::cout<<gp_in[m][i][0]<<"  "<<gp_in[m][i][1]<<"  "
            <<gp_in[m][i][2]<<"  "<<gp_in[m][i][3]<<std::endl;
        }
        std::cout<<std::endl;
    }

    // cout
    std::cout<<std::endl<<"合并之后平面参数: "<<std::endl;
    for(int m=0; m<3; m++)
    {
        for(int i=0; i<gp_out[m].size(); i++)
        {
            std::cout<<gp_out[m][i][0]<<"  "<<gp_out[m][i][1]<<"  "
            <<gp_out[m][i][2]<<"  "<<gp_out[m][i][3]<<std::endl;
        }
        std::cout<<std::endl;
    }
}


/************************************** rubbish **************************************/

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

/************************************** rubbish **************************************/