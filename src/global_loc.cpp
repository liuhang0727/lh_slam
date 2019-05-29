//
// Created by liuhang on 20190528.
//

#include "global_loc.h"

global_loc::global_loc()
    :_kinect_cloud(new CloudA()), _pk_cloud(new CloudA()),
     _faro_cloud(new CloudA()), _pf_cloud(new CloudA())
{ 
    _if_rgb = false;
    _if_depth = false;
    _if_cloud = false;
}

global_loc::~global_loc()
{ }

// set the params, subscribe and publish msgs
bool global_loc::init(ros::NodeHandle &node, ros::NodeHandle &private_node)
{
    // load params
    private_node.getParam("skip_count", _skip_count);
    private_node.getParam("faro_path", _faro_path);
    private_node.getParam("cam_params_path", _cam_params_path); 
    _cam_params << 365.52542597, 364.17089950, 256.01281275, 209.60721677;
    // TODO: why it cannot work in this file !!!
    // load_params("/home/liuhang/Documents/catkin_ws_kinect/src/lh_slam/params/ir.yaml", _cam_params); 

    // subscribe and advertise msgs
    _sub_rgb = node.subscribe<sensor_msgs::CompressedImage>
               ("/kinect2/sd/image_color_rect/compressed", 10, &global_loc::rgb_handler, this);
    _sub_depth = node.subscribe<sensor_msgs::CompressedImage>
                 ("/kinect2/sd/image_depth_rect/compressed", 10, &global_loc::depth_handler, this);
    _sub_cloud = node.subscribe<sensor_msgs::PointCloud2>
                 ("/kinect2/sd/points", 10, &global_loc::cloud_handler, this);       

    _pub_kinect_cloud = node.advertise<sensor_msgs::PointCloud2>("/init_cloud", 10);
    _pub_pk_cloud = node.advertise<sensor_msgs::PointCloud2>("/plane_cloud", 10);
    _pub_faro_cloud = node.advertise<sensor_msgs::PointCloud2>("/faro_cloud", 10);

    // load faro map
    load_faro();

    // start the sub-thread
    _sub_thread = std::thread(&global_loc::spin, this);

    return true;
}

// the process of this node
void global_loc::spin()
{
    // set the max process rate
    ros::Rate rate(100);

    // fall into the process loop
    bool status  = ros::ok();
    while(status)
    {
        ros::spinOnce();
        _mutex.lock();
        process();
        _mutex.unlock();
        status = ros::ok();
        rate.sleep();
    }
}

// the process of this node
void global_loc::process()
{
    // if there are no new data, return 
    if( !if_new_data() )
    { return; }

    // set the msgs flag to false, prapare for next frame
    reset();

    // skip some frames to reduce compution
    _msgs_index++;
    if(_skip_count > 0)
    {
        if(_msgs_index % (_skip_count+1) != 1)
        { return; }
    }

    // pre-process for kinect cloud: creat cloud, delete noise, pre-transform 
    pre_process();

    // plane segment for kinect cloud
    std::cout<<"kinect plane params: "<<std::endl;
    plane p(_kinect_cloud);
    p.process(_pk_cloud, _kinect_params);

    // plane match
    plane_match();

    // publish msgs
    publish_cloud_msg<PointA>(_pub_kinect_cloud, _kinect_cloud, _rgb_time, "kinect2_ir_optical_frame");
    publish_cloud_msg<PointA>(_pub_pk_cloud, _pk_cloud, _rgb_time, "kinect2_ir_optical_frame");
    publish_cloud_msg<PointA>(_pub_faro_cloud, _faro_cloud, _rgb_time, "kinect2_ir_optical_frame");

    std::cout<<"plane match complete !"<<std::endl;
}

// the recall handler of rgb msgs
inline void global_loc::rgb_handler(const sensor_msgs::CompressedImage::ConstPtr &rgb_msg)
{
    _mutex.lock();

    try
    {
        // bye, cv_bridge
        // cv_bridge::CvImagePtr rgb_bridge = cv_bridge::toCvCopy(rgb_msg, "bgr8");
        // _rgb_image = rgb_bridge->image.clone();

        _rgb_time = rgb_msg->header.stamp;
        _rgb_image = cv::imdecode(cv::Mat(rgb_msg->data),  cv::ImreadModes::IMREAD_COLOR);
        _if_rgb = true;

        // std::cout<<"_rgb_time"<<_rgb_time<<std::endl;
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    _mutex.unlock();
}

// the recall handler of depth msgs
inline void global_loc::depth_handler(const sensor_msgs::CompressedImage::ConstPtr &depth_msg)
{
    _mutex.lock();

    try
    { 
        // TODO: there are something wrong with this method
        // cv_bridge::CvImagePtr depth_bridge = cv_bridge::toCvCopy(depth_msg, "16UC1");
        // _depth_image = depth_bridge->image.clone();

        _depth_time = depth_msg->header.stamp;
        _depth_image = cv::imdecode(cv::Mat(depth_msg->data),  cv::ImreadModes::IMREAD_ANYDEPTH);
        _if_depth = true;      

        // std::cout<<"_depth_time"<<_depth_time<<std::endl<<std::endl;
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    _mutex.unlock();
}

inline void global_loc::cloud_handler(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)
{
    _mutex.lock();

    CloudA::Ptr temp_cloud(new CloudA());

    _cloud_time = cloud_msg->header.stamp;
    pcl::fromROSMsg(*cloud_msg, *temp_cloud);
    _if_cloud = true;

    // delete noise 
    delete_noise(temp_cloud, _kinect_cloud);

    // transform z axis to up/down
    Eigen::Isometry3f T = getTransformYXZT(0, 0, 0, -90.0/180.0*M_PI, 0.0/180.0*M_PI, 0.0/180.0*M_PI);
    pcl::transformPointCloud(*_kinect_cloud, *_kinect_cloud, T.matrix());

    Eigen::Isometry3f T1 = getTransformYXZT(0, 0, 0, 0.0/180.0*M_PI, 0.0/180.0*M_PI, -90.0/180.0*M_PI);
    pcl::transformPointCloud(*_kinect_cloud, *_kinect_cloud, T1.matrix());

    _mutex.unlock();
}

// ensure the all data are from the same frame, and each frame only be processed once
inline bool global_loc::if_new_data()
{
    return _if_depth && _if_rgb;  // && _if_cloud
}

// set the msgs flag to false, prapare for next frame
inline void global_loc::reset()
{
    _if_depth = false;
    _if_rgb = false;
    _if_cloud = false;
}

// pre-process for kinect cloud: creat cloud, delete noise, pre-transform 
inline void global_loc::pre_process()
{
    // create cloud from rgb and depth image
    CloudA::Ptr init_cloud(new CloudA());
    create_cloud(_rgb_image, _depth_image, _cam_params, init_cloud);
    
    // delete the noise in kinet cloud
    delete_noise(init_cloud, _kinect_cloud);

    // pre-transform
    // transform z axis to up/down
    Eigen::Isometry3f T0 = getTransformYXZT(0, 0, 0, -90.0/180.0*M_PI, 0.0/180.0*M_PI, 0.0/180.0*M_PI);
    pcl::transformPointCloud(*_kinect_cloud, *_kinect_cloud, T0.matrix());
    // TODO: this should be used in final method
    Eigen::Isometry3f T1 = getTransformYXZT(0, 0, 0, 0.0/180.0*M_PI, 0.0/180.0*M_PI, -90.0/180.0*M_PI);  // z -90
    pcl::transformPointCloud(*_kinect_cloud, *_kinect_cloud, T1.matrix());
}

// load faro map data
void global_loc::load_faro()
{
    // load data
    pcl::io::loadPCDFile<PointA>(_faro_path, *_faro_cloud);
    Eigen::Isometry3f T = getTransformYXZT(0, 0, 0, 0.0/180.0*M_PI, 0.0/180.0*M_PI, 20.0/180.0*M_PI);
    pcl::transformPointCloud(*_faro_cloud, *_faro_cloud, T.matrix());

    // plane segmentation
    std::cout<<std::endl<<"faro plane params: "<<std::endl;
    plane p(_faro_cloud);
    p.process(_pf_cloud, _faro_params);

    // build kd-tree
    _faro_tree.setInputCloud(_faro_cloud);
}

// plane association between kinect and faro
bool global_loc::plane_match()
{
    // scores of all potential matches
    std::map<double, Eigen::Matrix4f> scores;  

    // ensure data is available in all three directions of faro and kinect
    if(_faro_params[0].size()==0 || _faro_params[1].size()==0 || _faro_params[2].size()==0)
    {
        std::cout<<"The constraints in faro is not enough !!!!"<<std::endl;
        std::cout<<_faro_params[0].size()<<"  "<<_faro_params[1].size()<<"  "<<_faro_params[2].size()<<std::endl;
        return false;
    }
    if(_kinect_params[0].size()==0 || _kinect_params[1].size()==0 || _kinect_params[2].size()==0)
    {
        std::cout<<"The constraints in kinect is not enough !!!!"<<std::endl;
        std::cout<<_kinect_params[0].size()<<"  "<<_kinect_params[1].size()<<"  "<<_kinect_params[2].size()<<std::endl;
        return false;
    }

    // try to find the correct match between all possible combination
    std::vector<std::pair<Eigen::Vector4f, Eigen::Vector4f> > matches(3);
    for(int i=0; i<_faro_params[2].size(); i++)
    {
        // z
        std::pair<Eigen::Vector4f, Eigen::Vector4f> match_z(_kinect_params[2][0], _faro_params[2][i]);
        matches[0] = match_z;

        for(int j=0; j<_faro_params[0].size(); j++)
        {
            // x
            std::pair<Eigen::Vector4f, Eigen::Vector4f> match_x(_kinect_params[0][0], _faro_params[0][j]);
            matches[1] = match_x;
    
            for(int k=0; k<_faro_params[1].size(); k++)
            {
                // y
                std::pair<Eigen::Vector4f, Eigen::Vector4f> match_y(_kinect_params[1][0], _faro_params[1][k]);
                matches[2] = match_y;

                // solve 
                Eigen::Matrix4f T;
                if( pose_solve(matches, T) )
                {
                    // score
                    double s = score(T);
                    scores[s] = T;
                }
            }
        }

        // the relationship of kinect and faro in x, y directions are confused
        for(int j=0; j<_faro_params[0].size(); j++)
        {
            // x
            std::pair<Eigen::Vector4f, Eigen::Vector4f> match_x(_kinect_params[1][0], _faro_params[0][j]);
            matches[1] = match_x;
    
            for(int k=0; k<_faro_params[1].size(); k++)
            {
                // y
                std::pair<Eigen::Vector4f, Eigen::Vector4f> match_y(_kinect_params[0][0], _faro_params[1][k]);
                matches[2] = match_y;

                // solve 
                Eigen::Matrix4f T;
                if( pose_solve(matches, T) )
                {
                    // score
                    double s = score(T);
                    scores[s] = T;
                }
            }
        }
    }

    // transform cloud
    auto it = scores.begin();
    Eigen::Matrix4f TT = it->second;
    std::cout<<"TT: "<<std::endl<<TT.inverse()<<std::endl;
    pcl::transformPointCloud(*_pk_cloud, *_pk_cloud, TT.inverse());
    pcl::transformPointCloud(*_kinect_cloud, *_kinect_cloud, TT.inverse());
    
    return true;
}

// solve pose
bool global_loc::pose_solve(const std::vector<std::pair<Eigen::Vector4f, Eigen::Vector4f> > matches, Eigen::Matrix4f &T)
{
    // r
    Eigen::Matrix3f w = Eigen::Matrix3f::Zero();
    for(int i=0; i<matches.size(); i++)
    {
        Eigen::Vector4f k = matches[i].first;
        Eigen::Vector4f f = matches[i].second;
        w += Eigen::Vector3f(k[0], k[1], k[2]) * Eigen::Vector3f(f[0], f[1], f[2]).transpose();
    }
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(w, Eigen::ComputeFullU|Eigen::ComputeFullV);
    Eigen::Matrix3f u = svd.matrixU();
    Eigen::Matrix3f v = svd.matrixV();
    Eigen::Matrix3f r = u*(v.transpose());
    // std::cout<<"r: "<<std::endl<<r<<std::endl;

    // t
    Eigen::Matrix3f a;
    Eigen::Vector3f b;
    Eigen::Vector3f t;
    for(int i=0; i<matches.size(); i++)
    {
        Eigen::Vector4f k = matches[i].first;
        Eigen::Vector4f f = matches[i].second;
        a(i, 0) = k[0];
        a(i, 1) = k[1];
        a(i, 2) = k[2];

        b[i] = k[3] - f[3];
    }
    t = -1.0 * (a.inverse() * b);
    // std::cout<<"t: "<<std::endl<<t<<std::endl;

    // generate T
    T = Eigen::Matrix4f::Identity();
    T << r(0,0), r(0,1), r(0,2), t[0],
         r(1,0), r(1,1), r(1,2), t[1],
         r(2,0), r(2,1), r(2,2), t[2],
         0, 0, 0, 1;
    // std::cout<<"T: "<<std::endl<<T<<std::endl;     

    // TODO: There are some bug in Eigen::Quaterniond q(r.cast<double>());
    // T = Eigen::Isometry3d::Identity();
    // Eigen::Quaterniond q(r.cast<double>());
    // T.rotate(q);
    // T.pretranslate(t.cast<double>()); //t.cast<double>()  Eigen::Vector3d(0,0,0)
    // std::cout<<"T: "<<std::endl<<T.matrix()<<std::endl;

    return true;    
}

// compute score of each transform
double global_loc::score(Eigen::Matrix4f T)
{
    // TODO: remove the down sample process in plane detection
    // calculate sum of the distance to the nearest point as the socre
    CloudA::Ptr temp_ds_cloud(new CloudA());
    voxel_grid(_kinect_cloud, temp_ds_cloud, 0.15);
    CloudA::Ptr trans_cloud(new CloudA());
    pcl::transformPointCloud(*temp_ds_cloud, *trans_cloud, T.inverse());
    
    double score = 0;
    for(int i=0; i<trans_cloud->points.size(); i++)
    {
        PointA p = trans_cloud->points[i];
        std::vector<int> index;
        std::vector<float> dis;
        if(_faro_tree.nearestKSearch(p, 1, index, dis) >0)
        { score += dis[0]; }
    }

    return score;
}