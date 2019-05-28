// 
// Created by liuhang on 20190404.
// 

#include "feature.h"

feature::feature()
    :_cloud(new CloudA()), _ds_cloud(new CloudA()), 
     _plane_cloud(new CloudA()), _T_plane_cloud(new CloudA()),
     _faro_cloud(new CloudA()), _pf_cloud(new CloudA())
{
    _if_cloud = false;
    _if_rgb_image = false;
    _if_depth_image = false;
    _if_ir_image = false;
    _if_first_frame = true;
    
    _msgs_index = 0;

    faro();
}

feature::~feature()
{ }

// set the params, subscribe and publish msgs
bool feature::init(ros::NodeHandle &node, ros::NodeHandle &private_node)
{
    // load params
    private_node.getParam("skip_count", _skip_count);

    // subscribe and publish msgs
    // _sub_rgb = node.subscribe<sensor_msgs::CompressedImage>("/kinect2/sd/image_color_rect/compressed", 10, &feature::rgb_handler, this);
    // _sub_ir = node.subscribe<sensor_msgs::CompressedImage>("/kinect2/sd/image_ir/compressed", 10, &feature::ir_handler, this);
    // _sub_depth = node.subscribe<sensor_msgs::CompressedImage>("/kinect2/sd/image_depth/compressed", 10, &feature::depth_handler, this);
    // _pub_image = node.advertise<sensor_msgs::CompressedImage>("/match_image// compressed", 10);
    _sub_cloud = node.subscribe<sensor_msgs::PointCloud2>("/kinect2/sd/points", 10, &feature::cloud_handler, this);
    _pub_cloud = node.advertise<sensor_msgs::PointCloud2>("/plane_cloud", 10);
    _pub_faro = node.advertise<sensor_msgs::PointCloud2>("/faro_cloud", 10);
    _pub_kinect = node.advertise<sensor_msgs::PointCloud2>("/kinect", 10);

    // start the process thread
    _process_thread = std::thread(&feature::spin, this);

    return true;
}

// the process of this node
void feature::spin()
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
void feature::process()
{
    // if there are no new data, return 
    if(!if_new_data())
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

    // find the corrspondence frame in rgb_image_map for cloud frame
    //  if(!data_match())
    //  { return; }

    //  detect 2D features in rgb image, and macth two images
    // for rgb images
    //  bool b = feature_detect(_last_rgb_image, _rgb_image, _last_rgb_keypoints, _rgb_keypoints,
    //                          _last_rgb_descriptor, _rgb_descriptor,
    //                          _rgb_matches, _match_image, _if_first_frame);

    //  for ir images
    //  bool b = feature_detect(_last_ir_image, _ir_image, _last_ir_keypoints, _ir_keypoints, 
    //                          _last_ir_descriptor, _ir_descriptor,
    //                          _ir_matches, _match_image, _if_first_frame);
    
    //  if(!b)
    //  { return; }



    std::cout<<"kinect plane params: "<<std::endl;
    _plane_cloud->clear();
    plane p(_cloud);
    p.process(_plane_cloud, _kinect_gp);
    
    // match();
    if( !plane_match() )
    { return; }









    //  publish msgs
    publish_msgs();
}

// the recall handler of cloud msgs
inline void feature::cloud_handler(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)
{
    _mutex.lock();

     CloudA::Ptr temp_cloud(new CloudA());

    // _cloud->clear();
    ros::Time time = ros::Time::now();
    _cloud_time = cloud_msg->header.stamp;
    pcl::fromROSMsg(*cloud_msg, *temp_cloud);
    _if_cloud = true;

    // delete noise 
    delete_noise(temp_cloud, _cloud);

    // transform z axis to up/down
    Eigen::Isometry3f T = getTransformYXZT(0, 0, 0, -90.0/180.0*M_PI, 0.0/180.0*M_PI, 0.0/180.0*M_PI);
    pcl::transformPointCloud(*_cloud, *_cloud, T.matrix());

    Eigen::Isometry3f T1 = getTransformYXZT(0, 0, 0, 0.0/180.0*M_PI, 0.0/180.0*M_PI, -90.0/180.0*M_PI);
    pcl::transformPointCloud(*_cloud, *_cloud, T1.matrix());

    // Eigen::Isometry3f T2 = getTransformYXZT(0, 0, 0, 0.0/180.0*M_PI, -25.0/180.0*M_PI, 0.0/180.0*M_PI);
    // pcl::transformPointCloud(*_cloud, *_cloud, T2.matrix());

    
    // save_cloud("/home/liuhang/Documents/data/faro/kinect.pcd", *_cloud);

    _mutex.unlock();
}

// the recall handler of rgb msgs
inline void feature::rgb_handler(const sensor_msgs::CompressedImage::ConstPtr &rgb_msg)
{
    _mutex.lock();

    try
    { 
        //  // for rbg image "bgr8" or "8UC3"; for depth image "16UC1" or "mono16"; for ir image "16UC1" or "8UC1"
        //  // "toCvCopy" or "toCvShare"
        ros::Time rgb_time = rgb_msg->header.stamp;
        cv_bridge::CvImagePtr rgb_image = cv_bridge::toCvCopy(rgb_msg, "bgr8");
        
        _rgb_bridge_map[rgb_time] = rgb_image;
        if(_rgb_bridge_map.size() > 8)
        { _rgb_bridge_map.erase(_rgb_bridge_map.begin()); }
        
        //  save images
        //  double t = rgb_time.toSec();
        //  std::string file_path = "/home/liuhang/data/calibration/" + std::to_string(t) + "_rgb.png";
        //  cv::imwrite(file_path, rgb_image->image);
        //  std::cout<<rgb_time<<"     rgb image saved!"<<std::endl;
        //  std::cout<<rgb_image->image.rows<<"  "<<rgb_image->image.cols<<std::endl;
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    _mutex.unlock();
}

// the recall handler of depth msgs
inline void feature::depth_handler(const sensor_msgs::CompressedImage::ConstPtr &depth_msg)
{
    _mutex.lock();

    try
    { 
        // for rbg image "bgr8" or "8UC3"; for depth image "16UC1"; for ir image "16UC1" or "8UC1"
        // "toCvCopy" or "toCvShare"
        _depth_time = depth_msg->header.stamp;
        _depth_bridge = cv_bridge::toCvCopy(depth_msg, "16UC1");
        _if_depth_image = true;

        //  save images
        //  double t = _depth_time.toSec();
        //  std::string file_path = "/home/liuhang/data/calibration/" + std::to_string(t) + "_depth.png";
        //  cv::imwrite(file_path, _depth_bridge->image);
        //  std::cout<<_depth_time<<"     depth image saved!"<<std::endl;
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    _mutex.unlock();
}

// the recall handler of ir msgs
inline void feature::ir_handler(const sensor_msgs::CompressedImage::ConstPtr &ir_msg)
{
    _mutex.lock();

    try
    {
        _ir_time = ir_msg->header.stamp;
        _ir_bridge = cv_bridge::toCvCopy(ir_msg, "8UC1");
        _if_ir_image = true;

        gray_stretch();
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    _mutex.unlock();
}

// publish msgs
inline void feature::publish_msgs()
{
    // publish_image_msg(_pub_image, _match_image, _cloud_time, "kinect2_ir_optical_frame", "bgr8");
    
    publish_cloud_msg<PointA>(_pub_cloud, _plane_cloud, _cloud_time, "kinect2_ir_optical_frame");
    publish_cloud_msg<PointA>(_pub_faro, _faro_cloud, _cloud_time, "kinect2_ir_optical_frame");
    publish_cloud_msg<PointA>(_pub_kinect, _cloud, _cloud_time, "kinect2_ir_optical_frame");
}

// ensure the all data are from the same frame, and each frame only be processed once
inline bool feature::if_new_data()
{
    //  std::cout<<fabs((_depth_time - _rgb_time).toSec())<<std::endl;
    //  return _if_rgb_image && _if_depth_image &&
    //         fabs((_cloud_time - _rgb_time).toSec()) < 0.05;  // 0.3
    //         fabs((_depth_time - _rgb_time).toSec()) <0.05;

    return _if_cloud;
    //  && _if_ir_image;
}

// set the msgs flag to false, prapare for next frame
inline void feature::reset()
{
    _if_cloud = false;
    _if_rgb_image = false;
    _if_depth_image = false;
    _if_ir_image = false;

    _last_rgb_image = _rgb_image.clone();
    _last_rgb_keypoints = _rgb_keypoints;
    _last_rgb_descriptor = _rgb_descriptor.clone();

    _last_ir_image = _ir_image.clone();
    _last_ir_keypoints = _ir_keypoints;
    _last_ir_descriptor = _ir_descriptor.clone();
}

// search corresponding frame in rgb_image_map for cloud frame 
inline bool feature::data_match()
{
    auto it = _rgb_bridge_map.find(_cloud_time);
    if(it == _rgb_bridge_map.end())
    { return false; }

    _rgb_image = it->second->image;
    return true;
}

// feature detect
bool feature::feature_detect(cv::Mat &last_image, cv::Mat &image, 
                             std::vector<cv::KeyPoint> &last_keypoints, std::vector<cv::KeyPoint> &keypoints,
                             cv::Mat &last_descriptor, cv::Mat &descriptor,
                             std::vector<cv::DMatch> &matches, cv::Mat &match_image,
                             bool &if_first_frame)
{
     matches.clear();

    // feature deatect in OpenCV2.x
    //  cv::Ptr<cv::FeatureDetector> detector = cv::FeatureDetector::create("ORB");

    // feature detect in OpenCV3.X, the contrib is needed for SIFT and SURF
    //  cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create();  // 180ms for qhd size
    //  cv::Ptr<cv::xfeatures2d::SIFT> detector = cv::xfeatures2d::SIFT::create();  // 300ms for qhd size
    cv::Ptr<cv::ORB> detector = cv::ORB::create();  // 30ms for qhd size

    if(if_first_frame)
    {
        last_image = image.clone();
        //  detector->detect(last_image, last_keypoints);
        detect(detector, last_image, 250, last_keypoints);
        detector->compute(last_image, last_keypoints, last_descriptor);
        if_first_frame = false;
        return false;
    }

    //  detector->detect(image, keypoints);
    detect(detector, image, 250, keypoints);
    detector->compute(image, keypoints, descriptor);

    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<cv::DMatch> temp_matches;
    matcher.match(last_descriptor, descriptor, temp_matches);

    double min_dist = 1000, max_dist = 0;
    for(int i=0; i<temp_matches.size(); i++)
    {
        double dist = temp_matches[i].distance;
        if(dist < min_dist) min_dist = dist;
        if(dist > max_dist) max_dist = dist;
    }
    for(int i=0; i<temp_matches.size(); i++)
    {
        if(temp_matches[i].distance <= std::max(2*min_dist, 30.0))
        { matches.push_back(temp_matches[i]); }  
    }
    cv::drawMatches(last_image, last_keypoints, image, keypoints,
                    matches, match_image);

    //  std::cout<<"kp count: "<<keypoints.size()<<std::endl;
    //  std::cout<<"temp_matches count: "<<temp_matches.size()<<std::endl;                
    //  std::cout<<"matches count: "<<matches.size()<<std::endl;                
    //  cv::drawKeypoints(image, keypoints, _keypoints_image, 
    //                cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);    

    //  std::string path = "/home/liuhang/Documents/" + std::to_string(_msgs_index) + ".png";
    //  cv::imwrite(path, image);

    return true;
}

// stretch the gray scale of ir image to ensure feature extract process
void feature::gray_stretch()
{
    // statistic the gray value
    _ir_image = _ir_bridge->image.clone();
    std::map<int, int> sta;  
    int rows = _ir_image.rows;
    int cols = _ir_image.cols;
    for(int i=0; i<rows; i++)
    {
        for(int j=0; j<cols; j++)
        {
            // uchar/8UC1, uchar/16UC1
            int v = (int)_ir_image.at<uchar>(i,j);
            sta[v]++;
        }
    }

    // calculate the min and max of 0.5%
    double persent = 0;
    int min=0, max=0;
    bool if_min=false, if_max=false;
    for(auto it : sta)
    {
        double persent_temp = (it.second*1.0) / (rows*cols) * 100;
        persent += persent_temp;

        if(!if_min && persent>0.8)
        {
            min = it.first;
            if_min = true;
            //  std::cout<<"min "<<min<<std::endl;
        }
        else if(!if_max && persent>99.2)
        {
            max = it.first;
            if_max = true;
            //  std::cout<<"max "<<max<<std::endl;
            break;
        }
    }

    // strectch the gray scale
    for(int i=0; i<rows; i++)
    {
        for(int j=0; j<cols; j++)
        {
            int v = (int)_ir_image.at<uchar>(i,j);
            if(v <= min)
            { _ir_image.at<uchar>(i,j) = 0; }
            else if(v >= max)
            { _ir_image.at<uchar>(i,j) = 255; }
            else
            { _ir_image.at<uchar>(i,j) = (255/(max-min))*(_ir_image.at<uchar>(i,j)-min); } 
        }
    }
}




// load faro cloud 
void feature::faro()
{
    // load data
    pcl::io::loadPCDFile<PointA>("/home/liuhang/Documents/data/faro/216_ds_0.1.pcd", *_faro_cloud);
    Eigen::Isometry3f T = getTransformYXZT(0, 0, 0, 0.0/180.0*M_PI, 0.0/180.0*M_PI, 20.0/180.0*M_PI);
    pcl::transformPointCloud(*_faro_cloud, *_faro_cloud, T.matrix());

    // plane segmentation
    std::cout<<"faro plane params: "<<std::endl;
    plane p(_faro_cloud);
    p.process(_pf_cloud, _faro_gp);

    // build kd-tree
    _tree.setInputCloud(_faro_cloud);
}

// plane association between kinect and faro
bool feature::plane_match()
{
    _scores.clear();


    //
    std::cout<<"!!!!!!!!!!!!!!!"<<std::endl;
    std::cout<<_faro_gp[0].size()<<"  "<<_faro_gp[1].size()<<"  "<<_faro_gp[2].size()<<std::endl;
    std::cout<<_kinect_gp[0].size()<<"  "<<_kinect_gp[1].size()<<"  "<<_kinect_gp[2].size()<<std::endl;

    // ensure data is available in all three directions of faro and kinect
    if(_faro_gp[0].size()==0 || _faro_gp[1].size()==0 || _faro_gp[2].size()==0)
    {
        std::cout<<"The constraints in faro is not enough !!!!"<<std::endl;
        return false;
    }
    if(_kinect_gp[0].size()==0 || _kinect_gp[1].size()==0 || _kinect_gp[2].size()==0)
    {
        std::cout<<"The constraints in kinect is not enough !!!!"<<std::endl;
        return false;
    }

    // try to find the correct match between all possible combination
    int num = 0;
    std::vector<std::pair<Eigen::Vector4f, Eigen::Vector4f> > matches(3);
    for(int i=0; i<_faro_gp[2].size(); i++)
    {
        // z
        std::pair<Eigen::Vector4f, Eigen::Vector4f> match_z(_kinect_gp[2][0], _faro_gp[2][i]);
        matches[0] = match_z;

        for(int j=0; j<_faro_gp[0].size(); j++)
        {
            // x
            std::pair<Eigen::Vector4f, Eigen::Vector4f> match_x(_kinect_gp[0][0], _faro_gp[0][j]);
            matches[1] = match_x;
    
            for(int k=0; k<_faro_gp[1].size(); k++)
            {
                // y
                std::pair<Eigen::Vector4f, Eigen::Vector4f> match_y(_kinect_gp[1][0], _faro_gp[1][k]);
                matches[2] = match_y;

                // solve 
                Eigen::Matrix4f T;
                if( pose_solve(matches, T) )
                {
                    // score
                    double s = score(T);
                    _scores[s] = T;

                    // // transform cloud
                    // CloudA::Ptr temp(new CloudA());
                    // pcl::transformPointCloud(*_plane_cloud, *temp, T.inverse().matrix());
                    // std::string path = "/home/liuhang/Documents/data/faro/k" + std::to_string(num) + ".pcd";
                    // save_cloud(path, *temp);
                    // num++;
                }
            }
        }

        // the relationship of kinect and faro in x, y directions are confused
        for(int j=0; j<_faro_gp[0].size(); j++)
        {
            // x
            std::pair<Eigen::Vector4f, Eigen::Vector4f> match_x(_kinect_gp[1][0], _faro_gp[0][j]);
            matches[1] = match_x;
    
            for(int k=0; k<_faro_gp[1].size(); k++)
            {
                // y
                std::pair<Eigen::Vector4f, Eigen::Vector4f> match_y(_kinect_gp[0][0], _faro_gp[1][k]);
                matches[2] = match_y;

                // solve 
                Eigen::Matrix4f T;
                if( pose_solve(matches, T) )
                {
                    // score
                    double s = score(T);
                    _scores[s] = T;

                    // // transform cloud
                    // CloudA::Ptr temp(new CloudA());
                    // pcl::transformPointCloud(*_plane_cloud, *temp, T.inverse().matrix());
                    // std::string path = "/home/liuhang/Documents/data/faro/k" + std::to_string(num) + ".pcd";
                    // save_cloud(path, *temp);
                    // num++;
                }
                
            }
        }
    }

    // transform cloud
    auto it = _scores.begin();
    Eigen::Matrix4f TT = it->second;
    std::cout<<"TT: "<<std::endl<<TT.inverse()<<std::endl;
    pcl::transformPointCloud(*_plane_cloud, *_plane_cloud, TT.inverse());
    pcl::transformPointCloud(*_cloud, *_cloud, TT.inverse());
    
    return true;
}

// solve pose
bool feature::pose_solve(std::vector<std::pair<Eigen::Vector4f, Eigen::Vector4f> > matches, Eigen::Matrix4f &T)
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
    // There are some bug in Eigen::Quaterniond q(r.cast<double>());
    T = Eigen::Matrix4f::Identity();
    T << r(0,0), r(0,1), r(0,2), t[0],
         r(1,0), r(1,1), r(1,2), t[1],
         r(2,0), r(2,1), r(2,2), t[2],
         0, 0, 0, 1;
    // std::cout<<"T: "<<std::endl<<T<<std::endl;     

    // T = Eigen::Isometry3d::Identity();
    // Eigen::Quaterniond q(r.cast<double>());
    // T.rotate(q);
    // T.pretranslate(t.cast<double>()); //t.cast<double>()  Eigen::Vector3d(0,0,0)
    // std::cout<<"T: "<<std::endl<<T.matrix()<<std::endl;

    return true;    
}

// compute score of each transform
double feature::score(Eigen::Matrix4f T)
{
    // calculate sum of the distance to the nearest point as the socre
    CloudA::Ptr temp_ds_cloud(new CloudA());
    voxel_grid(_cloud, temp_ds_cloud, 0.15);
    CloudA::Ptr trans_cloud(new CloudA());
    pcl::transformPointCloud(*temp_ds_cloud, *trans_cloud, T.inverse());
    
    double score = 0;
    for(int i=0; i<trans_cloud->points.size(); i++)
    {
        PointA p = trans_cloud->points[i];
        std::vector<int> index;
        std::vector<float> dis;
        if(_tree.nearestKSearch(p, 1, index, dis) >0)
        { score += dis[0]; }
    }

    return score;
}



/*
// match kinect and faro
void feature::match()
{
    CloudI::Ptr kinect(new CloudI());
    CloudI::Ptr faro(new CloudI());
    eigen2pcl(_kinect_params, kinect);
    eigen2pcl(_faro_params, faro);

    // find nearest normal vector
    std::vector<std::pair<int, int> > matches;
    pcl::KdTreeFLANN<PointI> tree;
    tree.setInputCloud(faro);
    for(int i=0; i<kinect->points.size(); i++)
    {
        PointI p = kinect->points[i];

        std::vector<int> index;
        std::vector<float> dis;
        if(tree.nearestKSearch(p, 1, index, dis) > 0)
        {
             std::pair<int, int> match(i, index[0]);
             matches.push_back(match);
        }
    }

    // cout
    std::cout<<std::endl<<std::endl<<std::endl<<"matches: "<<std::endl;
    for(int i=0; i<matches.size(); i++)
    {
        int k = matches[i].first;
        int f = matches[i].second;
        
        std::cout<<kinect->points[k].x<<"  "<<kinect->points[k].y<<"  "
                 <<kinect->points[k].z<<"  "<<kinect->points[k].intensity<<std::endl;
    
        std::cout<<faro->points[f].x<<"  "<<faro->points[f].y<<"  "
                 <<faro->points[f].z<<"  "<<faro->points[f].intensity<<std::endl<<std::endl;
    }

    // solve pose
    // r
    Eigen::Matrix3d w = Eigen::Matrix3d::Zero();
    for(int i=0; i<matches.size(); i++)
    {
        int k = matches[i].first;
        int f = matches[i].second;
        PointI kp = kinect->points[k];
        PointI fp = faro->points[f];

        w += Eigen::Vector3d(kp.x, kp.y, kp.z) * Eigen::Vector3d(fp.x, fp.y, fp.z).transpose();
    }
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(w, Eigen::ComputeFullU|Eigen::ComputeFullV);
    Eigen::Matrix3d u = svd.matrixU();
    Eigen::Matrix3d v = svd.matrixV();
    Eigen::Matrix3d r = u*(v.transpose());
    std::cout<<std::endl<<"r: "<<std::endl<<r<<std::endl;

    // t 
    // Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> a(3, 3);
    // Eigen::VectorXd b(3);
    // Eigen::Vector3d t;
    // for(int i=0; i<3; i++)
    // {
    //     int k = matches[i].first;
    //     int f = matches[i].second;
    //     PointI kp = kinect->points[k];
    //     PointI fp = faro->points[f];

    //     a(i, 0) = fp.x;
    //     a(i, 1) = fp.y;
    //     a(i, 2) = fp.z;
    //     b(i, 0) = fp.intensity - kp.intensity;
    // }
    // t = a.inverse() * b;
    // std::cout<<std::endl<<"t: "<<std::endl<<t<<std::endl;

    // t
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> a(matches.size(), 3);
    Eigen::VectorXd b(matches.size());
    Eigen::Vector3d t;
    for(int i=0; i<matches.size(); i++)
    {
        int k = matches[i].first;
        int f = matches[i].second;
        PointI kp = kinect->points[k];
        PointI fp = faro->points[f];

        a(i, 0) = kp.x;
        a(i, 1) = kp.y;
        a(i, 2) = kp.z;
        b(i, 0) = kp.intensity - fp.intensity;
    }
    t = (a.transpose()*a).inverse()*a.transpose()*b;
    std::cout<<std::endl<<"t: "<<std::endl<<t<<std::endl;

    // transform cloud
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity(); 
    Eigen::Quaterniond q(r); 
    T.pretranslate(Eigen::Vector3d(0, 0, 0));  //Eigen::Vector3d(0, 0, 0)
    T.rotate(q);
    std::cout<<std::endl<<"T: "<<std::endl<<T.matrix()<<std::endl;

    pcl::transformPointCloud(*_plane_cloud, *_plane_cloud, T.inverse().matrix());
}
*/
