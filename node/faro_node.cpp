//
// Created by liuhang on 20190519.
//

#include "plane.h"

int main(int argc, char** argv)
{
    CloudA::Ptr faro(new CloudA());
    CloudA::Ptr pc(new CloudA());
    std::cout<<"loading faro cloud ..."<<std::endl;
    pcl::io::loadPCDFile<PointA>("/home/liuhang/Documents/data/faro/216_ds_0.1.pcd", *faro);
    std::cout<<"faro cloud loaded"<<std::endl;

    std::unordered_map<int, std::vector<Eigen::Vector4f> > gp;
    plane p(faro);
    p.process(pc, gp);

    save_cloud("/home/liuhang/Documents/data/faro/plane.pcd", *pc);


    //
    Eigen::Vector4f params;
    load_params("/home/liuhang/Documents/catkin_ws_kinect/src/lh_slam/params/ir.yaml", params);

    return 0;
}