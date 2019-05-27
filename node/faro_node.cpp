//
// Created by liuhang on 20190519.
//

#include "plane.h"

int main(int argc, char** argv)
{
    CloudA::Ptr faro(new CloudA());
    CloudA::Ptr pc(new CloudA());
    std::cout<<"loading faro cloud ..."<<std::endl;
    pcl::io::loadPCDFile<PointA>("/home/liuhang/Documents/data/faro/216.pcd", *faro);
    std::cout<<"faro cloud loaded"<<std::endl;

    std::unordered_map<int, std::vector<Eigen::Vector4f> > gp;
    plane p(faro);
    p.process(pc, gp);

    pc->height = 1;
    pc->width = pc->points.size();
    pcl::io::savePCDFile("/home/liuhang/Documents/data/faro/plane.pcd", *pc);
    std::cout<<"plane cloud saved"<<std::endl;


    /////////
    Eigen::Vector4f vv;
    load_params("/home/liuhang/Documents/catkin_ws_kinect/src/lh_slam/params/ir.yaml", vv);


    return 0;
}