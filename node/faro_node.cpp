//
// Created by liuhang on 20190519.
//

#include "plane.h"

int main(int argc, char** argv)
{
    CloudA::Ptr faro(new CloudA());
    CloudA::Ptr pc(new CloudA());
    std::cout<<"loading faro.pcd ..."<<std::endl;
    pcl::io::loadPCDFile<PointA>("/home/liuhang/Desktop/faro.pcd", *faro);
    std::cout<<"faro.pcd loaded"<<std::endl;

    plane p(faro);
    p.process(pc);

    pc->height = 1;
    pc->width = pc->points.size();
    pcl::io::savePCDFile("/home/liuhang/Desktop/pc.pcd", *pc);

    std::cout<<"hello world."<<std::endl;
    return 0;
}