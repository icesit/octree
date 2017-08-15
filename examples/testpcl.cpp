#include <iostream>
#include <cstdlib>
#include <time.h>

#include "../Octree.hpp"
#include "utils.h"

#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
//#include <pcl_conversions/pcl_conversions.h>

float start[3] = {-40, 40, -1};
float end[3] = {40, -40, -1};
float mid[3] = {(start[0]+end[0])/2, (start[1]+end[1])/2, (start[2]+end[2])/2};
float halfdist = 0.75 * sqrt(pow(start[0]-end[0],2) + pow(start[1]-end[1],2) + pow(start[2]-end[2],2));

//calculate potantial for blocks method
double calPotantialBlock(pcl::PointXYZRGB& center, std::vector<float_t*> blocks)
{
    double x,y,z,potan_t,potan=0;
    //if(blocks.size()>0) potan = 100;
    for(int i=0; i<blocks.size(); ++i)
    {
        if(blocks[i][3]>0.1){
            std::cout << "block size " << blocks[i][3] << ",block loca("
                      << blocks[i][0] << "," << blocks[i][1] << ","
                      << blocks[i][2] << ")" << std::endl;
            std::cout << "point is(" << center.x << "," << center.y << ","
                      << center.z << ")" << std::endl;
        }
        x = abs(center.x-blocks[i][0]);//-blocks[i][3];
        y = abs(center.y-blocks[i][1]);//-blocks[i][3];
        z = abs(center.z-blocks[i][2]);//-blocks[i][3];
        potan_t = sqrt(pow(x,2) + pow(y,2) + pow(z,2));
        //if in the box
        if(potan_t <= blocks[i][3])
        {
            potan = 1;
            break;
        }
        potan_t = 1 / (1 + 1*exp(potan_t-1));
        //poten += poten_t;
        if(potan < potan_t) potan = potan_t;
    }

    return potan;
}

//calculate potential for point method
double calPotantial(pcl::PointXYZRGB& center, std::vector<uint32_t>& resultIndices, std::vector<float>& distances)
{
    double mindist = 100, mini;
    for(int i = 0; i < resultIndices.size(); ++i)
    {
        if(mindist > distances[i])
        {
            mindist = distances[i];
            mini = i;
        }
    }

    double z,y,x;
    mindist = 1 / (1 + 1*exp(sqrt(mindist)-1));
    //potential from start
    x = pow(start[0]-center.x,2);
    y = pow(start[1]-center.y,2);
    z = pow(start[2]-center.z,2);
    //using sigmoid
    mindist += 1 / (1 + exp((sqrt(x+y+z)-halfdist)/20));
    //potential from end
    x = pow(end[0]-center.x,2);
    y = pow(end[1]-center.y,2);
    z = pow(end[2]-center.z,2);
    //using sigmoid
    mindist = mindist - 1 / (1 + exp((sqrt(x+y+z)-halfdist)/20));
    return mindist;
}

int main(int argc, char** argv)
{
    if (argc < 3)
    {
      std::cerr << "filename of point cloud missing." << std::endl;
      return -1;
    }
    std::string filename = argv[1];
    std::string filename2 = argv[2];

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    readPoints2<pcl::PointXYZRGB>(filename, filename2, pCloud->points);
    std::cout << "Read " << pCloud->points.size() << " points." << std::endl;
    if (pCloud->points.size() == 0)
    {
      std::cerr << "Empty point cloud." << std::endl;
      return -1;
    }

    //test initialize
    int64_t begin, end;
    begin = clock();
    unibn::Octree<pcl::PointXYZRGB> octree;
//    unibn::OctreeParams pp(32, false, 0.1f);
//    unibn::OctreeParams& params = pp;
    const unibn::OctreeParams& params = unibn::OctreeParams(32, false, 0.01f);
    octree.initialize(pCloud->points, params);
    end = clock();
    double search_time = ((double)(end - begin) / CLOCKS_PER_SEC);
    std::cout << "initialize time is" << search_time << std::endl;
    std::cout << "params are " << params.bucketSize << "," << params.minExtent << std::endl;

    //test find points in ball
    begin = clock();
    pcl::PointXYZRGB& tmp = pCloud->points[0];
    std::vector<uint32_t> results;
    octree.radiusNeighbors<unibn::L2Distance<pcl::PointXYZRGB> >(tmp, 5.0f, results);
    end = clock();
    search_time = ((double)(end - begin) / CLOCKS_PER_SEC);
    std::cout << results.size() << " radius neighbors (r = 5.0m) found for ("
              << tmp.x << ", " << tmp.y << "," << tmp.z << ")" << std::endl;
    std::cout << "Searching for all radius neighbors (r = 5.0m) took "
              << search_time << " seconds." << std::endl;

    //test find occupied block in ball
    begin = clock();
    std::vector<float_t*> blo;
    octree.radiusNeighborsBlock<unibn::L2Distance<pcl::PointXYZRGB> >(tmp, 5.0f, blo);
    end = clock();
    search_time = ((double)(end - begin) / CLOCKS_PER_SEC);
    std::cout << blo.size() << " radius block neighbors (r = 5.0m) found for ("
              << tmp.x << ", " << tmp.y << "," << tmp.z << ")" << std::endl;
    std::cout << "Searching for all radius block neighbors (r = 5.0m) took "
              << search_time << " seconds." << std::endl;

    //test calculate potantial
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr potanCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB pt;
    pt.z = -1;
    double po;
    double maxpo = -1000, minpo = 1000, maxblocknum = 0;
    std::vector<float> dist;
    begin = clock();
    for(float i = -50; i < 50; i+=0.1)
    {
        for(float j = -50; j < 50; j+=0.1)
        {
            //std::cout << "calculating " << i << "," << j << std::endl;
            pt.x = i;
            pt.y = j;
            octree.radiusNeighbors<unibn::L2Distance<pcl::PointXYZRGB> >(pt, 5.0f, results, dist);
            po = calPotantial(pt,results,dist);
            if(po>0)
            {
                pt.r = po*150;
                pt.g = 0;
                pt.b = 0;//255 - pt.r;
            }
            else
            {
                pt.r = 0;
                pt.g = -po*255;
                pt.b = 0;
            }
            potanCloud->push_back(pt);
            if(blo.size() > maxblocknum) maxblocknum = blo.size();
            if(po > maxpo) maxpo = po;
            if(po < minpo) minpo = po;
        }
    }
    end = clock();
    search_time = ((double)(end - begin) / CLOCKS_PER_SEC);
    std::cout << "Searching for all radius block neighbors (r = 5.0m) for all points took "
              << search_time << " seconds." << std::endl;
    std::cout << "max potantial " << maxpo << ", min potantial " << minpo << std::endl;
    std::cout << "max block num " << maxblocknum << std::endl;

    //pcl::visualization::CloudViewer poviewer("Potantial Viewer");
    //pcl::visualization::CloudViewer pclviewer("Cloud Viewer");

    std::cout << "show point cloud" << std::endl;
    int v1(0), v2(0);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters();

    viewer->createViewPort(0.0,0.0,0.5,1.0,v1);
    viewer->setBackgroundColor (255, 255, 255, v1);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(potanCloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (potanCloud, rgb, "potantial cloud", v1);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "potantial cloud", v1);
    viewer->addCoordinateSystem (1.0, "potantial cloud", v1);

    viewer->createViewPort(0.5,0.0,1.0,1.0,v2);
    viewer->setBackgroundColor (255, 255, 255, v2);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2(pCloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (pCloud, rgb2, "origin cloud", v2);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "origin cloud", v2);
    viewer->addCoordinateSystem (1.0, "origin cloud", v2);
    while(1)
    {
        viewer->spinOnce (100);
        //poviewer.showCloud(potanCloud);
        //pclviewer.showCloud(pCloud);
    }

    return 0;
}
