#include <iostream>
#include <cstdlib>
#include <time.h>

#include "../Octree.hpp"
#include "utils.h"

#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
//#include <pcl_conversions/pcl_conversions.h>

//float start[3] = {-40, 40, -1};
//float end[3] = {40, -40, -1};
float start[3] = {0, 0, 1};
float end[3] = {8, 2.5, 1};
float mid[3] = {(start[0]+end[0])/2, (start[1]+end[1])/2, (start[2]+end[2])/2};
float halfdist = 0.5 * sqrt(pow(start[0]-end[0],2) + pow(start[1]-end[1],2) + pow(start[2]-end[2],2));
float startFactor = 0.5;
float endFactor = 1.5;

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
    mindist = 1 / (1 + 1*exp((sqrt(mindist)-1)/0.5));
    //potential from start
    x = pow(start[0]-center.x,2);
    y = pow(start[1]-center.y,2);
    z = pow(start[2]-center.z,2);
    //using sigmoid
    mindist += 1 / (1 + exp((sqrt(x+y+z)-startFactor*halfdist)/20));
    //potential from end
    x = pow(end[0]-center.x,2);
    y = pow(end[1]-center.y,2);
    z = pow(end[2]-center.z,2);
    //using sigmoid
    mindist = mindist - 1 / (1 + exp((sqrt(x+y+z)-endFactor*halfdist)/20));
    return mindist;
}

float calPotenFromCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr poten, int i, int j)
{
    float po;
    po = float(poten->points[i*1001+j].r)/150.0 - float(poten->points[i*1001+j].g)/255.0;
    return po;
}

//find a path in potential field
bool findPath(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& poten, double *po)
{
    int curX = start[0]/0.1+500, curY = start[1]/0.1+500;//, curZ = start[2]/0.1;
    int nextX, nextY;//, nextZ;
    double curMinPoErr=10, poErr, curPoten;
    int step = 0;
//    double po[1000][1000];
//    for(int i=0; i<500; ++i)
//    {
//        for(int j=0; j<500; ++j)
//        {
//            po[i][j] = poten->points[i*1000+j];
//        }
//    }
    std::cout << "start potential " << po[1000*curX+curY]
              << ", end potential " << po[int(1000*(end[0]/0.1+500)+end[1]/0.1+500)] <<std::endl;
    while(fabs(curX-end[0]/0.1-500)>1.5 || fabs(curY-end[1]/0.1-500)>1.5)// || (curZ-end[2]/0.1)>0.5)
    {
        curMinPoErr=0;
        curPoten = po[1001*curX+curY];
        //cal poten err,find minimum direction
        poErr = po[1001*(curX-1)+curY] - curPoten;
        if(curMinPoErr > poErr)
        {
            curMinPoErr = poErr;
            nextX = curX-1;
            nextY = curY;
        }
        poErr = po[1001*curX+curY-1] - curPoten;
        if(curMinPoErr > poErr)
        {
            curMinPoErr = poErr;
            nextX = curX;
            nextY = curY-1;
        }
        poErr = po[1001*(curX+1)+curY] - curPoten;
        if(curMinPoErr > poErr)
        {
            curMinPoErr = poErr;
            nextX = curX+1;
            nextY = curY;
        }
        poErr = po[1001*curX+curY+1] - curPoten;
        if(curMinPoErr > poErr)
        {
            curMinPoErr = poErr;
            nextX = curX;
            nextY = curY+1;
        }

        poErr = (po[1001*(curX-1)+curY+1] - curPoten) / 1.414;
        if(curMinPoErr > poErr)
        {
            curMinPoErr = poErr;
            nextX = curX-1;
            nextY = curY+1;
        }
        poErr = (po[1001*(curX-1)+curY-1] - curPoten) / 1.414;
        if(curMinPoErr > poErr)
        {
            curMinPoErr = poErr;
            nextX = curX-1;
            nextY = curY-1;
        }
        poErr = (po[1001*(curX+1)+curY+1] - curPoten) / 1.414;
        if(curMinPoErr > poErr)
        {
            curMinPoErr = poErr;
            nextX = curX+1;
            nextY = curY+1;
        }
        poErr = (po[1001*(curX+1)+curY-1] - curPoten) / 1.414;
        if(curMinPoErr > poErr)
        {
            curMinPoErr = poErr;
            nextX = curX+1;
            nextY = curY-1;
        }

        if(curMinPoErr < 0)
        {
            curX = nextX;
            curY = nextY;
            poten->points[curX*1001+curY].b = 255;
            poten->points[curX*1001+curY].r = 0;
            poten->points[curX*1001+curY].g = 0;
            ++step;
            std::cout << "way point" << float(curX)/10-50 << "," << float(curY)/10-50 << std::endl;
            std::cout << "cur poten " << curPoten
                      << ", rest poten " << po[1001*curX+curY+1]
                      << "," << po[1001*curX+curY-1]
                      << "," << po[1001*(curX+1)+curY]
                      << "," << po[1001*(curX-1)+curY] <<std::endl;
        }
        else
        {
            std::cout << "to local minima!" << std::endl;
            std::cout << "go " << step << " steps" << std::endl;
            std::cout << "cur poten " << curPoten
                      << ", rest poten " << po[1001*curX+curY+1]
                      << "," << po[1001*curX+curY-1]
                      << "," << po[1001*(curX+1)+curY]
                      << "," << po[1001*(curX-1)+curY] <<std::endl;
            return false;
        }
    }

    std::cout << "to the end successfully!" << std::endl;
    return true;
}

int main(int argc, char** argv)
{
    if (argc < 2)//3
    {
      std::cerr << "filename of point cloud missing." << std::endl;
      return -1;
    }
    std::string filename = argv[1];
    //std::string filename2 = argv[2];

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    //readPoints2<pcl::PointXYZRGB>(filename, filename2, pCloud->points);
    readPoints<pcl::PointXYZRGB>(filename, pCloud->points);
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
    std::vector<uint32_t> results;
    for(int i=0; i<27; ++i){
    pcl::PointXYZRGB& tmp = pCloud->points[i];
    octree.radiusNeighbors<unibn::L2Distance<pcl::PointXYZRGB> >(tmp, 5.0f, results);
    }
    end = clock();
    search_time = ((double)(end - begin) / CLOCKS_PER_SEC);
//    std::cout << results.size() << " radius neighbors (r = 5.0m) found for ("
//              << tmp.x << ", " << tmp.y << "," << tmp.z << ")" << std::endl;
    std::cout << "Searching for all radius neighbors (r = 5.0m) took "
              << search_time << " seconds." << std::endl;

    //test find occupied block in ball
    begin = clock();
    std::vector<float_t*> blo;
    pcl::PointXYZRGB& tmp = pCloud->points[11];
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
    pt.z = 1;
    double po;
    double maxpo = -1000, minpo = 1000, maxblocknum = 0;
    std::vector<float> dist;
    begin = clock();
    double poten[1000*1001];
    for(float i = -50; i < 49.95; i+=0.1)
    {
        for(float j = -50; j < 50; j+=0.1)
        {
            //std::cout << "calculating " << i << "," << j << std::endl;
            pt.x = i;
            pt.y = j;
            octree.radiusNeighbors<unibn::L2Distance<pcl::PointXYZRGB> >(pt, 2.0f, results, dist);
            po = calPotantial(pt,results,dist);
            poten[int(i*10+500)*1001+int(j*10+500)] = po;
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
    std::cout << "calculate all potential took "
              << search_time << " seconds." << std::endl;
    std::cout << "max potantial " << maxpo << ", min potantial " << minpo << std::endl;
    std::cout << "potential cloud size " << potanCloud->size() << std::endl;
    //std::cout << "max block num " << maxblocknum << std::endl;

    //pcl::visualization::CloudViewer poviewer("Potantial Viewer");
    //pcl::visualization::CloudViewer pclviewer("Cloud Viewer");

    //test find path
    findPath(potanCloud, poten);

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
