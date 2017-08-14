#include <iostream>
#include <cstdlib>
#include <time.h>

#include "../Octree.hpp"
#include "utils.h"

#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
//#include <pcl_conversions/pcl_conversions.h>


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

    int64_t begin, end;
    begin = clock();
    unibn::Octree<pcl::PointXYZRGB> octree;
//    unibn::OctreeParams pp(32, false, 0.1f);
//    unibn::OctreeParams& params = pp;
    const unibn::OctreeParams& params = unibn::OctreeParams(32, false, 0.1f);
    octree.initialize(pCloud->points, params);
    end = clock();
    double search_time = ((double)(end - begin) / CLOCKS_PER_SEC);
    std::cout << "initialize time is" << search_time << std::endl;

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

    pcl::visualization::CloudViewer pclviewer("Cloud Viewer");

    while(1)
    {
        pclviewer.showCloud(pCloud);
    }

    return 0;
}
