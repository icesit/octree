/*
 * This is used for transforming keyframe into graph that can be
 * used for path finding
 *
 * by XueWuyang
 */

#ifndef KEYFRAMEHANDLE_H
#define KEYFRAMEHANDLE_H

#include <iostream>
#include <cstdlib>
#include <time.h>

#include "../Octree.hpp"
#include "utils.h"

#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/core/core.hpp>

//0 ORB-SLAM
//1 CARTOGRAPHER
//#define DEALMODE 1

using namespace std;

class PathNode
{
public:
    PathNode(float _x, float _y, float _z, int _id);
    ~PathNode();
    void setxyzid(float _x, float _y, float _z, int _id);
    void addLink(int _id_link, float _dist);
    void removeLink(int _id_link);
    //for A star or Dijkstra
    float distTilNow, distToEnd, distTotal;
    int id_fromWhere;
    //link between this node and the other node
    vector<int> id_link;
    //store distance between node
    vector<float> dist_link;
    //flag represent wether the node is dealed
    bool isDealed;
private:
    float x, y, z;
    //location in the queue
    int id_self;
};

class KeyFrameHandler
{
public:
  KeyFrameHandler(const string &mappointfile, const string &keyframefile);
  ~KeyFrameHandler();
  //deal with keyframe
  void dealKeyFrame();
  //display map points and keyframes and result
  void display();
  //save keyframe position and link relation
  void saveResult();
private:
  int DEALMODE;
  //store map points pos
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr mapPC;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr mapPCorigin;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr mapPCfinal;
  //octree of map point
  unibn::Octree<pcl::PointXYZRGB> mapOct;
  unibn::Octree<pcl::PointXYZRGB> mapOctfinal;
  float mappointSparse;
  //store keyframe pos
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr keyfPC;
  //octree of keyframe
  unibn::Octree<pcl::PointXYZRGB> keyfOct;
  //store dense keyframe pos
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr denkeyfPC;
  vector< vector<int> > denkeyfLine;
  float topPercent;
  float minKeyFdist;
  float maxlinkdist;
  //for pcl viewer
  int v1, v2;
  bool isview;
  //remove area
  vector< vector<float> > removeMappointArea;

  //read map points to mapPC
  void readMapPt(const string &mpfile);
  //read keyframe to keyfPC
  void readKeyFrame(const string &kffile);
  //read parameters from file
  void readParams();
  //read the region where you want to remove mappoint
  void readRemoveMapP();
  //init pcl viewer
  void initPclViewer();
  //for i=12345, change str(line00000) to str(line12345)
  void setstring(string &str, int k);
  //find dense keyframe pos
  void findDenseKeyFrame();
  //line all the dense keyframe pos
  void lineDenseKeyFrame();
  //remove redundant lines
  //void removeRedundantLine();
  //kill wrong mappoint using keyframe
  void killErrMapP();
  //find all points along keyframe
  void findPointAlongTwoPos(std::vector<uint32_t>& results, pcl::PointXYZRGB p1, pcl::PointXYZRGB p2);
};

#endif
