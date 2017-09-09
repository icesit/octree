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

using namespace std;

class KeyFrameHandler
{
public:
  KeyFrameHandler(const string &mappointfile, const string &keyframefile);
  ~KeyFrameHandler();
  //deal with keyframe
  void dealKeyFrame();
  //display map points and keyframes and result
  void display();
private:
  //store map points pos
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr mapPC;
  //octree of map point
  unibn::Octree<pcl::PointXYZRGB> mapOct;
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
  //for pcl viewer
  int v1, v2;
  bool isview;

  //read map points to mapPC
  void readMapPt(const string &mpfile);
  //read keyframe to keyfPC
  void readKeyFrame(const string &kffile);
  //read parameters from file
  void readParams();
  //init pcl viewer
  void initPclViewer();
  //for i=12345, change str(line00000) to str(line12345)
  void setstring(string &str, int k);
  //find dense keyframe pos
  void findDenseKeyFrame();
  //line all the dense keyframe pos
  void lineDenseKeyFrame();
};

#endif
