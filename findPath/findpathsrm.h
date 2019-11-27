#ifndef FINDPATHSRM_H
#define FINDPATHSRM_H

#include <iostream>
#include <cstdlib>
#include <time.h>

#include "../Octree.hpp"

#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/core/core.hpp>

using namespace std;

class PathNode
{
public:
    PathNode(float _x, float _y, float _z, int _id);
    ~PathNode();
    void setxyzid(float _x, float _y, float _z, int _id);
    void addLink(int _id_link, float _dist);
    void print(){
        cout<<x<<" "<<y<<" "<<z<<endl;
    }
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

class FindPathSRM
{
public:
    FindPathSRM(float _startx, float _starty, float _startz,
                float _endx, float _endy, float _endz, bool _isview);
    ~FindPathSRM();

    //find path
    bool findPath();
    //display point cloud
    void display();
private:
    //the start and end position
    pcl::PointXYZRGB startP, endP;
    //nearest node from start/end position
    int id_startNode, id_endNode;
    //id of node along path, link is (end position)<-(path.begin)<-(path.end)<-(start position)
    vector<int> path;
    vector<PathNode> pNode;
    //from keyframehandle for view, keyPosPC<->denkeyfPC, keyPosLink<->denkeyfLine
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keyPosPC;
    vector< vector<int> > keyPosLink;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mapPC;
    unibn::Octree<pcl::PointXYZRGB> mapOct;
    float mappointSparse;
    //for pcl viewer
    int v1;
    bool isview;
    //for time
    int64_t time_begin, time_end;

    //read file and reconstruct node graph
    void reconstructGraph();
    //read key pos
    void readPoint();
    //read link between key pos
    void readLink();
    //read map point for view
    void readMapPoint();
    //read parameters from file
    void readParams();
    //init pcl viewer
    void initPclViewer();
    //for i=12345, change str(line00000) to str(line12345)
    void setstring(string &str, int k);

    //find start and end node
    template <typename Distance>
    bool findStartEndNode();
    //a star
    template <typename Distance>
    bool astar();
    //insert and sort path node in queue by distTotal, large at end, small at begin
    void insertSortByDistTotal(int _id, vector<int> &_nodeQueue);

    //for test
    //show node
    void testShowNode();
    //show queue
    void testShowQueue(vector<int> &vec);
};


#endif
