#include "findpathsrm.h"

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
const unibn::OctreeParams& mapOctParams = unibn::OctreeParams(128, false, 0.02f);

PathNode::PathNode(float _x, float _y, float _z, int _id):
    x(_x),y(_y),z(_z),id_self(_id), id_fromWhere(-1), isDealed(false),
    distTilNow(-1), distToEnd(-1), distTotal(-1)
{}

PathNode::~PathNode()
{}

void PathNode::setxyzid(float _x, float _y, float _z, int _id)
{
    x = _x;
    y = _y;
    z = _z;
    id_self = _id;
}

void PathNode::addLink(int _id_link, float _dist)
{
    id_link.push_back(_id_link);
    dist_link.push_back(_dist);
}

FindPathSRM::FindPathSRM(float _startx, float _starty, float _startz,
                         float _endx, float _endy, float _endz, bool _isview):
    startP(200,0,200), endP(100, 50, 50), id_startNode(-1), id_endNode(-1), isview(_isview),
    mappointSparse(5.0),
    keyPosPC(new pcl::PointCloud<pcl::PointXYZRGB>),
    mapPC(new pcl::PointCloud<pcl::PointXYZRGB>)
{
    startP.x = _startx;
    startP.y = _starty;
    startP.z = _startz;
    endP.x = _endx;
    endP.y = _endy;
    endP.z = _endz;
    reconstructGraph();
    initPclViewer();
    mapOct.initialize(mapPC->points, mapOctParams);
}

FindPathSRM::~FindPathSRM()
{}

bool FindPathSRM::findPath()
{
    //testShowNode();
    time_begin = clock();
    if(findStartEndNode<unibn::L2Distance<pcl::PointXYZRGB> >())
    {
        astar<unibn::L2Distance<pcl::PointXYZRGB> >();
    }
    else
    {
        cout << "Start/End position is not reachable!" << endl;
        return 0;
    }
    time_end = clock();
    cout << "search path time:" << ((double)(time_end - time_begin) / CLOCKS_PER_SEC) << endl;
    return 1;
}

void FindPathSRM::display()
{
    if(isview)
    {
        //display map point and dense keyframe position
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> pchmapPC(mapPC);
        viewer->addPointCloud<pcl::PointXYZRGB> (mapPC, pchmapPC, "origin map point cloud", v1);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "origin map point cloud", v1);

        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> pchkeyPosPC(keyPosPC);
        viewer->addPointCloud<pcl::PointXYZRGB> (keyPosPC, pchkeyPosPC, "keyframe pos", v1);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "keyframe pos", v1);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr startendPC(new pcl::PointCloud<pcl::PointXYZRGB>);
        startendPC->push_back(startP);
        startendPC->push_back(endP);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> pchsePC(startendPC);
        viewer->addPointCloud<pcl::PointXYZRGB> (startendPC, pchsePC, "startend", v1);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "startend", v1);

        viewer->addCoordinateSystem (1.0, "origin cloud", v1);
        //add line of keyframe pos
        string li("line00000");
        for(int i=0; i<keyPosLink.size(); ++i)
        {
            setstring(li, i);
            viewer->addLine(keyPosPC->points[keyPosLink[i][0]], keyPosPC->points[keyPosLink[i][1]], 0, 255, 0, li, v1);
        }
        if(id_startNode>=0)
        {
            viewer->addLine(startP, keyPosPC->points[id_startNode], 200, 0, 200, "linestart", v1);
        }
        if(id_endNode>=0)
        {
            viewer->addLine(endP, keyPosPC->points[id_endNode], 200, 0, 200, "lineend", v1);
        }

        //add path line
        if(path.size()>1)
        {
            string pli("path00000");
            cout << "Path(node number):";
            for(int j=0; j<path.size()-1; ++j)
            {
                cout << path[j] << "<-";
                setstring(pli, j);
                viewer->addLine(keyPosPC->points[path[j]], keyPosPC->points[path[j+1]], 200, 0, 200, pli, v1);
            }
            cout << path[path.size()-1] << endl;
            for(int j=path.size()-1; j>=0; --j)
            {
                pNode[path[j]].print();
            }
        }

    }

    while(isview)
    {
        viewer->spinOnce (100);
    }
}

void FindPathSRM::reconstructGraph()
{
    cout << "Get start position(" << startP.x << ","
         << startP.y << "," << startP.z << "), end position("
         << endP.x << "," << endP.y << "," << endP.z << ")." << endl;
    readPoint();
    readLink();
    readMapPoint();
    readParams();
}

void FindPathSRM::readPoint()
{
    std::ifstream in("../Result/denKeyfPos.txt");
    std::string line;
    boost::char_separator<char> sep(" ");
    pcl::PointXYZRGB pt;
    //float minx=3000, miny=3000,maxx=-3000,maxy=-3000, minz=3000, maxz=-3000;
    PathNode pn(0,0,0,0);
    int i=0;
    // read point cloud from "freiburg format"
    while (!in.eof())
    {
        std::getline(in, line);
        in.peek();
        boost::tokenizer<boost::char_separator<char> > tokenizer(line, sep);
        std::vector<std::string> tokens(tokenizer.begin(), tokenizer.end());
        if (tokens.size() != 3) continue;
        pt.x = boost::lexical_cast<float>(tokens[0]);
        pt.y = boost::lexical_cast<float>(tokens[1]);
        pt.z = boost::lexical_cast<float>(tokens[2]);
        pt.r = 0;
        pt.g = 0;
        pt.b = 255;

        keyPosPC->push_back(pt);

        pn.setxyzid(pt.x, pt.y, pt.z, i);
        pNode.push_back(pn);
        ++i;
//        if(pt.x < minx){
//            minx = pt.x;
//        }
//        if(pt.x > maxx){
//            maxx = pt.x;
//        }
//        if(pt.y < miny){
//            miny = pt.y;
//        }
//        if(pt.y > maxy){
//            maxy = pt.y;
//        }
//        if(pt.z < minz){
//            minz = pt.z;
//        }
//        if(pt.z > maxz){
//            maxz = pt.z;
//        }
    }

    std::cout << "read key pos----->" << std::endl
//              << "  x range (" << minx << "," << maxx << ")" << std::endl
//              << "  y range (" << miny << "," << maxy << ")" << std::endl
//              << "  z range (" << minz << "," << maxz << ")" << std::endl
              << "  total " << keyPosPC->size() << " key pos." << endl;
    in.close();
}

void FindPathSRM::readLink()
{
    std::ifstream in("../Result/denKeyfPosRelation1.txt");
    std::string line;
    boost::char_separator<char> sep(" ");
    int i,j;
    vector<int> tmp(2);
    float distmp;

    while (!in.eof())
    {
        std::getline(in, line);
        in.peek();
        boost::tokenizer<boost::char_separator<char> > tokenizer(line, sep);
        std::vector<std::string> tokens(tokenizer.begin(), tokenizer.end());
        if (tokens.size() != 2) continue;
        i = boost::lexical_cast<float>(tokens[0]);
        j = boost::lexical_cast<float>(tokens[1]);
        tmp[0] = i;
        tmp[1] = j;
        keyPosLink.push_back(tmp);
        distmp = unibn::L2Distance<pcl::PointXYZRGB>::sqrt(
                    unibn::L2Distance<pcl::PointXYZRGB>::compute(
                        keyPosPC->points[i],keyPosPC->points[j]));
                /*sqrt(pow((keyPosPC->points[i].x-keyPosPC->points[j].x),2)
                    + pow((keyPosPC->points[i].y-keyPosPC->points[j].y),2)
                    + pow((keyPosPC->points[i].z-keyPosPC->points[j].z),2));*/

        pNode[i].addLink(j, distmp);
        pNode[j].addLink(i, distmp);
    }
    cout << "read link------>" << endl
         << "  total " << keyPosLink.size() << " link." << endl;
}

void FindPathSRM::readMapPoint()
{
    std::ifstream in("../Result/MapPointsPos.txt");
    std::string line;
    boost::char_separator<char> sep(" ");
    pcl::PointXYZRGB pt;
    // read point cloud from "freiburg format"
    while (!in.eof())
    {
        std::getline(in, line);
        in.peek();
        boost::tokenizer<boost::char_separator<char> > tokenizer(line, sep);
        std::vector<std::string> tokens(tokenizer.begin(), tokenizer.end());
        if (tokens.size() != 3) continue;
        pt.x = boost::lexical_cast<float>(tokens[0]);
        pt.y = boost::lexical_cast<float>(tokens[1]);
        pt.z = boost::lexical_cast<float>(tokens[2]);
        pt.r = 255;
        pt.g = 0;
        pt.b = 0;

        mapPC->push_back(pt);
    }

    std::cout << "read map points----->" << std::endl
              << "  total " << mapPC->size() << " map points." << endl;
    in.close();
}

void FindPathSRM::readParams()
{
    cv::FileStorage fs("../examples/aParamForKeyframeHandle.yaml", cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        cout << "can not find parameter file ../examples/aParamForKeyframeHandle.yaml" << endl
             << "default parameters are used!" << endl;
        return;
    }

    fs["mappointSparse"] >> mappointSparse;
    cout << "read parameters----->" << endl
         << "  mappointSparse:" << mappointSparse <<endl;
}

void FindPathSRM::initPclViewer()
{
    viewer->initCameraParameters();
    viewer->createViewPort(0.0,0.0,1.0,1.0,v1);
    viewer->setBackgroundColor (255, 255, 255, v1);
}

void FindPathSRM::setstring(string &str, int k)
{
    int t = 0, i = 0, size = str.size();
    char b = '0';
    while(k>0)
    {
        t = k%10;
        k = k/10;
        b = b + t;
        str.insert(str.end()-i-1, b);
        str.erase(size-i,1);
        b = b - t;
        ++i;
    }
}

template <typename Distance>
bool FindPathSRM::findStartEndNode()
{
    float mindistS = 10000, mindistE = 10000, tmpdist;
    for(int i=0; i<keyPosPC->size(); ++i)
    {
        tmpdist = Distance::compute(keyPosPC->points[i], startP);
        if(tmpdist<mindistS && !mapOct.isBlock<Distance >(startP, keyPosPC->points[i], mappointSparse))
        {
            id_startNode = i;
            mindistS = tmpdist;
        }

        tmpdist = Distance::compute(keyPosPC->points[i], endP);
        if(tmpdist<mindistE && !mapOct.isBlock<Distance >(endP, keyPosPC->points[i], mappointSparse))
        {
            id_endNode = i;
            mindistE = tmpdist;
        }
    }

    cout << "start node id:" << id_startNode << ", end node id:" << id_endNode << endl;
    if(id_startNode<0 || id_endNode<0)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

template <typename Distance>
bool FindPathSRM::astar()
{
    vector<int> nodeQueue;
    //first node
    pNode[id_startNode].id_fromWhere = id_startNode;
    pNode[id_startNode].distTilNow = 0;
    pNode[id_startNode].distToEnd = Distance::sqrt(
                Distance::compute(
                    keyPosPC->points[id_startNode], keyPosPC->points[id_endNode]));
    pNode[id_startNode].distTotal = pNode[id_startNode].distTilNow +
            pNode[id_startNode].distToEnd;
    pNode[id_startNode].isDealed = true;
    nodeQueue.push_back(id_startNode);

    int id_now, id_next, dealtime=0;
    float dtn_tmp;
    cout<<"start find path."<<endl;
    bool valid_path = false;
    while(!nodeQueue.empty())
    {
        if((++dealtime)>200)
        {
            cout << "Path not found" << endl;
            break;
        }

        id_now = nodeQueue[nodeQueue.size()-1];
//        cout<<"dealing node: "<<id_now<<", queuesize: "<<nodeQueue.size()
//           <<endl;

        if(id_now == id_endNode){
            // end node is deal and path is found
            valid_path = true;
        }
//        if(id_now == id_endNode)
//        {
//            while(id_now != id_startNode)
//            {
//                path.push_back(id_now);
//                id_now = pNode[id_now].id_fromWhere;
//            }
//            path.push_back(id_startNode);
//            cout << "Find path done!" << endl;
//            break;
//        }
        nodeQueue.pop_back();

        for(int i=0; i<pNode[id_now].id_link.size(); ++i)
        {
            id_next = pNode[id_now].id_link[i];

            if(id_next == pNode[id_now].id_fromWhere) continue;

            if(!pNode[id_next].isDealed)
            {
                pNode[id_next].distTilNow = pNode[id_now].distTotal
                        + pNode[id_now].dist_link[i];
                pNode[id_next].id_fromWhere = id_now;
                pNode[id_next].distToEnd = 0;/*Distance::sqrt(
                        Distance::compute(
                            keyPosPC->points[id_startNode], keyPosPC->points[id_endNode]));*/
                pNode[id_next].distTotal = pNode[id_next].distToEnd
                        + pNode[id_next].distTilNow;
                pNode[id_next].isDealed = true;

                insertSortByDistTotal(id_next, nodeQueue);
            }
            else
            {
                dtn_tmp = pNode[id_now].distTotal + pNode[id_now].dist_link[i];
                if(dtn_tmp < pNode[id_next].distTilNow)
                {
                    pNode[id_next].distTilNow = dtn_tmp;
                    pNode[id_next].id_fromWhere = id_now;
                    pNode[id_next].distTotal = pNode[id_next].distToEnd
                            + pNode[id_next].distTilNow;

                    int j=0;
                    //erase from queue and readd into queue
                    for(j=0; j<nodeQueue.size(); ++j)
                    {
                        if(nodeQueue[j] == id_next)
                        {
                            nodeQueue.erase(nodeQueue.begin()+j);
                            insertSortByDistTotal(id_next, nodeQueue);
                            break;
                        }
                    }
                }
            }
        }
    }
    if(!valid_path){
        cout<<"no valid path"<<endl;
        return valid_path;
    }
    //find all nodes
    id_now = id_endNode;
    while(id_now != id_startNode)
    {
        path.push_back(id_now);
        id_now = pNode[id_now].id_fromWhere;
    }
    path.push_back(id_startNode);
    cout << "Find path done!" << endl;
    return valid_path;
}

void FindPathSRM::insertSortByDistTotal(int _id, vector<int> &_nodeQueue)
{
    if(_nodeQueue.empty())
    {
        _nodeQueue.push_back(_id);
    }
    else if(pNode[_id].distTotal > pNode[_nodeQueue[0]].distTotal)
    {
        _nodeQueue.insert(_nodeQueue.begin(), _id);
    }
    else if(pNode[_id].distTotal < pNode[_nodeQueue[_nodeQueue.size()-1]].distTotal)
    {
        _nodeQueue.push_back(_id);
    }
    else
    {
        int _start=0, _end=_nodeQueue.size()-1, half=0;
        while((_end-_start)>1)
        {
            half = (_end+_start) / 2;
            if(pNode[_id].distTotal > pNode[_nodeQueue[half]].distTotal)
            {
                _end = half;
            }
            else
            {
                _start = half;
            }
        }
        _nodeQueue.insert(_nodeQueue.begin()+_end, _id);
    }
}

void FindPathSRM::testShowNode()
{
    for(int i=0; i<pNode.size(); ++i)
    {
        cout << "~~node number:" << i << endl << " links:";
        for(int j=0; j<pNode[i].id_link.size(); ++j)
        {
            cout << pNode[i].id_link[j] << " ";
        }
        cout << endl;
    }
}

void FindPathSRM::testShowQueue(vector<int> &vec)
{
    cout << "queue:";
    for(int i=0; i<vec.size(); ++i)
    {
        cout << vec[i] << " ";
    }
    cout << endl;
    cout << "distTotal:";
    for(int i=0; i<vec.size(); ++i)
    {
        cout << pNode[vec[i]].distTotal <<" ";
    }
    cout <<endl;
}
