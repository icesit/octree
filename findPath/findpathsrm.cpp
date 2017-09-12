#include "findpathsrm.h"

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
const unibn::OctreeParams& mapOctParams = unibn::OctreeParams(128, false, 0.02f);

PathNode::PathNode(float _x, float _y, float _z, int _id):
    x(_x),y(_y),z(_z),id_self(_id)
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

void PathNode::addLink(int _id_link)
{
    id_link.push_back(_id_link);
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
    if(findStartEndNode<unibn::L2Distance<pcl::PointXYZRGB> >())
    {}
    else
    {
        cout << "Start/End position is not reachable!" << endl;
        return 0;
    }
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
    std::ifstream in("../Result/denKeyfPosRelation.txt");
    std::string line;
    boost::char_separator<char> sep(" ");
    int i,j;
    vector<int> tmp(2);

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

        pNode[i].addLink(j);
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
        if(tmpdist<mindistS && !mapOct.isBlock<unibn::L2Distance<pcl::PointXYZRGB> >(startP, keyPosPC->points[i], mappointSparse))
        {
            id_startNode = i;
            mindistS = tmpdist;
        }

        tmpdist = Distance::compute(keyPosPC->points[i], endP);
        if(tmpdist<mindistE && !mapOct.isBlock<unibn::L2Distance<pcl::PointXYZRGB> >(endP, keyPosPC->points[i], mappointSparse))
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
