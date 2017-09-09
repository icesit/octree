#include "keyframehandle.h"

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
const unibn::OctreeParams& keyfOctParams = unibn::OctreeParams(32, false, 0.01f);
const unibn::OctreeParams& mapOctParams = unibn::OctreeParams(128, false, 0.02f);

KeyFrameHandler::KeyFrameHandler(const string &mappointfile, const string &keyframefile):
    v1(0),v2(0),isview(true),topPercent(0.25),minKeyFdist(5.0),mappointSparse(0.35),
    mapPC(new pcl::PointCloud<pcl::PointXYZRGB>),
    keyfPC(new pcl::PointCloud<pcl::PointXYZRGB>),
    denkeyfPC(new pcl::PointCloud<pcl::PointXYZRGB>)
{
    readMapPt(mappointfile);
    readKeyFrame(keyframefile);
    readParams();
    keyfOct.initialize(keyfPC->points, keyfOctParams);
    mapOct.initialize(mapPC->points, mapOctParams);
    initPclViewer();
}

KeyFrameHandler::~KeyFrameHandler()
{}

void KeyFrameHandler::dealKeyFrame()
{
    findDenseKeyFrame();
    lineDenseKeyFrame();
}

void KeyFrameHandler::display()
{
    //display origin map point and keyframe
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> pchmapPC(mapPC);
    viewer->addPointCloud<pcl::PointXYZRGB> (mapPC, pchmapPC, "origin map point cloud", v1);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "origin map point cloud", v1);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> pchkeyfPC(keyfPC);
    viewer->addPointCloud<pcl::PointXYZRGB> (keyfPC, pchkeyfPC, "origin keyframe cloud", v1);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "origin keyframe cloud", v1);
    viewer->addCoordinateSystem (1.0, "origin cloud", v1);
    //add line of keyframe
    string li("line00000");
    for(int i=0; i<keyfPC->size()-1; ++i)
    {
        setstring(li, i);
        viewer->addLine(keyfPC->points[i], keyfPC->points[i+1], 0, 255, 0, li, v1);
    }

    //display dealed map point and keyframe pos
    viewer->addPointCloud<pcl::PointXYZRGB> (mapPC, pchmapPC, "dealed map point cloud", v2);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "dealed map point cloud", v2);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> pchdenkeyfPC(denkeyfPC);
    viewer->addPointCloud<pcl::PointXYZRGB> (denkeyfPC, pchdenkeyfPC, "dealed keyframe cloud", v2);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "dealed keyframe cloud", v2);
    viewer->addCoordinateSystem (1.0, "dealed cloud", v2);
    //add line of dense keyframe
    string lii("dline00000");
    for(int i=0; i<denkeyfLine.size(); ++i)
    {
        setstring(lii, i);
        viewer->addLine(denkeyfPC->points[denkeyfLine[i][0]], denkeyfPC->points[denkeyfLine[i][1]], 0, 255, 0, lii, v2);
    }

    while(isview)
    {
        viewer->spinOnce (100);
    }
}

void KeyFrameHandler::readMapPt(const string &mpfile)
{
    std::ifstream in(mpfile.c_str());
    std::string line;
    boost::char_separator<char> sep(" ");
    pcl::PointXYZRGB pt;
    float minx=3000, miny=3000,maxx=-3000,maxy=-3000, minz=3000, maxz=-3000;
    // read point cloud from "freiburg format"
    while (!in.eof())
    {
        //cout << "read mappoint" << endl;
        std::getline(in, line);
        in.peek();
        //cout << "read mappoint1" << endl;
        boost::tokenizer<boost::char_separator<char> > tokenizer(line, sep);
        std::vector<std::string> tokens(tokenizer.begin(), tokenizer.end());
        //cout << "read mappoint2" << tokens.size() << endl;
        if (tokens.size() != 3) continue;
        pt.x = boost::lexical_cast<float>(tokens[2]);
        pt.y = - boost::lexical_cast<float>(tokens[0]);
        pt.z = - boost::lexical_cast<float>(tokens[1]);
        pt.r = 255;
        pt.g = 0;
        pt.b = 0;

        mapPC->push_back(pt);
        if(pt.x < minx){
            minx = pt.x;
        }
        if(pt.x > maxx){
            maxx = pt.x;
        }
        if(pt.y < miny){
            miny = pt.y;
        }
        if(pt.y > maxy){
            maxy = pt.y;
        }
        if(pt.z < minz){
            minz = pt.z;
        }
        if(pt.z > maxz){
            maxz = pt.z;
        }
    }

    std::cout << "read map point pos----->" << std::endl
              << "  x range (" << minx << "," << maxx << ")" << std::endl
              << "  y range (" << miny << "," << maxy << ")" << std::endl
              << "  z range (" << minz << "," << maxz << ")" << std::endl
              << "  total " << mapPC->size() << " map points." << endl;
    in.close();
}

void KeyFrameHandler::readKeyFrame(const string &kffile)
{
    std::ifstream in(kffile.c_str());
    std::string line;
    boost::char_separator<char> sep(" ");
    pcl::PointXYZRGB pt;
    float minx=3000, miny=3000,maxx=-3000,maxy=-3000, minz=3000, maxz=-3000;
    // read point cloud from "freiburg format"
    while (!in.eof())
    {
        std::getline(in, line);
        in.peek();

        boost::tokenizer<boost::char_separator<char> > tokenizer(line, sep);
        std::vector<std::string> tokens(tokenizer.begin(), tokenizer.end());

        if (tokens.size() != 8) continue;
        pt.x = boost::lexical_cast<float>(tokens[3]);
        pt.y = - boost::lexical_cast<float>(tokens[1]);
        pt.z = - boost::lexical_cast<float>(tokens[2]);
        pt.r = 0;
        pt.g = 0;
        pt.b = 255;

        keyfPC->push_back(pt);
        if(pt.x < minx){
            minx = pt.x;
        }
        if(pt.x > maxx){
            maxx = pt.x;
        }
        if(pt.y < miny){
            miny = pt.y;
        }
        if(pt.y > maxy){
            maxy = pt.y;
        }
        if(pt.z < minz){
            minz = pt.z;
        }
        if(pt.z > maxz){
            maxz = pt.z;
        }
    }

    std::cout << "read keyframe pos----->" << std::endl
              << "  x range (" << minx << "," << maxx << ")" << std::endl
              << "  y range (" << miny << "," << maxy << ")" << std::endl
              << "  z range (" << minz << "," << maxz << ")" << std::endl
              << "  total " << keyfPC->size() << " keyframe pos." << endl;
    in.close();
}

void KeyFrameHandler::readParams()
{
    cv::FileStorage fs("../examples/aParamForKeyframeHandle.yaml", cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        cout << "can not find parameter file ../examples/aParamForKeyframeHandle.yaml" << endl
             << "default parameters are used!" << endl;
        return;
    }
    fs["topPercent"] >> topPercent;
    fs["minKeyFdist"] >> minKeyFdist;
    fs["mappointSparse"] >> mappointSparse;
    cout << "read parameters----->" << endl
         << "  topPercent:" << topPercent << endl
         << "  minKeyFdist:" << minKeyFdist << endl
         << "  mappointSparse:" << mappointSparse <<endl;
}

void KeyFrameHandler::initPclViewer()
{
    viewer->initCameraParameters();
    viewer->createViewPort(0.0,0.0,0.5,1.0,v1);
    viewer->setBackgroundColor (255, 255, 255, v1);
    viewer->createViewPort(0.5,0.0,1.0,1.0,v2);
    viewer->setBackgroundColor (255, 255, 255, v2);
}

void KeyFrameHandler::setstring(string &str, int k)
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

void KeyFrameHandler::findDenseKeyFrame()
{
    //vector<float*> rank_dense_Keyf;
    keyfOct.rankDenseGrid(denkeyfPC->points, topPercent, minKeyFdist);
    cout << "dense keyframe pos number:" << denkeyfPC->size() << endl;
}

void KeyFrameHandler::lineDenseKeyFrame()
{
    vector<int> a(2);
    for(int i=0; i<denkeyfPC->size()-1; ++i)
    {
        for(int j=i+1; j<denkeyfPC->size(); ++j)
        {
            if( !mapOct.isBlock<unibn::L2Distance<pcl::PointXYZRGB> >(denkeyfPC->points[i], denkeyfPC->points[j], mappointSparse) )
            {
                a[0] = i;
                a[1] = j;
                denkeyfLine.push_back(a);
            }
        }
    }
    cout << "dense keyframe line number:" << denkeyfLine.size() << endl;
}
