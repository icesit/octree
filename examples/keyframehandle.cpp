#include "keyframehandle.h"

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
const unibn::OctreeParams& keyfOctParams = unibn::OctreeParams(12, false, 0.01f);//16 for lab,32 for sim,12 for lidar
const unibn::OctreeParams& mapOctParams = unibn::OctreeParams(128, false, 0.02f);

KeyFrameHandler::KeyFrameHandler(const string &mappointfile, const string &keyframefile):
    v1(0),v2(0),isview(true),topPercent(0.25),minKeyFdist(5.0),mappointSparse(0.35), maxlinkdist(6.0),
    mapPC(new pcl::PointCloud<pcl::PointXYZRGB>),
    mapPCorigin(new pcl::PointCloud<pcl::PointXYZRGB>),
    mapPCfinal(new pcl::PointCloud<pcl::PointXYZRGB>),
    keyfPC(new pcl::PointCloud<pcl::PointXYZRGB>),
    denkeyfPC(new pcl::PointCloud<pcl::PointXYZRGB>)
{
    readRemoveMapP();
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
    killErrMapP();
    findDenseKeyFrame();
    lineDenseKeyFrame();
}

void KeyFrameHandler::display()
{
    //display origin map point and keyframe
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> pchmapPC(mapPCorigin);
    viewer->addPointCloud<pcl::PointXYZRGB> (mapPCorigin, pchmapPC, "origin map point cloud", v1);
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
    viewer->addPointCloud<pcl::PointXYZRGB> (mapPCfinal, pchmapPC, "dealed map point cloud", v2);
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

void KeyFrameHandler::saveResult()
{
    cout << "saving dense keyframe position and and link relation..." << endl;

    ofstream of("../Result/denKeyfPos.txt");

    for(int i=0; i<denkeyfPC->size(); ++i)
    {
        of << denkeyfPC->points[i].x << " "
           << denkeyfPC->points[i].y << " "
           << denkeyfPC->points[i].z << endl;
    }
    of.close();

    ofstream of1("../Result/denKeyfPosRelation.txt");
    for(int i=0; i<denkeyfLine.size(); ++i)
    {
        of1 << denkeyfLine[i][0] << " "
            << denkeyfLine[i][1] << endl;
    }
    of1.close();

    ofstream of2("../Result/MapPointsPos.txt");
    for(int i=0; i<mapPCfinal->size(); ++i)
    {
        of2 << mapPCfinal->points[i].x << " "
            << mapPCfinal->points[i].y << " "
            << mapPCfinal->points[i].z << endl;
    }
    of2.close();

    cout << "save done!" << endl;
}

void KeyFrameHandler::readMapPt(const string &mpfile)
{
    std::ifstream in(mpfile.c_str());
    std::string line;
    boost::char_separator<char> sep(" ");
    pcl::PointXYZRGB pt;
    float minx=3000, miny=3000,maxx=-3000,maxy=-3000, minz=3000, maxz=-3000;
    bool skipthispoint = false;
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
        if(DEALMODE == 0){
        //orbslam
        pt.x = boost::lexical_cast<float>(tokens[2]);
        pt.y = - boost::lexical_cast<float>(tokens[0]);
        pt.z = - boost::lexical_cast<float>(tokens[1]);
        }
        else if(DEALMODE == 1){
        //cartographer
        pt.x = boost::lexical_cast<float>(tokens[0]);
        pt.y = boost::lexical_cast<float>(tokens[1]);
        pt.z = boost::lexical_cast<float>(tokens[2]);
        }
        pt.r = 255;
        pt.g = 0;
        pt.b = 0;

        mapPCorigin->push_back(pt);

        for(int i=0; i<removeMappointArea.size(); ++i)
        {
            if(pt.x > removeMappointArea[i][0] && pt.x < removeMappointArea[i][1] &&
               pt.y > removeMappointArea[i][2] && pt.y < removeMappointArea[i][3] &&
               pt.z > removeMappointArea[i][4] && pt.z < removeMappointArea[i][5]){
                skipthispoint = true;
                //break;
            }
        }

        if(skipthispoint)
        {
            skipthispoint = false;
            continue;
        }

        mapPC->push_back(pt);
        mapPCfinal->push_back(pt);
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
        if(DEALMODE == 0){
        //orbslam
        pt.x = boost::lexical_cast<float>(tokens[3]);
        pt.y = - boost::lexical_cast<float>(tokens[1]);
        pt.z = - boost::lexical_cast<float>(tokens[2]);
        }
        else if(DEALMODE == 1){
        //cartographer
        pt.x = boost::lexical_cast<float>(tokens[1]);
        pt.y = boost::lexical_cast<float>(tokens[2]);
        pt.z = boost::lexical_cast<float>(tokens[3]);
        }
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

void KeyFrameHandler::readRemoveMapP()
{
    std::ifstream in("../examples/removeMapP.txt");
    if(!in.is_open())
    {
        std::cout << "cannot read remove area file" <<std::endl;
    }
    std::string line;
    boost::char_separator<char> sep(" ");
    vector<float> tmp;
    tmp.resize(6);
    while (!in.eof())
    {
        std::getline(in, line);
        in.peek();

        boost::tokenizer<boost::char_separator<char> > tokenizer(line, sep);
        std::vector<std::string> tokens(tokenizer.begin(), tokenizer.end());

        if (tokens.size() != 6) continue;
//        tmp.push_back( boost::lexical_cast<float>(tokens[0]));cout<<tmp[0]<<endl;
//        tmp.push_back( boost::lexical_cast<float>(tokens[1]));cout<<tmp[1]<<endl;
//        tmp.push_back( boost::lexical_cast<float>(tokens[2]));cout<<tmp[2]<<endl;
//        tmp.push_back( boost::lexical_cast<float>(tokens[3]));cout<<tmp[3]<<endl;
//        tmp.push_back( boost::lexical_cast<float>(tokens[4]));cout<<tmp[4]<<endl;
//        tmp.push_back( boost::lexical_cast<float>(tokens[5]));cout<<tmp[5]<<endl;
        tmp[0] = boost::lexical_cast<float>(tokens[0]);
        tmp[1] = boost::lexical_cast<float>(tokens[1]);
        tmp[2] = boost::lexical_cast<float>(tokens[2]);
        tmp[3] = boost::lexical_cast<float>(tokens[3]);
        tmp[4] = boost::lexical_cast<float>(tokens[4]);
        tmp[5] = boost::lexical_cast<float>(tokens[5]);

        removeMappointArea.push_back(tmp);
    }
    //cout<<"here:"<<removeMappointArea[0].size()<<endl;

    std::cout << "read remove area----->" <<removeMappointArea.size()<<" area."<<std::endl;
    for(int i=0; i<removeMappointArea.size(); ++i)
    {
        std::cout << "x(" << removeMappointArea[i][0] << "," << removeMappointArea[i][1] <<"),"
                  << "y(" << removeMappointArea[i][2] << "," << removeMappointArea[i][3] <<"),"
                  << "z(" << removeMappointArea[i][4] << "," << removeMappointArea[i][5] <<"),"<<std::endl;
    }
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
    fs["maxlinkdist"] >> maxlinkdist;
    cout << "read parameters----->" << endl
         << "  topPercent:" << topPercent << endl
         << "  minKeyFdist:" << minKeyFdist << endl
         << "  mappointSparse:" << mappointSparse <<endl
         << "  maxlinkdist:" << maxlinkdist <<endl;
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
    //if(DEALMODE == 0){
    keyfOct.rankDenseGrid(denkeyfPC->points, topPercent, minKeyFdist);
    //}
    //else if(DEALMODE == 1){
    //    denkeyfPC = keyfPC;
    //}
    cout << "dense keyframe pos number:" << denkeyfPC->size() << endl;
}

void KeyFrameHandler::lineDenseKeyFrame()
{
    vector<int> a(2);
    for(int i=0; i<denkeyfPC->size()-1; ++i)
    {
        for(int j=i+1; j<denkeyfPC->size(); ++j)
        {
            double _dist2 = pow(denkeyfPC->points[i].x-denkeyfPC->points[j].x, 2) + pow(denkeyfPC->points[i].y-denkeyfPC->points[j].y, 2);
            //here is a param
            if( (_dist2 < maxlinkdist) && (!mapOctfinal.isBlock<unibn::L2Distance<pcl::PointXYZRGB> >(denkeyfPC->points[i], denkeyfPC->points[j], mappointSparse)) )
            {
                a[0] = i;
                a[1] = j;
                denkeyfLine.push_back(a);
            }
        }
    }
    cout << "dense keyframe line number:" << denkeyfLine.size() << endl;
}

void KeyFrameHandler::killErrMapP()
{
    std::vector<uint32_t> results, allPointToKill;
    //find all points along keyframe path
    for(int i=1; i < keyfPC->size(); ++i)
    {
        if(fabs(keyfPC->points[i].x - keyfPC->points[i-1].y) > 2 || fabs(keyfPC->points[i].y - keyfPC->points[i-1].y) > 2){
            continue;
        }
        //mapOct.radiusNeighbors<unibn::L2Distance<pcl::PointXYZRGB> >(keyfPC->points[i], 0.25f, results);
        findPointAlongTwoPos(results, keyfPC->points[i], keyfPC->points[i-1]);
        allPointToKill.insert(allPointToKill.end(), results.begin(), results.end());
    }
    //stamp points
    for(int i=0; i<allPointToKill.size(); ++i)
    {
        mapPCfinal->points[allPointToKill[i]].x = 0;
        mapPCfinal->points[allPointToKill[i]].y = 0;
        mapPCfinal->points[allPointToKill[i]].z = 0;
    }
    //remove points
    for(int i=0; i<mapPCfinal->size(); ++i)
    {
        if(fabs(mapPCfinal->points[i].x) + fabs(mapPCfinal->points[i].y) + fabs(mapPCfinal->points[i].z) < 0.01)
        {
            mapPCfinal->points.erase(mapPCfinal->points.begin()+i);
            --i;
        }
    }

    mapOctfinal.initialize(mapPCfinal->points, mapOctParams);
}

void KeyFrameHandler::findPointAlongTwoPos(std::vector<uint32_t>& results, pcl::PointXYZRGB p1, pcl::PointXYZRGB p2){
    if((pow(p1.x-p2.x, 2) + pow(p1.y-p2.y, 2) + pow(p1.z-p2.z, 2)) > 0.25){
        pcl::PointXYZRGB mid;
        mid.x = (p1.x + p2.x) / 2;
        mid.y = (p1.y + p2.y) / 2;
        mid.z = (p1.z + p2.z) / 2;
        findPointAlongTwoPos(results, mid, p1);
        findPointAlongTwoPos(results, mid, p2);
    }
    else{
        std::vector<uint32_t> _res;
        mapOct.radiusNeighbors<unibn::L2Distance<pcl::PointXYZRGB> >(p1, 0.25f, _res);
        results.insert(results.end(), _res.begin(), _res.end());
        _res.clear();
        mapOct.radiusNeighbors<unibn::L2Distance<pcl::PointXYZRGB> >(p2, 0.25f, _res);
        results.insert(results.end(), _res.begin(), _res.end());
    }
}
