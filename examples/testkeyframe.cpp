#include "keyframehandle.h"

int main(int argc, char** argv)
{
    if (argc < 3)
    {
      std::cerr << "filename of point cloud missing." << std::endl;
      return -1;
    }
    std::string mappointfilename = argv[1];
    std::string keyframefilename = argv[2];
    KeyFrameHandler kfh(mappointfilename, keyframefilename);

    kfh.dealKeyFrame();
    kfh.display();
    
    return 1;
}
