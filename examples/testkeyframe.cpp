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
    int64_t begin, end;
    begin = clock();
    KeyFrameHandler kfh(mappointfilename, keyframefilename);
    end = clock();
    kfh.dealKeyFrame();

    kfh.saveResult();


    double search_time = ((double)(end - begin) / CLOCKS_PER_SEC);
    cout << "total deal time: " << search_time << " seconds." <<endl;
    kfh.display();
    return 1;
}
