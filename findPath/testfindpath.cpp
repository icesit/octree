#include "findpathsrm.h"

int main(int argc, char** argv)
{
    if (argc < 6)
    {
        std::cout << "please input 6 params(startx starty startz endx endy endz)" << std::endl;
        return 0;
    }

    int64_t ts, te;
    ts = clock();
    FindPathSRM fpsrm((float)atof(argv[1]), (float)atof(argv[2]), (float)atof(argv[3]),
            (float)atof(argv[4]), (float)atof(argv[5]), (float)atof(argv[6]), true);

    fpsrm.findPath();
    te = clock();
    cout << "total time(including read file and construct graph):" << ((double)(te - ts) / CLOCKS_PER_SEC) << endl;
    fpsrm.display();
    return 1;
}
