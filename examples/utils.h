#ifndef EXAMPLES_UTILS_H_
#define EXAMPLES_UTILS_H_

#include <fstream>
#include <string>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>

template <typename PointT, typename ContainerT>
void readPoints(const std::string& filename, ContainerT& points)
{
  std::ifstream in(filename.c_str());
  std::string line;
  boost::char_separator<char> sep(" ");
  // read point cloud from "freiburg format"
  while (!in.eof())
  {
    std::getline(in, line);
    in.peek();

    boost::tokenizer<boost::char_separator<char> > tokenizer(line, sep);
    std::vector<std::string> tokens(tokenizer.begin(), tokenizer.end());

    if (tokens.size() != 6) continue;
    float x = boost::lexical_cast<float>(tokens[3]);
    float y = boost::lexical_cast<float>(tokens[4]);
    float z = boost::lexical_cast<float>(tokens[5]);

    points.push_back(PointT(x, y, z));
  }

  in.close();
}

template <typename PointT, typename ContainerT>
void readPoints2(const std::string& filename, const std::string& filename2, ContainerT& points)
{
  std::ifstream in(filename.c_str());
  std::ifstream in2(filename2.c_str());
  std::string line, line2;
  boost::char_separator<char> sep(" ");
  PointT pt;
  // read point cloud from "freiburg format"
  while (!in.eof())
  {
    std::getline(in, line);
    in.peek();
    std::getline(in2, line2);
    in2.peek();

    boost::tokenizer<boost::char_separator<char> > tokenizer(line, sep);
    std::vector<std::string> tokens(tokenizer.begin(), tokenizer.end());

    if (tokens.size() != 6) continue;
    pt.x = boost::lexical_cast<float>(tokens[3]);
    pt.y = boost::lexical_cast<float>(tokens[4]);
    pt.z = boost::lexical_cast<float>(tokens[5]);

    boost::tokenizer<boost::char_separator<char> > tokenizer2(line2, sep);
    std::vector<std::string> tokens2(tokenizer2.begin(), tokenizer2.end());
    if(tokens2.size()!=1) continue;
    pt.r = (boost::lexical_cast<float>(tokens2[0]))/20*255;
    pt.g = (20-boost::lexical_cast<float>(tokens2[0]))/20*255;
    pt.b = (boost::lexical_cast<float>(tokens2[0]))/20*255;

    points.push_back(pt);
  }

  in.close();
}
#endif /* EXAMPLES_UTILS_H_ */
