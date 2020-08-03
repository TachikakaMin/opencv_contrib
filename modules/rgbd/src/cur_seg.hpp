//
// Created by YIMIN TANG on 8/2/20.
//

#ifndef OPENCV_CUR_SEG_HPP
#define OPENCV_CUR_SEG_HPP

#include <vector>
#include <queue>
#include "opencv2/ml.hpp"

namespace cv
{
namespace pcseg
{
    // INPUT:
    //  3D point cloud made of points pi (1st) with normals ni(2nd) and curvatures ci(3rd)
    //  4th: angle threshold
    //  5th: curvature threshold
    // OUTPUT:
    //  seg index for each point
    std::vector<int> planarSegments(Mat , Mat , Mat , double , double);
}
}


#endif //OPENCV_CUR_SEG_HPP
