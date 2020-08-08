//
// Created by YIMIN TANG on 8/2/20.
//

#ifndef OPENCV_CUR_SEG_HPP
#define OPENCV_CUR_SEG_HPP

#include <vector>
#include <queue>
#include <queue>
#include <cmath>
#include <vector>
#include <opencv2/surface_matching/ppf_helpers.hpp>
#include <opencv2/core/mat.hpp>


namespace cv
{
namespace pcseg
{
    float angleBetween(const Point3f& , const Point3f&);
    std::vector<float> calCurvatures(Mat& , int);
    std::vector<int> planarSegments(
            Mat&,
            std::vector<float>&,
            int,
            float ,
            float );
}
}


#endif //OPENCV_CUR_SEG_HPP
