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
    int findFather(std::vector<int>& , int );
    bool planarSegments(
            Mat& ,
            std::vector<float>& ,
            int ,
            float ,
            float ,
            std::vector<std::vector<Point3f> >& ,
            std::vector<std::vector<Point3f> >&
    );
    bool planarMerge(
            std::vector<Point3f>& ,
            std::vector<Point3f>& ,
            float& ,
            std::vector<Point3f>& ,
            std::vector<Point3f>& ,
            float );
    void growingPlanar(Mat& ,
                       std::vector<std::vector<int> >& ,
                       std::vector<Point3f>& ,
                       std::vector<int>& ,
                       std::vector<float>& ,
                       Mat& ,
                       std::vector<std::vector<int> >& ,
                       std::vector<Point3f>& ,
                       std::vector<int>& ,
                       std::vector<float>& ,
                       Point6f& ,
                       float ,
                       float ,
                       float );
}
}


#endif //OPENCV_CUR_SEG_HPP
