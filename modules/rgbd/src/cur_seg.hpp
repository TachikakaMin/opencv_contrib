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
            std::vector<Point3f>& ,
            std::vector<Point3f>& ,
            float );
    bool growingPlanar(
            std::vector< std::vector<Point3f> >& ,
            std::vector< std::vector<Point3f> >& ,
            std::vector<float>& ,
            std::vector< std::vector<Point3f> >& ,
            std::vector< std::vector<Point3f> >& ,
            std::vector<float>& ,
            Point3f& ,
            float ,
            float ,
            float ,
            std::vector< pair<int,int> >&
    );
    bool mergeCloseSegments(
            std::vector< pair< std::vector<Point3f> ,std::vector<Point3f> > >& ,
            std::vector< pair< std::vector<Point3f> ,std::vector<Point3f> > >& ,
            std::vector<int> ;
            std::vector< std::vector<Point3f> >& ,
            std::vector< std::vector<Point3f> >& ,
            std::vector<float>&
    );
}
}


#endif //OPENCV_CUR_SEG_HPP
