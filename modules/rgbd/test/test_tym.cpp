//
//  main.cpp
//  Compatible with all formats (Binary both little endian and big endian and ASCII)
//
//  Created by Cedric Leblond Menard on 16-07-11.
//  Copyright © 2016 Cedric Leblond Menard. All rights reserved.
//

#include "test_precomp.hpp"
#include "opencv2/surface_matching/ppf_helpers.hpp"
#include <filesystem>
namespace fs = std::filesystem;
namespace opencv_test { namespace {
//using namespace cv;
class CV_RgbdPLYDataInputTest : public cvtest::BaseTest {
public:
    CV_RgbdPLYDataInputTest() {
    }

    ~CV_RgbdPLYDataInputTest() {
    }
    bool test_angleBetween()
    {
        Point3f p1(0,1,0);
        Point3f p2(0,0,1);
        printf("%f, Should be %f\n", cv::pcseg::angleBetween(p1,p2), M_PI/2);
        p1 = Point3f(0,1,0);
        p2 = Point3f(0,1,0);
        printf("%f, Should be %f\n", cv::pcseg::angleBetween(p1,p2), 0.);
        p1 = Point3f(1,1,0);
        p2 = Point3f(-1,1,0);
        printf("%f, Should be %f\n", cv::pcseg::angleBetween(p1,p2), M_PI/2);
        p1 = Point3f(-1,0,0);
        p2 = Point3f(1,1,0);
        printf("%f, Should be %f\n", cv::pcseg::angleBetween(p1,p2), M_PI - M_PI/4);
        return 1;
    }

    bool test_calCurvatures()
    {
        float data[3][6] = {
                {0,0,1.1, 1.1,0,0},
                {0,1.2,0, 0,1.2,0},
                {1.3,0,0, 0,0,1.3}};
        Mat A = Mat(3, 6, CV_32FC1, &data);
        std::vector<Point3f> points;
        std::vector<Point3f> normal;
        std::vector<float> curvatures;
        cv::pcseg::calCurvatures(A,3,points,normal,curvatures);
        for (int i=0;i<curvatures.size();i++)
            printf("%f ",curvatures[i]);
        printf("\n");
        return 0;
    }

    bool test_planarSegments()
    {
        // std::vector<Point3f> points = {
        //     {  0,   0,   1},
        //     {  0,   0,   2},
        //     {  0,   0,   3},

        //     {100,   0,   1},
        //     {100,   0,   2},
        //     {100,   0,   3}};
        // std::vector<Point3f> normals = {
        //     {  0,   0,   1},
        //     {  0,   0,   1},
        //     {  0,   0,   1},

        //     {1,   0,   0},
        //     {1,   0,   0},
        //     {1,   0,   0}};
        float data[6][6] = {
                {  0,   0,   1,   0,   0,   1},
                {  0,   0,   2,   0,   0,   1},
                {  0,   0,   3,   0,   0,   1},
                {100,   0,   1,   0,   0,   1},
                {100,   0,   2,   0,   0,   1},
                {100,   0,   3,   0,   0,   1}};
        Mat A = Mat(3, 6, CV_32FC1, &data);
        int k = 4;
        std::vector<Point3f> points;
        std::vector<Point3f> normal;
        std::vector<float> curvatures;
        cv::pcseg::calCurvatures(A,k,points,normal,curvatures);
        std::vector<std::vector<Point3f> > vecRetPoints;
        std::vector<std::vector<Point3f> > vecRetNormals;
        cv::pcseg::planarSegments(points, normal, curvatures, k, 10.0/360*2*M_PI, 0.1, vecRetPoints, vecRetNormals);
        return 1;
    }

protected:
    void
    run(int) {
        try {
            test_angleBetween();
        } catch (...) {
            ts->set_failed_test_info(cvtest::TS::FAIL_MISMATCH);
        }
        ts->set_failed_test_info(cvtest::TS::OK);
    }
};


TEST(Rgbd_TYM, compute)
{
    CV_RgbdPLYDataInputTest test;
    test.safe_run();
}

}}