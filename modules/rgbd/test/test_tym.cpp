//
//  main.cpp
//  Compatible with all formats (Binary both little endian and big endian and ASCII)
//
//  Created by Cedric Leblond Menard on 16-07-11.
//  Copyright © 2016 Cedric Leblond Menard. All rights reserved.
//

//#include <iostream>
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include "PLYData.hpp"
#include "test_precomp.hpp"

namespace opencv_test { namespace {
//using namespace cv;
class CV_RgbdPLYDataInputTest : public cvtest::BaseTest {
public:
    CV_RgbdPLYDataInputTest() {
    }

    ~CV_RgbdPLYDataInputTest() {
    }

protected:
    void
    run(int) {
        try {
            cv::Mat points, colors;

            // 3D coordinates matrix (Nx3; x,y,z floats format),
            // color associated with each vertex (coordinate matrix) OpenCV BGR format (3 channel Nx1 mat),
            // Path to input file

            pcseg::getPlyFile(points, colors, "pcseg/living-room.ply");

            // Data can be accessed using "points" and "colors" matrices
            std::cout << points << colors; // Remove this line for large datasets

//            // Export data to see
//            // 3D coordinate matrix to output, Color matrix to output, Output file name/path, Output format, can be PLY_ASCII, PLY_BIGEND or PLY_LITEND
//            DataExporter exporter(points, colors, "output_test.ply", PLY_ASCII);
//            exporter.exportToFile();

        } catch (...) {
            ts->set_failed_test_info(cvtest::TS::FAIL_MISMATCH);
        }
        ts->set_failed_test_info(cvtest::TS::OK);
    }
};


TEST(Rgbd_DepthTo3d, compute)
{
    CV_RgbdPLYDataInputTest test;
    test.safe_run();
}

}}