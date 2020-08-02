//
//  main.cpp
//  Compatible with all formats (Binary both little endian and big endian and ASCII)
//
//  Created by Cedric Leblond Menard on 16-07-11.
//  Copyright © 2016 Cedric Leblond Menard. All rights reserved.
//

#include "test_precomp.hpp"
#include "opencv2/surface_matching/ppf_helpers.hpp"

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
//        try {
            cv::Mat points, colors;

            // 3D coordinates matrix (Nx3; x,y,z floats format),
            // color associated with each vertex (coordinate matrix) OpenCV BGR format (3 channel Nx1 mat),
            // Path to input file
            std::string filename_tmp = ts->get_data_path() + "rgbd/pcseg/living-room.ply";
            ts->printf(cvtest::TS::LOG, "\n%s", filename_tmp.c_str());
            const char* filename = filename_tmp.c_str();
            points = cv::ppf_match_3d::loadPLYSimple(filename, 1);
//            pcseg::getPlyFile(points, colors, "pcseg/living-room.ply");
            // Data can be accessed using "points" and "colors" matrices
            std::cout << points.dims; // Remove this line for large datasets

//            // Export data to see
//            // 3D coordinate matrix to output, Color matrix to output, Output file name/path, Output format, can be PLY_ASCII, PLY_BIGEND or PLY_LITEND
//            DataExporter exporter(points, colors, "output_test.ply", PLY_ASCII);
//            exporter.exportToFile();

//        } catch (...) {
//            ts->set_failed_test_info(cvtest::TS::FAIL_MISMATCH);
//        }
        ts->set_failed_test_info(cvtest::TS::OK);
    }
};


TEST(Rgbd_TYM, compute)
{
    CV_RgbdPLYDataInputTest test;
    test.safe_run();
}

}}