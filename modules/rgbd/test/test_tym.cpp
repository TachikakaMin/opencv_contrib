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

protected:
    void
    run(int) {
        try {
            cv::Mat points, colors;

            // 3D coordinates matrix (Nx3; x,y,z floats format),
            // color associated with each vertex (coordinate matrix) OpenCV BGR format (3 channel Nx1 mat),
            // Path to input file
            ts->printf(cvtest::TS::LOG, "\n PATH:  %s\n", fs::current_path());
            std::string filename_tmp =  std::string(TS::ptr()->get_data_path()) + "rgbd/pcseg/living-room.ply";
            ts->printf(cvtest::TS::LOG, "\n TYM:  %s\n", filename_tmp.c_str());
            const char* filename = filename_tmp.c_str();
            points = cv::ppf_match_3d::loadPLYSimple(filename, 1);
            std::cout << points.size() << std::endl; // Remove this line for large datasets

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