//
// Created by YIMIN TANG on 7/20/20.
//

#include "precomp.hpp"
#include "plydata.hpp"
#include <opencv2/rgbd/pcseg.hpp>

namespace cv
{
namespace pcseg
{
    bool getPlyFile(cv::Mat& _data, cv::Mat& _color , std::string _inputPath) {
        DataImporter importer(_data, _color, _inputPath);
        bool _b = importer.isFileValid();
        if (_b) importer.importPCDataFromFile();
        return _b;
    }
}
}