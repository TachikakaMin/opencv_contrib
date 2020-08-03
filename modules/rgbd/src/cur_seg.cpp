//
// Created by YIMIN TANG on 8/2/20.
//

#include "precomp.hpp"
#include "cur_seg.hpp"

namespace cv
{
namespace pcseg
{
    template <typename T, typename A>
    int arg_max(std::vector<T, A> const& vec) {
        return static_cast<int>(std::distance(vec.begin(), max_element(vec.begin(), vec.end())));
    }

    template <typename T, typename A>
    int arg_min(std::vector<T, A> const& vec) {
        return static_cast<int>(std::distance(vec.begin(), min_element(vec.begin(), vec.end())));
    }

    std::vector<int> kNearestNeighbour(k, points, seedPointId)
    {
        int n = points.rows;
        for (int i=0;i<n;i++)
    }

    std::vector<int> planarSegments(Mat& points, Mat& normal, std::vector<double>& curvatures, double& thetaThreshold, double& curvatureThreshold)
    {
        int isSegCount = 0;
        std::queue<int> q;
        std::vector<int> isSeg;
        while (isSegCount < points.rows) {
            int seedPointId = -1;
            if (q.empty())
            {
                int max_idx = arg_max(curvatures);
                seedPointId = max_idx;
            }
            else {
                seedPointId = q.front();
                q.pop();
            }
            if (!isSeg[seedPointId]) isSeg[seedPointId] = seedPointId;
            std::vector<int> kNN = kNearestNeighbour(k, points, seedPointId);
            for (int i=0;i<kNN.size();i++)
                if (!isSeg[kNN[i]])
                    if (arccos(points[kNN[i]], points[seedPointId]) < thetaThreshold)
                    {
                        isSeq[kNN[i]] = seedPointId;
                        if (curvatures[kNN[i]] < curvatureThreshold) q.push(kNN[i]);
                    }
        }
        return isSeg;
    }
}
}