//
// Created by YIMIN TANG on 8/2/20.
//

#include "precomp.hpp"
#include "cur_seg.hpp"

namespace cv
{
namespace pcseg
{

    std::vector<float> calCurvatures(Mat& points, float r)
    {

        int len = points.size().height;
        std::vector<float> curvatures;
        for (int i=0; i<len;i++)
        {
            std::vector<Point3d> nearPoints;
            nearPoints.clear();
            for (int j=0;j<len;j++)
            {
                Point3d p(points.at<float>(0,i), points.at<float>(1,i), points.at<float>(2,i));
                Point3d q(points.at<float>(0,j), points.at<float>(1,j), points.at<float>(2,j));
                if (norm(Mat(p), Mat(q)) < r)
                    nearPoints.push_back(q);
            }
            printf("%lu\n",nearPoints.size());
            PCA pca(Mat(nearPoints), noArray(), 0);
            printf("233\n");
            std::cout<<pca.eigenvalues.size()<<std::endl;
            int size = pca.eigenvalues.size().height;
            float a = pca.eigenvalues.at<float>(0);
            float b = pca.eigenvalues.at<float>(size/2);
            float c = pca.eigenvalues.at<float>(size-1);
            curvatures.push_back(c/(a+b+c));
        }
        return curvatures;
    }


    std::vector<int> planarSegments(Mat& points, Mat& normal, std::vector<float>& curvatures, float& thetaThreshold, float& curvatureThreshold)
    {
        int isSegCount = 0;
        std::queue<int> q;
        std::vector<int> isSeg;
        while (isSegCount < points.rows) {
            int seedPointId = -1;
            if (q.empty())
            {
                for (int )
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