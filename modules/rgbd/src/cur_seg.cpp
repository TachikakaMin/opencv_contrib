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
            std::vector<Point3f> nearPoints;
            nearPoints.clear();
            for (int j=0;j<len;j++)
            {
                Point3f p(points.at<float>(0,i), points.at<float>(1,i), points.at<float>(2,i));
                Point3f q(points.at<float>(0,j), points.at<float>(1,j), points.at<float>(2,j));
                if (norm(Mat(p), Mat(q)) < r)
                    nearPoints.push_back(q);
            }

            Mat pointMat = Mat(nearPoints).reshape(1);
            PCA pca(pointMat, Mat(), 0);
            std::cout<<pca.eigenvalues<<std::endl;
            int size = pca.eigenvalues.size().height;
            float a = pca.eigenvalues.at<float>(0);
            float b = pca.eigenvalues.at<float>(size/2);
            float c = pca.eigenvalues.at<float>(size-1);
            curvatures.push_back(c/(a+b+c));
        }
        return curvatures;
    }


    std::vector<int> planarSegments(
            Mat& pointsWithNormal,
            std::vector<float>& curvatures,
            float& thetaThreshold,
            float& curvatureThreshold)
    {

        std::vector<Point3f> points;
        std::vector<Point3f> normal;
        int len = pointsWithNormal.size().height;
        for (int i=0;i<len;i++)
        {
            Point3f p(pointsWithNormal.at<float>(0,i), pointsWithNormal.at<float>(1,i), pointsWithNormal.at<float>(2,i));
            points.push_back(p);
            Point3f q(pointsWithNormal.at<float>(3,i), pointsWithNormal.at<float>(4,i), pointsWithNormal.at<float>(5,i));
            normal.push_back(q);
        }


        flann::KDTreeIndexParams indexParams;
        flann::Index kdtree(Mat(points).reshape(1), indexParams);


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


            vector<float> query;
            query.push_back(pnt.x); //Insert the 2D point we need to find neighbours to the query
            query.push_back(pnt.y); //Insert the 2D point we need to find neighbours to the query
            vector<int> indices;
            vector<float> dists;
            kdtree.radiusSearch(query, indices, dists, range, numOfPoints);

            for (int i=0;i<indices.size();i++)
                if (!isSeg[indices[i]])
                    if (arccos(points[indices[i]], points[seedPointId]) < thetaThreshold)
                    {
                        isSeq[indices[i]] = seedPointId;
                        if (curvatures[indices[i]] < curvatureThreshold) q.push(indices[i]);
                    }
        }
        return isSeg;
    }
}
}