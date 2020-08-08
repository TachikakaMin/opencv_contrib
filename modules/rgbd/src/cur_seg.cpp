//
// Created by YIMIN TANG on 8/2/20.
//

#include "precomp.hpp"
#include "cur_seg.hpp"

namespace cv
{
namespace pcseg
{

    float angleBetween(const Point3f &v1, const Point3f &v2)
    {
        float len1 = sqrt(v1.x * v1.x + v1.y * v1.y);
        float len2 = sqrt(v2.x * v2.x + v2.y * v2.y);
        float dot = v1.x * v2.x + v1.y * v2.y;
        float a = dot / (len1 * len2);

        if (a >= 1.0)
            return 0.0;
        else if (a <= -1.0)
            return M_PI;
        else
            return acos(a); // 0..PI
    }

    std::vector<float> calCurvatures(Mat& pointsWithNormal, int k)
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

        std::vector<float> curvatures;
        flann::KDTreeIndexParams indexParams;
        flann::Index kdtree(Mat(points).reshape(1), indexParams);
        for (int i=0; i<len;i++)
        {
            std::vector<float> query;
            query.push_back(points[i].x);
            query.push_back(points[i].y);
            query.push_back(points[i].z);
            std::vector<int> indices;
            std::vector<float> dists;
            kdtree.knnSearch(query, indices, dists, k);


            std::vector<Point3f> nearPoints;
            nearPoints.push_back(points[i]);
            for (int j=0;j<indices.size();j++) nearPoints.push_back(points[indices[j]]);

            Mat pointMat = Mat(nearPoints).reshape(1);
            std::cout<<nearPoints<<std::endl;
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
            int k,
            float thetaThreshold,
            float curvatureThreshold)
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
        std::vector<bool> isSeg(len, 0 );
        std::vector<int> idSeg(len);
        for (int i=0;i<len;i++) idSeg[i] = i;

        while (isSegCount < len || !q.empty()) {
            int seedPointId = -1;
            if (q.empty())
            {
                float cur = 1e6;
                for (int i=0;i<len;i++)
                    if (cur > curvatures[i] && isSeg[i] == 0) {seedPointId = i; cur = curvatures[i];}
                isSeg[seedPointId] = 1;
                isSegCount++;
            }
            else {
                seedPointId = q.front();
                q.pop();
            }

            std::vector<float> query;
            query.push_back(points[seedPointId].x);
            query.push_back(points[seedPointId].y);
            query.push_back(points[seedPointId].z);
            std::vector<int> indices;
            std::vector<float> dists;
            kdtree.knnSearch(query, indices, dists, k);
            for (int i=0;i<indices.size();i++)
            {
                if (angleBetween(normal[seedPointId], normal[indices[i]]) < thetaThreshold)
                {
                    idSeg[indices[i]] = idSeg[seedPointId];
                    if (curvatures[indices[i]] < curvatureThreshold && isSeg[indices[i]] == 0)
                    {
                        isSeg[indices[i]] = 1;
                        isSegCount++;
                        q.push(indices[i]);
                    }
                }
            }
        }
        return idSeg;
    }
}
}