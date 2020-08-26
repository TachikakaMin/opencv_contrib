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
        float len1 = sqrt(v1.x * v1.x + v1.y * v1.y + v1.z * v1.z);
        float len2 = sqrt(v2.x * v2.x + v2.y * v2.y + v2.z * v2.z);
        float dot = v1.dot(v2);
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
        int channel = pointsWithNormal.size().width;
        printf("%d\n",len);
        if (channel < 6)
        {
            Mat pointsWithNormal2;
            int x = ppf_match_3d::computeNormalsPC3d(pointsWithNormal, pointsWithNormal2, k, 0, Vec3f(0,0,0));
            printf("computeNormalsPC3d: %d\n",x);
            pointsWithNormal = pointsWithNormal2;
            // ppf_match_3d::writePLY(pointsWithNormal, "pickup_big_normal.ply");
            channel = pointsWithNormal.size().width;
            printf("channel: %d\n",channel);
        }
        for (int i=0;i<len;i++)
        {
            Point3f p(pointsWithNormal.at<float>(i,0), pointsWithNormal.at<float>(i,1), pointsWithNormal.at<float>(i,2));
            points.push_back(p);
            Point3f q(pointsWithNormal.at<float>(i,3), pointsWithNormal.at<float>(i,4), pointsWithNormal.at<float>(i,5));
            normal.push_back(q);
        }
        printf("Build KDTree\n");fflush(stdout);
        std::vector<float> curvatures;
        flann::KDTreeIndexParams indexParams;
        flann::Index kdtree(Mat(points).reshape(1), indexParams);
        printf("Build KDTree Done\n");fflush(stdout);

        Mat querys = Mat(points).reshape(1);
        Mat indices;
        Mat dists;
        flann::SearchParams params(32);
        printf("kd tree search\n");fflush(stdout);
        kdtree.knnSearch(querys, indices, dists, k, params);
        std::cout<<indices.size()<<std::endl;
        printf("kd tree search end\n");fflush(stdout);
        for (int i=0; i<len;i++)
        {
            // printf("%d\n",i);fflush(stdout);
            std::vector<Point3f> nearPoints;
            // nearPoints.push_back(normal[i]);
            for (int j=0;j<k;j++) nearPoints.push_back(normal[indices.at<int>(i,j)]);
            Mat pointMat = Mat(nearPoints).reshape(1);
            PCA pca(pointMat, Mat(), 0);
            int size = pca.eigenvalues.size().height;
            float a = pca.eigenvalues.at<float>(0);
            float b = pca.eigenvalues.at<float>(size/2);
            float c = pca.eigenvalues.at<float>(size-1);
            // std::cout<<c/(a+b+c)<<std::endl;
            curvatures.push_back(c/(a+b+c));
        }
        return curvatures;
    }

    int findFather(std::vector<int>& v, int x)
    {
        if (v[x] != x) v[x] = findFather(v[x]);
        return x;
    }

    bool planarSegments(
            Mat& pointsWithNormal,
            std::vector<float>& curvatures,
            int k,
            float thetaThreshold,
            float curvatureThreshold,
            std::vector<std::vector<Point3f> >& vecRetPoints,
            std::vector<std::vector<Point3f> >& vecRetNormals
    )
    {

        std::vector<Point3f> points;
        std::vector<Point3f> normals;
        int len = pointsWithNormal.size().height;
        int channel = pointsWithNormal.size().width;
        if (channel != 6) CV_Error(Error::StsBadArg, String("No Normal Channel!"));
        for (int i=0;i<len;i++)
        {
            Point3f p(pointsWithNormal.at<float>(i,0), pointsWithNormal.at<float>(i,1), pointsWithNormal.at<float>(i,2));
            points.push_back(p);
            Point3f q(pointsWithNormal.at<float>(i,3), pointsWithNormal.at<float>(i,4), pointsWithNormal.at<float>(i,5));
            normals.push_back(q);
        }


        flann::KDTreeIndexParams indexParams;
        flann::Index kdtree(Mat(points).reshape(1), indexParams);


        int isSegCount = 0;
        std::queue<int> q;
        std::vector<bool> isSeg(len, 0 );
        std::vector<int> idSeg(len);
        for (int i=0;i<len;i++) idSeg[i] = i; // disjoint set as index counter

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
                if (angleBetween(normals[seedPointId], normals[indices[i]]) < thetaThreshold)
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

        std::map<int, std::vector<Point3f> > retPoints;
        std::map<int, std::vector<Point3f> > retNormals;
        for (int i=0;i<idSeg.size();i++)
        {
            int idx = findFather(i); // disjoint set
            if (retPoints.find(idx) == retPoints.end())
            {
                std::vector<int> tmp;
                retPoints[idx] = tmp;
                std::vector<int> tmp2;
                retNormals[idx] = tmp2;
                // first point is plane center point and normal is plane normal
                retPoints[idx].push_back(points[idx]);
                retNormals[idx].push_back(normals[idx]);
            }
            retPoints[idx].push_back(points[i]);
            retNormals[idx].push_back(normals[i]);
        }

        // first is Multi-Planar Points, second is Multi-Planar Normals
        std::map<int, std::vector<Point3f> >::iterator it;
        for ( it = retPoints.begin(); it != retPoints.end(); it++ )
        {
            int i = it->first;
            vecRetPoints.push_back[retPoints[i]];
            vecRetNormals.push_back[retNormals[i]];
        }
        return true;
    }

    bool planarMerge(
            std::vector<Point3f>& pointsA,
            std::vector<Point3f>& normalsA,
            float& timestepA,
            std::vector<Point3f> pointsB,
            std::vector<Point3f> normalsB,
            float disThreshold)
    {
        Point3f aCenter = pointsA[0];
        Point3f bCenter = pointsB[0];
        Point3f& normalA = normalsA[0];
        Point3f& normalB = normalsB[0];
        std::vector<int> indicesConcaveB;
        indicesConcaveB =
        for (int i=0;i<indicesConcaveB.size();i++)
        {
            Point3f h = pointsB[indicesConcaveB[i]];
            Point3f dis = aCenter - h;
            if (dis.dot(dis) < disThreshold*disThreshold)
            {
                normalA = normalA*sqrt(aCenter.dot(aCenter)) + normalB*sqrt(bCenter.dot(bCenter));
                normalA /= (sqrt(aCenter.dot(aCenter)) + sqrt(bCenter.dot(bCenter)));
                for (int j=1;j<pointsB.size();j++)
                {
                    idA.push_back(idB[j]);
                }
                timestepA = 0;
                idB.clear();
                return true;
            }
        }
        return false;
    }

    void growingPlanar(Mat& newPointsWithNormal,
                       std::vector<std::vector<int> >& idNs,
                       std::vector<Point3f>& normalNs,
                       std::vector<int>& idCenterNs,
                       std::vector<float>& timestepNs,
                       Mat& oldPointsWithNormal,
                       std::vector<std::vector<int> >& idQs,
                       std::vector<Point3f>& normalQs,
                       std::vector<int>& idCenterQs,
                       std::vector<float>& timestepQs,
                       Point3f& curCameraPos,
                       float thetaThreshold,
                       float timestepThreshold,
                       float timestepDisThreshold
    )
    {
        for (int i=0;i<idNs.size();i++)
        {
            std::vector<int> R;
            R.clear();
            bool gotPlane = 0;
            for (int j=0;j<idQs.size();j++)
            {
                if (!finalQ[i] && angleBetween(normalNs[i], normalQs[j]) < thetaThreshold)
                {
                    std::vector<int> indicesConcave;
                    indicesConcave = getConcaveHull(newPointsWithNormal, idNs[i]);
                    gotPlane = planarMerge();
                    if (gotPlane) break;
                    else R.push_back(j);
                }
            }

            if (!gotPlane)
            {
                kdtree N ic;
                timestepNs[i] = 0;
                idQs.push_back(idNs[i]);
                normalQs.push_back(normalNs[i]);
                idCenterQs.push_back(idCenterNs[i]);
                timestepQs.push_back(timestepNs[i]);



            }
            removeNicFromM;
        }

        for (int i=0;i<idQs.size();i++)
        {
            timestepQs[i] ++;
            if (timestepQs[i] > timestepThreshold)
            {
                finalQ[i] = 1;
                std::vector<int> ind = idQs[i];
                for (int j=0;j<ind.size();j++)
                {
                    Point3f p = curCameraPos - points[ind[j]];
                    if (p.dot(p) < timestepDisThreshold)
                    {
                        finalQ[i] = 0;
                        break;
                    }
                }
            }
        }
    }

}
}