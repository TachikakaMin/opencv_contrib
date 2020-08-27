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
        std::vector<Point3f>& pointsB,
        std::vector<Point3f>& normalsB,
        float disThreshold = 0.08)
    {
        Point3f aCenter = pointsA[0];
        Point3f bCenter = pointsB[0];
        Point3f& normalA = normalsA[0];
        Point3f& normalB = normalsB[0];
        std::vector<int> indicesConcaveB;
        convexHull(Mat(pointsB[i]), indicesConcaveB);
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
                    pointsA.push_back(pointsB[i]);
                    normalsA.push_back(normalsB[i]);
                }
                pointsB.clear();
                normalsB.clear();
                return true;
            }
        }
        return false;
    }

    bool growingPlanar(
        std::vector< std::vector<Point3f> >& setPointsN,
        std::vector< std::vector<Point3f> >& setNormalsN,
        std::vector<float>& timestepsN,
        std::vector< std::vector<Point3f> >& setPointsQ,
        std::vector< std::vector<Point3f> >& setNormalsQ,
        std::vector<float>& timestepsQ,
        Point3f& curCameraPos,
        float thetaThreshold,
        float timestepThreshold,
        float timestepDisThreshold,
        std::vector< pair<int,int> >& retS
        )
    {
        retS.clear();
        std::vector<bool> finalQ(setPointsQ.size(), 0);
        std::vector< pair<int,int> > S;

        int sizeN = setPointsN.size();
        int sizeQ = setPointsQ.size();

        for (int i=0; i<sizeN; i++)
        {
            std::vector<int> R;
            R.clear();
            bool gotPlane = 0;
            for (int j=0; j<sizeQ; j++)
            {
                if (!finalQ[j] && angleBetween(setNormalsN[i][0], setNormalsQ[j][0]) < thetaThreshold)
                {
                    gotPlane = planarMerge(setPointsQ[j],
                                           setNormalsQ[j],
                                           setPointsN[i],
                                           setNormalsN[i]);
                    if (gotPlane) break;
                        else R.push_back(j);
                }
            }

            if (!gotPlane)
            {
                timestepsN[i] = 0;
                setPointsQ.push_back(setPointsN[i]);
                setNormalsQ.push_back(setNormalsN[i]);
                for (int j=0;j<R.size();j++)
                    S.push_back(make_pair(i, R[j]))
            }
            // TODO remove from M
        }

        for (int i=0;i<sizeQ;i++)
        {
            timestepsQ[i] ++;
            if (timestepsQ[i] > timestepThreshold)
            {
                finalQ[i] = 1;
                for (int j=0;j<setPointsQ[i].size();j++)
                {
                    Point3f p = curCameraPos - setPointsQ[i][j];
                    if (p.dot(p) < timestepDisThreshold)
                    {
                        finalQ[i] = 0;
                        break;
                    }
                }
            }
        }
        for (int i=0; i<S.size(); i++)
            if (finalQ[S[i].first] + finalQ[S[i].second] == 0)
                retS.push_back(S[i]);
        return true;
    }



    bool mergeCloseSegments(
        std::vector< pair< std::vector<Point3f> ,std::vector<Point3f> > >& pointsS,
        std::vector< pair< std::vector<Point3f> ,std::vector<Point3f> > >& normalsS,
        std::vector<int> alphaS;
        std::vector< std::vector<Point3f> >& setPointsQ,
        std::vector< std::vector<Point3f> >& setNormalsQ,
        std::vector<float>& timestepsQ
        )
    {
        for (int i=0;i<pointsS.size();i++)
        {
            bool gotPlane = 0;
            std::vector<Point3f>& pointsS1 = pointsS[i].first;
            std::vector<Point3f>& pointsS2 = pointsS[i].second;
            std::vector<Point3f>& normalsS1 = normalsS[i].first;
            std::vector<Point3f>& normalsS2 = normalsS[i].second;
            if (pointsS1.size() > pointsS2.size())
            {
                swap(pointsS1, pointsS2);
                swap(normalsS1, normalsS2);
            }
            alphaS[i] = pointsS1.size();
            gotPlane = planarMerge(pointsS2,
                                   normalsS2,
                                   pointsS1,
                                   normalsS1);
            if (gotPlane)
            {
                for (int j=0; j<setPointsQ.size(); j++)
                {
                    // TODO
                    if (setPointsQ[j] == pointsS1)
                    {
                        setPointsQ[j].clear();
                        break;
                    }
                }

                for (int j=0; j<pointsS.size(); j++)
                {
                    if (i == j) continue;
                    std::vector<Point3f>& pointsSJ1 = pointsS[i].first;
                    std::vector<Point3f>& pointsSJ2 = pointsS[i].second;
                    std::vector<Point3f>& normalsSJ1 = normalsS[i].first;
                    std::vector<Point3f>& normalsSJ2 = normalsS[i].second;
                    // TODO
                    if (pointsS1 == pointsSJ1)
                    {
                        pointsSJ1 = pointsS2;
                        normalsSJ1 = normalsS2;
                    }
                    // TODO
                    else if (pointsSJ2 == pointsS1)
                    {
                        pointsSJ2 = pointsS2;
                        normalsSJ2 = normalsS2;
                    }
                    // TODO
                    if (pointsSJ1 == pointsSJ2) {delete pointsS[j];delete normalsS[j];}
                }
                // TODO
                delete pointsS[i];delete normalsS[i];
            }
        }
        return true;
    }

}
}