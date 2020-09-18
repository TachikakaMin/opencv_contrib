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
        if (len1 == 0 || len2 == 0){
            std::cout << v1 << "   " << v2 << std::endl;
        }
        float dot = v1.dot(v2);
        float a = dot / (len1 * len2);

        if (a >= 1.0)
            return 0.0;
        else if (a <= -1.0)
            return M_PI;
        else
            return acos(a); // 0..PI
    }

    bool calCurvatures(
            Mat& pointsWithNormal,
            int k,
            std::vector<Point3f>& points,
            std::vector<Point3f>& normal,
            std::vector<float>& curvatures
    )
    {

        int len = pointsWithNormal.size().height;
        int channel = pointsWithNormal.size().width;
        std::cout<<pointsWithNormal.size() << std::endl;
        if (channel < 6)
        {
            Mat pointsWithNormal2;
            printf("computeNormalsPC3d\n");
            int x = ppf_match_3d::computeNormalsPC3d(pointsWithNormal, pointsWithNormal2, k, 0, Vec3f(0,0,0));
            printf("computeNormalsPC3d Done! \n");
            pointsWithNormal = pointsWithNormal2;
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
        flann::KDTreeIndexParams indexParams;
        std::cout<<points.size()<<std::endl;
        std::cout<<Mat(points).reshape(1).size()<<std::endl;
        flann::Index kdtree(Mat(points).reshape(1), indexParams);
        printf("Build KDTree Done\n");fflush(stdout);

        Mat querys = Mat(points).reshape(1);
        Mat indices;
        Mat dists;
        flann::SearchParams params(16);
        printf("kd tree search\n");fflush(stdout);

        kdtree.knnSearch(querys, indices, dists, k, params);

        std::cout<<indices.size()<<std::endl;
        printf("kd tree search end\n");fflush(stdout);
        for (int i=0; i<len;i++)
        {
            std::vector<Point3f> nearPoints;
            for (int j=0;j<k;j++) nearPoints.push_back(normal[indices.at<int>(i,j)]);
            Mat pointMat = Mat(nearPoints).reshape(1);
            PCA pca(pointMat, Mat(), 0);
            int size = pca.eigenvalues.size().height;
            float a = pca.eigenvalues.at<float>(0);
            float b = pca.eigenvalues.at<float>(size/2);
            float c = pca.eigenvalues.at<float>(size-1);
            if (isnan(c/(a+b+c))) curvatures.push_back(1e6);
            else curvatures.push_back(c/(a+b+c));
        }
        return 1;
    }


    int findFather(std::vector<int>& v, int x)
    {
        if (v[x] != x) v[x] = findFather(v, v[x]);
        return v[x];
    }


    bool planarSegments(
            std::vector<Point3f>& points,
            std::vector<Point3f>& normals,
            std::vector<float>& curvatures,
            int k,
            float thetaThreshold,
            float curvatureThreshold,
            std::vector<std::vector<Point3f> >& vecRetPoints,
            std::vector<std::vector<Point3f> >& vecRetNormals
    )
    {
        int len = points.size();
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
                float cur = 1e9;
                for (int i=0;i<len;i++)
                    if (cur > curvatures[i] && isSeg[i] == 0) {seedPointId = i; cur = curvatures[i];}
                if (seedPointId == -1) { for (int i=0;i<len;i++) if (isSeg[i] == 0) printf("%f\n", curvatures[i]); }
            }
            else {
                seedPointId = q.front();
                q.pop();
            }
            isSegCount -= isSeg[seedPointId];
            isSeg[seedPointId] = 1;
            isSegCount++;

            std::vector<float> query;
            query.push_back(points[seedPointId].x);
            query.push_back(points[seedPointId].y);
            query.push_back(points[seedPointId].z);
            std::vector<int> indices;
            std::vector<float> dists;
            kdtree.knnSearch(query, indices, dists, k);


            // auto compPriority = [] (std::pair<float, int> &a, std::pair<float, int> &b) -> bool { return a.first < b.first; };
            // std::priority_queue<
            //     std::pair<float, int>,
            //     std::vector<std::pair<float, int> >,
            //     decltype(compPriority) > knearest(compPriority);
            // while (!knearest.empty()) knearest.pop();
            // std::vector<int> indices;
            // indices.clear();
            // for (int i=0;i<len;i++)
            // {
            //     if (isSeg[i]) continue;
            //     if (i == seedPointId) continue;
            //     Point3f dis = points[seedPointId] - points[i];
            //     float d = dis.dot(dis);
            //     knearest.push(std::make_pair(d, i));
            //     if (knearest.size() > k) knearest.pop();
            // }
            // while (!knearest.empty())
            // {
            //     indices.push_back(knearest.top().second);
            //     knearest.pop();
            // }


            for (int i=0;i<indices.size();i++)
            {
                if (isSeg[indices[i]] == 1) continue;
                if (normals[indices[i]].dot(normals[indices[i]]) == 0) continue;
                if (normals[seedPointId].dot(normals[seedPointId]) == 0) continue;
                float ang = angleBetween(normals[seedPointId], normals[indices[i]]);
                if (ang > M_PI/2) ang = M_PI - ang;
                if (ang < thetaThreshold)
                {
                    idSeg[indices[i]] = idSeg[seedPointId];
                    isSeg[indices[i]] = 1;
                    isSegCount++;
                    if (curvatures[indices[i]] < curvatureThreshold) {q.push(indices[i]);}
                }

            }
        }

        std::map<int, std::vector<Point3f> > retPoints;
        std::map<int, std::vector<Point3f> > retNormals;
        for (int i=0;i<idSeg.size();i++)
        {
            int idx = findFather(idSeg, i); // disjoint set
            if (retPoints.find(idx) == retPoints.end())
            {
                std::vector<Point3f> tmp;
                retPoints[idx] = tmp;
                std::vector<Point3f> tmp2;
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
            vecRetPoints.push_back(retPoints[i]);
            vecRetNormals.push_back(retNormals[i]);
        }
        printf("vecRetPoints.size() : %lu\n", vecRetPoints.size());
        printf("\n");
        for (int i=0;i<idSeg.size();i++) printf("%d,",findFather(idSeg, i));
        printf("\n");

        return true;
    }



    bool from3dTo2dPlane(
            std::vector<Point3f>& points,
            std::vector<Point3f>& normals,
            std::vector<Point2f >& twodPoints)
    {
        Point3f eye = points[0];
        Point3f target = normals[0] + eye;
        Point3f up(0,0,1);

        Point3f forward = target - eye;
        forward /= norm(forward);

        Point3f side = forward.cross(up);
        side /= norm(side);
        up = side.cross(forward);
        up /= norm(up);

        float plusx, plusy;
        plusx = -side.dot(eye);
        plusy = -up.dot(eye);
        for (int i=0;i<points.size();i++)
        {
            Point2f tmp;
            float x = side.dot(points[i]) + plusx;
            float y = up.dot(points[i]) + plusy;
            tmp.x = x;
            tmp.y = y;
            twodPoints.push_back(tmp);
        }

        // std::vector<std::vector<float> > data = {
        //     {side.x, up.x, -forward.x, 0},
        //     {side.y, up.y, -forward.y, 0},
        //     {side.z, up.z, -forward.z, 0},
        //     {-side.dot(eye), -up.dot(eye), forward.dot(eye), 1.}
        // };
        // Mat lookAtMat = Mat(4, 4, CV_32F, &data);
        return 1;
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

        std::vector<Point2f > twodPoints;

        from3dTo2dPlane(pointsB, normalsB, twodPoints);

        std::vector<int> indicesConcaveB;
        convexHull(Mat(twodPoints), indicesConcaveB);

        std::vector<std::array<float, 2> > concavePoints;
        for (int i=0;i<twodPoints.size();i++)
        {
            std::array<float, 2> tmp;
            tmp[0] = twodPoints[i].x;
            tmp[1] = twodPoints[i].y;
            concavePoints.push_back(tmp);
        }
        auto concave = concaveman<float, 16>(concavePoints, indicesConcaveB, 2, 1);

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
            std::vector< std::pair<int,int> >& retS
    )
    {
        retS.clear();
        std::vector<bool> finalQ(setPointsQ.size(), 0);
        std::vector< std::pair<int,int> > S;

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
                    S.push_back(std::make_pair(i, R[j]));
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
            std::vector< std::pair< std::vector<Point3f> ,std::vector<Point3f> > >& pointsS,
            std::vector< std::pair< std::vector<Point3f> ,std::vector<Point3f> > >& normalsS,
            std::vector<int> alphaS,
            std::vector< std::vector<Point3f> >& setPointsQ,
            std::vector< std::vector<Point3f> >& setNormalsQ,
            std::vector<float>& timestepsQ
    )
    {
        std::vector<bool> deletedS(pointsS.size(),0);

        for (int i=0;i<pointsS.size();i++)
        {
            if (deletedS[i]) continue;
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
                    if (setPointsQ[j][0] == pointsS1[0])
                    {
                        setPointsQ[j].clear();
                        break;
                    }
                }

                for (int j=0; j<pointsS.size(); j++)
                {
                    if (i == j) continue;
                    if (deletedS[j]) continue;
                    std::vector<Point3f>& pointsSJ1 = pointsS[j].first;
                    std::vector<Point3f>& pointsSJ2 = pointsS[j].second;
                    std::vector<Point3f>& normalsSJ1 = normalsS[j].first;
                    std::vector<Point3f>& normalsSJ2 = normalsS[j].second;
                    if (pointsS1[0] == pointsSJ1[0])
                    {
                        pointsSJ1 = pointsS2;
                        normalsSJ1 = normalsS2;
                    }
                    else if (pointsSJ2[0] == pointsS1[0])
                    {
                        pointsSJ2 = pointsS2;
                        normalsSJ2 = normalsS2;
                    }
                    if (pointsSJ1[0] == pointsSJ2[0]) deletedS[j] = 1;
                }
                deletedS[i] = 1;
            }
        }
        return true;
    }



}
}