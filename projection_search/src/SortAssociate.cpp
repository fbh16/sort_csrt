#include <set>
#include <vector>

#include "opencv2/opencv.hpp"
#include "Hungarian.h"

#include "SortAssociate.h"

using namespace cv;
using namespace std;

double SortAssociate::GetIOU(Rect2d bb_test, Rect2d bb_gt)
{
    float in = (bb_test & bb_gt).area();           // 交集
    float un = bb_test.area() + bb_gt.area() - in; // 并集
    if (un < DBL_EPSILON)    
    {
        return 0; // 避免分母为0
    }                      
    return (double)(in / un);
}

void SortAssociate::AssocDets2Trks(const vector<Rect2d> dets, const vector<Rect2d> trks, \ 
    set<int> &newDets, vector<pair<int, int>> &pairs, const float iouThresh)
{
    pairs.clear();
    newDets.clear();
    int trkNum = trks.size();
    int detNum = dets.size();
    ////////////////////////////
    ////////////////////////////
    // std::cout << "detNum: " << detNum << std::endl;
    // std::cout << "trkNum: " << trkNum << std::endl;
    ////////////////////////////
    ////////////////////////////
    if (trkNum == 0)
    {
        for (int i = 0; i < dets.size(); i++)
        {
            newDets.insert(i);
        }
    } 
    else {
        iouMatrix.resize(trkNum, vector<double>(detNum, 0));
        for (int t = 0; t < trkNum; t++)
        {
            for (int d = 0; d < detNum; d++) 
            {
                iouMatrix[t][d] = 1 - GetIOU(trks[t], dets[d]);
            }
        }
        assignment.clear();
        if (std::min(iouMatrix.size(), iouMatrix[0].size()) > 0)
        {
            HungAlgo.Solve(iouMatrix, assignment, false); // assignment[i]表示与第i个track相匹配的检测索引
        }
        ////////////////////////////
        ////////////////////////////
        // std::cout << "assignment:" << std::endl;
        // for (int i=0; i<assignment.size(); i++)
        // {
        //     std::cout << assignment[i] << " ";
        // }
        // std::cout << std::endl;
        ////////////////////////////
        ////////////////////////////

        /** fill up pairs and newDets */
        if (assignment.size() > 0) {
            for (int i = 0; i < trkNum; i++)
            {
                if (assignment[i] != -1)
                {
                    if (1 - iouMatrix[i][assignment[i]] > iouThresh) 
                    {
                        matchedDets.insert(assignment[i]);
                        pairs.push_back({i, assignment[i]});
                    } 
                    else {
                        newDets.insert(assignment[i]);
                        pairs.push_back({i, -1});
                    }
                } else {
                    pairs.push_back({i, -1});
                }
            }
        }
        ////////////////////////////
        ////////////////////////////
        // std::cout << "newDets1: " << std::endl;
        // for (auto det : newDets)
        // {
        //     std::cout << det << " ";
        // }
        // std::cout << std::endl; 

        // std::cout << "pairs1: " << std::endl;
        // for (auto pair : pairs)
        // {
        //     std::cout << pair.first << " " << pair.second << std::endl;
        // }
        // std::cout << std::endl;
        ////////////////////////////
        ////////////////////////////

        for (int d = 0; d < dets.size(); d++)
        {
            allDets.insert(d);
        }
        std::set_difference(
            allDets.begin(), allDets.end(), 
            matchedDets.begin(), matchedDets.end(), 
            insert_iterator<set<int>>(newDets, newDets.begin())
        );
        ////////////////////////////
        ////////////////////////////
        // std::cout << "newDets2: " << std::endl;
        // for (auto det : newDets)
        // {
        //     std::cout << det << " ";
        // }
        // std::cout << std::endl; 

        // std::cout << "pairs2: " << std::endl;
        // for (auto pair : pairs)
        // {
        //     std::cout << pair.first << " " << pair.second << std::endl;
        // }
        // std::cout << std::endl;
        ////////////////////////////
        ////////////////////////////
    }
}





// void SortAssociate::AssocDets2Trks(const vector<Rect2d> dets, const vector<Rect2d> trks, \
//     set<int> &umDets, set<int> &umTrks, vector<pair<int, int>> &mPairs, const double iouThresh=0.3f) 
// {
//     umTrks.clear();
//     umDets.clear();
//     mPairs.clear();
//     int trkNum = trks.size();
//     int detNum = dets.size();
//     if (trkNum == 0)
//     {
//         for (int i = 0; i < dets.size(); i++)
//         {
//             umDets.insert(i);
//         }
//     } else {
//         iouMatrix.resize(trkNum, vector<double>(detNum, 0));
//         for (int t = 0; t < trkNum; t++)
//         {
//             for (int d = 0; d < detNum; d++)
//                 iouMatrix[t][d] = 1 - GetIOU(trks[t], dets[d]);
//         }
//         assignment.clear();
//         if (std::min(iouMatrix.size(), iouMatrix[0].size()) > 0)
//         {
//             HungAlgo.Solve(iouMatrix, assignment, false); // assignment[i] 表示第 i 个检测框的匹配结果
//         }    
//         for (int d = 0; d < dets.size(); d++)
//         {
//             allDets.insert(d);
//         }
//         for (int t = 0; t < trks.size(); t++)
//         {
//             allTrks.insert(t);
//         }
//         if (assignment.size() > 0) {
//             for (int i = 0; i < trks.size(); i++)
//             {
//                 if (assignment[i] != -1)
//                 {
//                     mDets.insert(assignment[i]);
//                     mTrks.insert(i);
//                 }
//             }
//         }
//         std::set_difference (allDets.begin(), allDets.end(), \
//             mDets.begin(), mDets.end(), insert_iterator<set<int>>(umDets, umDets.begin()));

//         std::set_difference (allTrks.begin(), allTrks.end(), \
//             mTrks.begin(), mTrks.end(), insert_iterator<set<int>>(umTrks, umTrks.begin()));

//         for (int i = 0; i < assignment.size(); i++)
//         {
//             if (assignment[i] == -1)
//             {
//                 continue;
//                 // umTrks.insert(i);
//             }
//             if (1 - iouMatrix[i][assignment[i]] < iouThresh)
//             {
//                 umDets.insert(assignment[i]);
//                 umTrks.insert(i);
//             } 
//             else 
//             {
//                 mPairs.push_back(make_pair(i, assignment[i])); // matches(trk, det)
//             }
//         }
//     }
// }