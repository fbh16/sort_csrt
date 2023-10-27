#include <set>
#include <vector>

#include "opencv2/opencv.hpp"
#include "Hungarian.h"

using namespace cv;
using namespace std;

class SortAssociate
{
public:
    double GetIOU(Rect2d bb_test, Rect2d bb_gt);

    // void AssocDets2Trks(
    //     const vector<Rect2d> dets,
    //     const vector<Rect2d> trks,
    //     set<int> &umDets,
    //     set<int> &umTrks,
    //     vector<pair<int, int>> &mPairs,
    //     const double iouThresh = 0.3f
    // );

    void AssocDets2Trks(
        const vector<Rect2d> dets,
        const vector<Rect2d> trks,
        set<int> &newDets,
        vector<pair<int, int>> &pairs,
        const float iouThresh = 0.3f
    );

private:
    set<int> matchedDets, matchedTrks;
    set<int> allTrks, allDets;
    vector<int> assignment;
    vector<vector<double>> iouMatrix;
    HungarianAlgorithm HungAlgo;
};