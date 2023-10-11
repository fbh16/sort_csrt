#include "sortCSRT.h"
#include "KalmanTracker.h"

sortCSRT::sortCSRT()
{

}

void sortCSRT::KFpredict(vector<KalmanTracker> &allKF, vector<Rect2d> &kf_trks)
{
    for (auto it=allKF.begin(); it!=allKF.end();)
    {
        Rect2d pBox = (*it).predict();
        if (pBox.x >= 0 && pBox.y >= 0)
        {
            kf_trks.push_back(pBox);
            it++;
        }
        else {
            it = allKF.erase(it);
        }
    }
    cout << "finished predicting." << endl;
}

void sortCSRT::CSRTgrow(map<int, trkinfo> &allCSRT, set<int> &csrt_umTrks)
{
    if (!csrt_umTrks.empty())
    {
        auto allCSRT_it = allCSRT.begin();
        for (auto ut : csrt_umTrks)
        {
            int umTrkIdx = ut;
            std::advance(allCSRT_it, umTrkIdx);
            allCSRT_it->second.trkAge += 1;
        }
    } else {
        cout << "allCSRT has no trackers to manage" << endl;
    }
}

void sortCSRT::associateDets2Trks(const vector<Rect2d> dets, const vector<Rect2d> trks, set<int> &umDets, vector<pair<int, int>> &mPairs, set<int> &umTrks, const float iouThreshold) 
{
    umTrks.clear();
    umDets.clear();
    mPairs.clear();
    int trkNum = trks.size();
    int detNum = dets.size();

    vector<vector<double>> iouMatrix;
    iouMatrix.resize(trkNum, vector<double>(detNum, 0));
    for (int t = 0; t < trkNum; t++)
    {
        for (int d = 0; d < detNum; d++)
        {
            iouMatrix[t][d] = 1 - GetIOU(trks[t], dets[d]);
        }
    }
    vector<int> assignment;
    assignment.clear();
    HungarianAlgorithm HungAlgo;
    if (min(iouMatrix.size(), iouMatrix[0].size()) > 0)
    {
        HungAlgo.Solve(iouMatrix, assignment, false); // assignment[i] 表示第 i 个检测框的匹配结果
    }
    set<int> allDets, allTrks;    
    for (int d = 0; d < dets.size(); d++)
    {
        allDets.insert(d);
    } 
    set<int> mDets, mTrks;
    for (int t = 0; t < trks.size(); t++)
    {
        allTrks.insert(t);
    }
    if (assignment.size() > 0) {
        for (int i = 0; i < trks.size(); i++)
        {
            if (assignment[i] != -1)
            {
                mDets.insert(assignment[i]);
                mTrks.insert(i);
            }
        }
    }
    set_difference (allDets.begin(), allDets.end(),
                    mDets.begin(), mDets.end(), insert_iterator<set<int>>(umDets, umDets.begin()));

    set_difference (allTrks.begin(), allTrks.end(),
                    mTrks.begin(), mTrks.end(), insert_iterator<set<int>>(umTrks, umTrks.begin()));

    for (int i = 0; i < assignment.size(); i++)
    {
        if (assignment[i] == -1)
        {
            continue;
        } 
        else 
            ;
        if (1 - iouMatrix[i][assignment[i]] < iouThreshold)
        {
            umDets.insert(assignment[i]);
            umTrks.insert(i);
        } 
        else 
        {
            mPairs.push_back(make_pair(i, assignment[i])); // matches(trk, det)
        }
    }
}



void sortCSRT::KFupdate(const vector<Rect2d> dets, vector<KalmanTracker> &allKF, const vector<pair<int, int>> kf_mPairs, const int minHits)
{
    int kfDetIdx, kfTrkIdx;

    if (!allKF.empty())
    {
        for (unsigned int i = 0; i < kf_mPairs.size(); i++)
        {
            kfTrkIdx = kf_mPairs[i].first;
            kfDetIdx = kf_mPairs[i].second;
            // KF update
            allKF[kfTrkIdx].update(dets[kfDetIdx]);
        }
    } 
    else {
        cout << "(allKF): An empty vector<KalmanTracker>. No kf need to update" << endl;
    }
    
}

void sortCSRT::updateCSRT(vector<Rect2d> dets, const vector<Rect2d> kf_trks, vector<KalmanTracker> allKF, vector<pair<int,int>> csrt_mPairs, cv::Mat image, map<int, trkinfo> &allCSRT) 
{
    double weighted_x, weighted_y, weighted_w, weighted_h;
    double weight_kf = 0.3;
    double weight_det = 0.7;
    Rect2d kfBox, detBox, weightedBox;
    int detIdx, trkIdx;
    if (!allCSRT.empty())
    {   
        cout << "111 all csrt:" << endl;
        for (auto it=allCSRT.begin(); it !=allCSRT.end(); it++)
        {
            cout << "id:" << it->first << " "<< "roi:" << it->second.trkROI.x << " "<< it->second.trkROI.y <<" "<<it->second.trkROI.width << " " << it->second.trkROI.height<<endl;
        }
        for (auto it=csrt_mPairs.begin(); it!=csrt_mPairs.end(); it++)
        {
            trkIdx = it->first;
            detIdx = it->second;
            detBox = dets[detIdx];    
            cout << trkIdx << " " << detIdx << endl;
            auto allCSRT_it = allCSRT.begin();
            std::advance(allCSRT_it, trkIdx);
            cout << "allCSRT_it: " << (*allCSRT_it).first << endl;
            allCSRT_it->second.trkROI = detBox;
            allCSRT_it->second.trk->update(image, detBox);
            allCSRT_it->second.trkAge = 0;
        }
        cout << "222 all csrt:" << endl;
        for (auto it=allCSRT.begin(); it !=allCSRT.end(); it++)
        {
            cout << "id:" << it->first << " " << "roi:" << it->second.trkROI.x << " "<< it->second.trkROI.y <<" "<<it->second.trkROI.width << " " << it->second.trkROI.height<<endl;
        }
        cout << endl;
    } else {
        cout << "(allCSRT): An empty map<int, trkinfo>. No csrt need to update." << endl;
    }
}

void sortCSRT::initKF(const vector<Rect2d> dets, set<int> kf_umDets, vector<KalmanTracker> &allKF) 
{
    for (auto detIdx : kf_umDets)
    {
        Rect2d trkROI(dets[detIdx].x, dets[detIdx].y, dets[detIdx].width, dets[detIdx].height);
        KalmanTracker kf = KalmanTracker(trkROI);
        allKF.push_back(kf);
    }
    
}

void sortCSRT::initCSRT(const vector<Rect2d> dets, map<int, trkinfo> &allCSRT, cv::Mat image, set<int> csrt_umDets) 
{
    int trkAge = 0;
    int trkHit = 0;
    int maxID, trkID;
    Rect2d det, trkROI;
    for (auto ud : csrt_umDets)
    {
        det = dets[ud];
        trkROI = det;
        if (allCSRT.empty())
        {
            trkID = globalMaxID;
        }
        else 
        {
            map<int, trkinfo>::reverse_iterator maxIt = allCSRT.rbegin();
            maxID = maxIt->first;
            globalMaxID = std::max(globalMaxID, maxID);
            trkID = globalMaxID +1;
        }   
        Ptr<TrackerCSRT> csrt = TrackerCSRT::create();
        csrt->init(image, trkROI);
        trkinfo trkInfo = {csrt, trkAge, trkHit, trkROI};
        allCSRT[trkID] = trkInfo; 
    }
}

void sortCSRT::deleteTracker(const int maxAge, const int minHits, vector<KalmanTracker> &allKF, map<int, trkinfo> &allCSRT) 
{
    if (!allCSRT.empty())
    {
        for (auto it=allCSRT.begin(); it != allCSRT.end(); it++)
        {
            if (it != allCSRT.end() && (*it).second.trkAge >= maxAge) 
            {
                cout << "csrt" << it->first << " is deleted." << endl;
                it = allCSRT.erase(it);
            }
        }
    } else {
        cout << "no csrt need to delete." << endl;
    }
    
    if (!allKF.empty())
    {
        for (auto it=allKF.begin(); it != allKF.end(); it++) 
        {
            if (it != allKF.end() && (*it).time_since_update >= maxAge)
            {
                it = allKF.erase(it);
            }
        }
    } else {
        cout << "no kf need to delete." << endl;
    }
}

Rect2d sortCSRT::adjustBbx(cv::Rect2d bbox, cv_bridge::CvImagePtr cv_ptr)
{
    if (bbox.x <0) {
        bbox.width = bbox.width + bbox.x;
        bbox.x = 0;
    } 
    else if (bbox.y <0) {
        bbox.height = bbox.height + bbox.y;
        bbox.y = 0;
    } 
    else if ((bbox.x+bbox.width) > cv_ptr->image.cols)
    {
        bbox.width = cv_ptr->image.cols;
    }
    else if ((bbox.y+bbox.height) > cv_ptr->image.rows) 
    {
        bbox.height = cv_ptr->image.rows;
    }

    return bbox;
}

double sortCSRT::GetIOU(Rect2d bb_test, Rect2d bb_gt)
{
    float in = (bb_test & bb_gt).area();           // 交集
    float un = bb_test.area() + bb_gt.area() - in; // 并集
    if (un < DBL_EPSILON)                          // 避免分母为0
        return 0;
    return (double)(in / un);
}

void sortCSRT::transformCoordinate(darknet_ros_msgs::BoundingBox &bbx, const int std_w, const int std_h, const int ori_w, const int ori_h)
{
    double w_ratio = (double)std_w / ori_w;
    double h_ratio = (double)std_h / ori_h;
    // cout << "before:" << bbx.xmin << " " << bbx.ymin << " " << bbx.xmax << " " << bbx.ymax << endl;
    bbx.xmin = w_ratio * bbx.xmin;
    bbx.xmax = w_ratio * bbx.xmax;
    bbx.ymin = h_ratio * bbx.ymin;
    bbx.ymax = h_ratio * bbx.ymax; 
    // cout << "after:" << bbx.xmin << " " << bbx.ymin << " " << bbx.xmax << " " << bbx.ymax << endl;
} 