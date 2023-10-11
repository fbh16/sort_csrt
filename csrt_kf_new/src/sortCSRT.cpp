#include "sortCSRT.h"
#include "KalmanTracker.h"

sortCSRT::sortCSRT()
{

}

void sortCSRT::predict(map<int, Track> &allTracks, set<int> &umTrks)
{
    cout << "预测前 allTrks: " << endl;
    for (auto it=allTracks.begin(); it!=allTracks.end(); it++)
    {
        cout << it->first << " ";
    }
    cout << endl;
    cout << "KF prediction: " << endl;
    for (auto it=allTracks.begin(); it!=allTracks.end(); it++)
    {
        Rect2d pBox = it->second.kfTrack.predict();
        // cout << pBox.x << " " << pBox.y << " " << pBox.width << " " << pBox.height << endl;
        if (pBox.x >= 0 && pBox.y >= 0)
        {
            it->second.kfPrediction = pBox;
        }
        else {
            // int umTrkID = it->first;
            // auto umTrkIt = allTracks.find(umTrkID);
            // int umTrkIdx = std::distance(allTracks.begin(), umTrkIt);
            // cout << "被kf删的Track的ID " << umTrkID << endl;
            // cout << "被删的索引是: " << umTrkIdx << endl;
            // umTrks.erase(umTrkIdx);
            // it = allTracks.erase(it);
            ;
            
        }
    }
    cout << "预测后 allTrks: " << endl;
    for (auto it=allTracks.begin(); it!=allTracks.end(); it++)
    {
        cout << it->first << " ";
    }
    cout << endl;
}

void sortCSRT::update(vector<Rect2d> dets, map<int, Track> &allTracks, vector<pair<int, int>> mPairs, cv::Mat img)
{
    if (!allTracks.empty())
    {
        for (auto pair : mPairs)
        {
            int detIdx = pair.second;
            int trkIdx = pair.first;
            Rect2d detBox = dets[detIdx];

            auto it = allTracks.begin();
            std::advance(it, trkIdx);
            int trkID = it->first;
            cout << "--更新TrkIdx: " << trkIdx << "--" << endl;
            cout << "--更新TrackID: " << trkID << "--" << endl;

            Rect2d kfState = allTracks[trkID].kfTrack.get_state();
            cout << "更新前 KFState: " << kfState.x << " " << kfState.y << " " << kfState.width << " " << kfState.height << endl; 
            
            allTracks[trkID].csrtInfo.hitStreak += 1;
            allTracks[trkID].csrtInfo.age = 0;
            Rect2d csrtState = allTracks[trkID].csrtInfo.roi;

            ///////////////////////////////////////////////
            Rect2d weightedBox = weightedRects(detBox, csrtState, 0.5, 0.5);
            allTracks[trkID].kfTrack.update(weightedBox); 
            ///////////////////////////////////////////////

            Rect2d kfState1 = allTracks[trkID].kfTrack.get_state();

            cout << "更新后 KFState: " << kfState1.x << " " << kfState1.y << " " << kfState1.width << " " << kfState1.height << endl; 
        }
        cout << endl;
    } 
    else
        ;
}

void sortCSRT::initialize(vector<Rect2d> dets, map<int, Track> &allTracks, set<int> umDets, cv::Mat img)
{
    cout << "初始化前的 allTracks Size: " << allTracks.size() << endl;
    int age = 0;
    int hitStreak = 0;
    int maxID, trackID;
    int globalMaxID = 0;
    Rect detBox, csrtROI;
    for (auto detIdx : umDets)
    {
        if (allTracks.empty())
        {
            trackID = globalMaxID;
        }
        else 
        {
            map<int, Track>::reverse_iterator maxIt = allTracks.rbegin();
            maxID = maxIt->first;
            globalMaxID = std::max(globalMaxID, maxID);
            trackID = globalMaxID + 1;
        }
        detBox = dets[detIdx];
        csrtROI = detBox;
        cv::Ptr<TrackerCSRT> csrtTrack = TrackerCSRT::create();
        csrtTrack->init(img, csrtROI);
        csrtinfo csrtInfo = {age, hitStreak, csrtROI};
        // Fill Track
        allTracks[trackID].csrtTrack = csrtTrack;
        allTracks[trackID].csrtInfo = csrtInfo;
        allTracks[trackID].kfPrediction = detBox;
        KalmanTracker kfTrack = KalmanTracker(detBox);
        // allTracks[trackID].kfPrediction = kfTrack.predict();
        allTracks[trackID].kfTrack = kfTrack;
    }
    cout << "初始化后的 allTracks size: " << allTracks.size() << endl;
    cout << endl;
}

void sortCSRT::manage(map<int, Track> &allTracks, set<int> umTrks, const int maxAge)
{
    cout << "删之前的 allTracks size: " << allTracks.size() << endl;
    for (auto it=allTracks.begin(); it!=allTracks.end(); it++)
    {
        cout << it->first << " ";
    }
    cout << endl;
    cout << "删之前 umTrks: " << endl;
    for (auto ut : umTrks)
    {
        cout << ut << " ";
    }
    cout << endl;

    if (!umTrks.empty())
    {
        for (auto trkIdx : umTrks)
        {
            auto it = allTracks.begin();
            std::advance(it, trkIdx); 
            int umTrackID = (*it).first;
            // cout << "allTracks size2: " << allTracks.size() << endl;
            allTracks[umTrackID].csrtInfo.age += 1;
            // cout << "allTracks size3: " << allTracks.size() << endl;
            allTracks[umTrackID].csrtInfo.hitStreak = 0;
            // cout << "allTracks size4: " << allTracks.size() << endl;
        }
    }

    for (auto it=allTracks.begin(); it!=allTracks.end(); it++)
    {
        int age = (*it).second.csrtInfo.age;
        if (age >= maxAge)
        {
            if (allTracks.size() > 1)
            {
                cout << "Track"<< it->first << " 被删了." << endl;
                it = allTracks.erase(it);
                // cout << 111111111 << endl;
            }
            else
            {
                allTracks.clear();
                // cout << 22222 << endl;
            }
        }
        // cout << 3333333 << endl;
    }
    cout << "删之后的 allTracks size: " << allTracks.size() << endl;
    cout << "allTrks: " << endl;
    for (auto it=allTracks.begin(); it!=allTracks.end(); it++)
    {
        cout << it->first << " ";
    }
    cout << endl;
    cout << endl;
}

Rect2d sortCSRT::weightedRects(Rect2d det, Rect2d box, float det_weight, float box_weight)
{
    double x = det_weight*det.x + box_weight*box.x;
    double y = det_weight*det.y + box_weight*box.y;
    double w = det_weight*det.width + box_weight*box.width;
    double h = det_weight*det.height + box_weight*box.height;
    Rect2d weightedBox;
    weightedBox.x = x;
    weightedBox.y = y;
    weightedBox.width = w;
    weightedBox.height = h;
    return weightedBox;
}

Rect2d sortCSRT::adjustBbx(cv::Rect2d bbox, cv::Mat img)
{
    if (bbox.x <0) {
        bbox.width = bbox.width + bbox.x;
        bbox.x = 0;
    } 
    else if (bbox.y <0) {
        bbox.height = bbox.height + bbox.y;
        bbox.y = 0;
    } 
    else if ((bbox.x+bbox.width) > img.cols)
    {
        bbox.width = img.cols;
    }
    else if ((bbox.y+bbox.height) > img.rows) 
    {
        bbox.height = img.rows;
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

void sortCSRT::associateDets2Trks(const vector<Rect2d> dets, const vector<Rect2d> trks, set<int> &umDets, vector<pair<int, int>> &mPairs, set<int> &umTrks, const float iouThreshold) 
{
    umTrks.clear();
    umDets.clear();
    mPairs.clear();
    int trkNum = trks.size();
    int detNum = dets.size();
    if (trkNum == 0)
    {
        cout << "11233333333333333333334444444444444444444444" << endl;
        for (int i = 0; i < dets.size(); i++)
        {
            umDets.insert(i);
        }    
        cout << "没Track时umDets为:" << endl;
        for (auto ud : umDets)
        {
            cout << ud << " ";
        }
        cout << endl;
    }
    else
    {
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
}