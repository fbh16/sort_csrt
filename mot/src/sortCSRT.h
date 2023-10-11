#include <map>
#include <set>
#include <tuple>
#include <vector>
#include <cfloat>
#include <fstream>

#include <iostream>
#include <ros/ros.h>
#include <algorithm>
#include "KalmanTracker.h"
#include "Hungarian.h"
#include <cv_bridge/cv_bridge.h>

#include "opencv2/opencv.hpp"
#include "opencv2/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <darknet_ros_msgs/BoundingBox.h>
#include "darknet_ros_msgs/BoundingBoxes.h"
#include <image_transport/image_transport.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
using namespace cv;
using namespace std;
#define resultType tuple<vector<pair<int, int>>, set<int>, set<int>> 

class sortCSRT
{
public:
    sortCSRT();
    
    struct trkinfo
    {
        Ptr<cv::TrackerCSRT> trk;
        int trkAge;
        int trkHit;
        Rect2d trkROI;
    };

    void initKF (
        const vector<Rect2d> dets,
        set<int> kf_umDets,
        vector<KalmanTracker> &allKF
    );

    void initCSRT (
        const vector<Rect2d> dets,
        map<int, trkinfo> &allCSRT,
        cv::Mat image,
        set<int> csrt_umDets
    );

    void KFpredict (vector<KalmanTracker> &allKF, vector<Rect2d> &predictedBoxes);
    
    void CSRTgrow(map<int, trkinfo> &allCSRT, set<int> &csrt_umTrks);

    void KFupdate (
        const vector<Rect2d> dets, 
        vector<KalmanTracker> &allKF, 
        const vector<pair<int, int>> kf_mPairs, 
        const int minHits
    );

    void updateCSRT (
        vector<Rect2d> dets,  
        const vector<Rect2d> kf_trks, 
        vector<KalmanTracker> allKF,
        vector<pair<int,int>> csrt_mPairs,
        cv::Mat image,
        map<int, trkinfo> &allCSRT
    );

    void associateDets2Trks
    (
        const vector<Rect2d> dets,
        const vector<Rect2d> trks, 
        set<int> &umDets, 
        vector<pair<int, int>> &mPairs,
        set<int> &umTrks,
        const float iou_threshold = 0.3 
    );

    void deleteTracker (
        const int maxAge,
        const int minHits,
        vector<KalmanTracker> &allKF,
        map<int, trkinfo> &allCSRT
    );

    double GetIOU(Rect2d bb_test, Rect2d bb_gt);
    
    Rect2d adjustBbx(Rect2d bbox, cv_bridge::CvImagePtr cv_ptr);

    void transformCoordinate(darknet_ros_msgs::BoundingBox &bbx, const int std_w, const int std_h, const int ori_w, const int ori_h);

protected:
    float iou_threshold = 0.3;
    int globalMaxID = 0;
    vector<KalmanTracker> allKF;
    map<int, trkinfo> allCSRT;
    set<int> umDets;
    vector<pair<int, int>> mPairs;
};