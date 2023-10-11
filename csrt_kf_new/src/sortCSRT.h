#include <map>
#include <set>
#include <vector>
#include <cfloat>
#include <fstream>
#include "Hungarian.h"
#include "KalmanTracker.h"

#include <iostream>
#include <ros/ros.h>
#include <algorithm>
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


class sortCSRT
{
public:
    sortCSRT();

    struct csrtinfo
    {
        int age;
        int hitStreak;
        Rect2d roi;
    };

    struct Track
    {
        KalmanTracker kfTrack;
        Rect2d kfPrediction;
        cv::Ptr<cv::TrackerCSRT> csrtTrack;
        csrtinfo csrtInfo;
    };

    void predict(map<int, Track> &allTracks, set<int> &umTrks);
    
    void update(vector<Rect2d> dets, map<int, Track> &allTracks, vector<pair<int, int>> mPairs, cv::Mat img);

    void initialize(vector<Rect2d> dets, map<int, Track> &allTracks, set<int> umDets, cv::Mat img);

    void manage(map<int, Track> &allTracks, set<int> umTrks, const int maxAge);

    void associateDets2Trks
    (
        const vector<Rect2d> dets,
        const vector<Rect2d> trks, 
        set<int> &umDets, 
        vector<pair<int, int>> &mPairs,
        set<int> &umTrks,
        const float iou_threshold = 0.3 
    );

    Rect2d weightedRects(Rect2d det, Rect2d box, float det_weight, float box_weight);

    double GetIOU(Rect2d bb_test, Rect2d bb_gt);
    
    Rect2d adjustBbx(Rect2d bbox, cv::Mat img);

    void transformCoordinate(darknet_ros_msgs::BoundingBox &bbx, const int std_w, const int std_h, const int ori_w, const int ori_h);

protected:
    map<int, Track> allTracks;
    set<int> umDets;
    set<int> umTrks;
    vector<pair<int, int>> mPairs;
};