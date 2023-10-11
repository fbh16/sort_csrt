#include <fstream>
#include "sortCSRT.h"

class SubPub : public sortCSRT
{
public:
    SubPub(bool view_img, int max_age, int min_hits);
    void callback(const sensor_msgs::Image::ConstPtr &img, const darknet_ros_msgs::BoundingBoxes::ConstPtr &detMsg);

private:
    ros::NodeHandle nh;
    image_transport::Publisher pubb;
    int thickness = 2;
    bool viewImg;
    float iouThresh = 0.3;
    int maxAge;
    int minHits;
    cv_bridge::CvImagePtr cv_ptr;
    vector<KalmanTracker> allKF;
    map<int, trkinfo> allCSRT;
    vector<Rect2d> dets;
    vector<Rect2d> kf_predictions;
    vector<Rect2d> csrt_trks;
    set<int> csrt_umDets;
    set<int> kf_umDets;
    set<int> csrt_umTrks;
    set<int> kf_umTrks;
    vector<pair<int, int>> csrt_mPairs;
    vector<pair<int, int>> kf_mPairs;
};
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> SyncPolicy;

SubPub::SubPub (bool view_img, int max_age, int min_hits) : viewImg(view_img), maxAge(max_age), minHits(min_hits)
{
    message_filters::Subscriber<sensor_msgs::Image> yolo_img(nh, "/yolo/img", 10);
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> yolo_bbx(nh, "/yolo/bbx", 10);
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), yolo_img, yolo_bbx);
    sync.registerCallback(boost::bind(&SubPub::callback, this, _1, _2));
    ros::spin();
}

void SubPub::callback(const sensor_msgs::ImageConstPtr &img, const darknet_ros_msgs::BoundingBoxes::ConstPtr &detMsg)
{
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    int bbs_num = detMsg->bounding_boxes.size();
    dets.clear();
    for (size_t i = 0; i < bbs_num; i++)
    {
        const auto &bb_msg = detMsg->bounding_boxes[i];
        Rect2d det(bb_msg.xmin, bb_msg.ymin, bb_msg.xmax - bb_msg.xmin, bb_msg.ymax - bb_msg.ymin); // l,t,w,h
        dets.push_back(det);
    }
    cout << "detects:" << endl;
    for (int i=0; i<dets.size(); i++)
    {
        cout << dets[i].x << " " << dets[i].y << " " << dets[i].width << " " << dets[i].height << endl;
    }

    // if first frame
    if (allCSRT.size() == 0)
    {
        int trkID = 0;
        int trkAge = 0;
        int trkHit = 0;
        for (int i=0; i<dets.size(); i++)
        {
            Rect2d trkROI = dets[i];
            static Ptr<TrackerCSRT> csrt = TrackerCSRT::create();
            csrt->init(cv_ptr->image, trkROI);
            trkinfo trkInfo = {csrt, trkAge, trkHit, trkROI};
            allCSRT[trkID] = trkInfo;
            trkID += 1;
        }
    }
    else if (allKF.size() == 0)
    {
        for (int i=0; i<dets.size(); i++)
        {
            Rect2d trkROI = dets[i];
            KalmanTracker kf = KalmanTracker(trkROI);
            allKF.push_back(kf);
        }
    }
    else    
        ;
    // KF predict
    kf_predictions.clear();
    sortCSRT::KFpredict(allKF, kf_predictions); // age+1
    sortCSRT::CSRTgrow(allCSRT, csrt_umTrks); // age+1

    cout << "finished predicting and growing." << endl;
    if (!kf_predictions.empty())
    {
        cout << "kf_predictions:" << endl;
        for (int i=0; i<kf_predictions.size();i++)
        {
            cout << kf_predictions[i].x << " " << kf_predictions[i].y << " " << kf_predictions[i].width << " " << kf_predictions[i].height << endl;
        }
    } 
    else 
    {
        cout << "no kf predictions generate." << endl;
    }

    csrt_trks.clear();
    for (auto it = allCSRT.begin(); it != allCSRT.end(); it++)
    {
        csrt_trks.push_back(it->second.trkROI);
    }

    if (!csrt_trks.empty())
    {
        cout << "csrt_trks:" << endl;
        for (int i=0; i<csrt_trks.size(); i++)
        {
            cout << csrt_trks[i].x << " " << csrt_trks[i].y << " " << csrt_trks[i].width << " " << csrt_trks[i].height << endl;
        }
    }
    else 
    {
        cout << "no CSRT trajectories generate." << endl;
    }

    // associate detects to csrt trackers.
    sortCSRT::associateDets2Trks(dets, csrt_trks, csrt_umDets, csrt_mPairs, csrt_umTrks);

    if (!csrt_umDets.empty())
    {
        cout << "csrt unmatched detects:" << endl;
        for (int ele : csrt_umDets)
        {
            cout << ele << " ";
        }
        cout << endl;
    } else {
        cout << "No new detections for CSRT." << endl;
    }

    if (!csrt_mPairs.empty())
    {
        cout << "csrt matched:" << endl;
        for (int i=0; i<csrt_mPairs.size(); i++)
        {
            cout << csrt_mPairs[i].first << " " << csrt_mPairs[i].second << endl;
        }
    } else {
        cout << "No successful detection-CSRT matches." << endl;
    }

    // associate detects to kf predictions.
    sortCSRT::associateDets2Trks(dets, kf_predictions, kf_umDets, kf_mPairs, kf_umTrks);

    if (!kf_umDets.empty())
    {
        cout << "KF unmatched detects:" << endl;
        for (const int &v : kf_umDets)
        {
            cout << v << " ";
        }
        cout << endl;
    } else {
        cout << "No new detections for KF." << endl;
    }
    
    if (!kf_mPairs.empty())
    {
        cout << "KF matched:" << endl;
        for (int i=0; i<kf_mPairs.size(); i++)
        {
            cout << kf_mPairs[i].first << " " << kf_mPairs[i].second << endl;
        }
    } else {
        cout << "No successful detection-KF matches." << endl;
    }

    // if (!kf_mPairs.empty())
    // {
    //     for (auto it=kf_mPairs.begin(); it!=kf_mPairs.end(); it++)
    //     {
    //         int trkIdx = it->first;
    //         allKF[trkIdx].hit_streak += 1;
    //     }
    // } else {
    //     cout << "Error occurs when increase kf hit_streak" << endl;
    // }

    // if (!csrt_mPairs.empty())
    // {
    //     for (auto it=csrt_mPairs.begin(); it!=csrt_mPairs.end(); it++)
    //     {
    //         int trkIdx = it->first;
    //         allCSRT[trkIdx].trkHit += 1;
    //     }
    // } else {
    //     cout << "Error occurs when increase csrt hit_streak" << endl;
    // }

    // Update current trackers.
    sortCSRT::KFupdate(dets, allKF, kf_mPairs, minHits); // age清零
    cout << "Finishing to update KF" << endl;

    sortCSRT::CSRTupdate(dets, kf_predictions, allKF, csrt_mPairs, cv_ptr, allCSRT);
    cout << "Finishing to update CSRT." << endl;

    // Init new tracker for csrt_umDets.
    for (int ud : kf_umDets)
    {
        sortCSRT::initKF(dets, ud, allKF);
    }
    cout << "(after initial) KF size: " << allKF.size() << endl;
    
    
    for (int ud : csrt_umDets)
    {
        sortCSRT::initCSRT(dets, ud, allCSRT, cv_ptr);
    }
    cout << "(after initial) CSRT size:" << allCSRT.size() << endl;

    // Delete matching failed tracker.
    sortCSRT::deleteTracker(maxAge, minHits, allKF, allCSRT);
    
    cout << "(end) all CSRT Trackers:" << endl;
    for (auto it=allCSRT.begin(); it!=allCSRT.end(); it++)
    {
        cout << (*it).first << " ";
    }
    cout << endl;

    cout << "(end) all KF Trackers:" << endl;
    for (auto it=allKF.begin(); it!=allKF.end(); it++)
    {
        cout << (*it).id << " ";
    }
    cout << endl;

    cout << "----------------------" << endl;
    if (viewImg)
    {
        float txtScale = 0.6;
        int txtThickness = 1;
        Scalar boxRGB(0, 255, 0);
        Scalar txtRGB(255,255,0);
        for (auto it = allCSRT.begin(); it != allCSRT.end();)
        {
            if ((it->second.trkAge < 1)) // && (it->second.trkHit > minHits)
            {
                int id = (*it).first;
                int age = (*it).second.trkAge; 
                string content = "ID= " + to_string(id) + "  " + "Age= " + to_string(age);
                Rect2d bbx = (*it).second.trkROI;
                // cout << bbx.x <<"  "<< bbx.y<<"  " << bbx.width<<"  " << bbx.height << endl;
                Point p1(bbx.x, bbx.y); // lt
                Point p2(bbx.x + bbx.width, bbx.y + bbx.height); // rb
                rectangle(cv_ptr->image, p1, p2, boxRGB, thickness);
                putText(cv_ptr->image, content, Point(bbx.x, bbx.y-3), FONT_HERSHEY_SIMPLEX, txtScale, txtRGB, txtThickness, LINE_AA);
            }
        }
        cv::imshow("6", cv_ptr->image);
        cv::waitKey(1);
    } 
}

int main(int argc, char **argv)
{
    bool debug = false;
    std::ofstream logFile;
    std::streambuf* originalCoutStream;
    if (debug)
    {
        // 打开一个输出文件流
        logFile.open("/home/fbh/2023_goal/factor_graph/src/log.txt");
        // 检查文件是否成功打开
        if (!logFile.is_open()) {
            std::cerr << "Error opening log file." << std::endl;
            return 1; // 返回错误代码
        }
        // 将标准输出重定向到文件流
        originalCoutStream = std::cout.rdbuf();
        std::cout.rdbuf(logFile.rdbuf());
    }
    
    ros::init(argc, argv, "MOT");

    bool view_img = false;
    int max_age, min_hits;
    string img_sub, bbx_sub;

    ros::param::get("~view_img", view_img);
    ros::param::get("~max_age", max_age);
    ros::param::get("~min_hits", min_hits);
    ros::param::get("~img_sub", img_sub);
    ros::param::get("~bbx_sub", bbx_sub);   

    SubPub SP(view_img, max_age, min_hits);
    
    if (debug) {
        std::cout.rdbuf(originalCoutStream);
        logFile.close();
    }
    return 0;
}

// typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> SyncPolicyImgBbx;
// typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImgBbx>> SynchronizerImgBbx;

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "MOT");
//     ros::NodeHandle n;

//     sortCSRT sort_csrt(n);

//     shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> yolo_img_;
//     shared_ptr<message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes>> yolo_bbx_;
//     SynchronizerImgBbx sync_img_bbx_;

//     yolo_img_.reset(new message_filters::Subscriber<sensor_msgs::Image>(n, "yolo/img", 1));
//     yolo_bbx_.reset(new message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes>(n, "yolo/bbx", 1));
//     sync_img_bbx_.reset(new message_filters::Synchronizer<SyncPolicyImgBbx>(SyncPolicyImgBbx(10), *yolo_img_, *yolo_bbx_));
//     sync_img_bbx_->registerCallback(boost::bind(&sortCSRT::callback, &sort_csrt, _1, _2));

//     ros::spin();
//     return 0;
// }