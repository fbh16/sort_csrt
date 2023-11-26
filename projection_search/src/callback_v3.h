#include <boost/thread/thread.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "Hungarian.h"
#include "KalmanTracker3D.h"
#include "projection_sim.h"
#include "SortAssociate.h"

using namespace cv;
using namespace darknet_ros_msgs;
using namespace sensor_msgs;
using namespace nav_msgs;
using namespace message_filters;

class SubPub
{
public:
    SubPub() {}
    
    SubPub(ros::NodeHandle& n, bool view_img, int max_age, int min_hits, float psr_thresh, float padding, float filter_lr, int template_size);

    ~SubPub() {}

    void callback
    (
        const nav_msgs::Odometry::ConstPtr &drone_pose,
        const sensor_msgs::Image::ConstPtr &img_ptr, 
        const darknet_ros_msgs::BoundingBoxes::ConstPtr &det_msg
    );

    Projection proj;

    SortAssociate sort;
    
    struct KF
    {
        KalmanTracker3D trk;
        cv::Point3d predPoint3;
        cv::Point2d predPoint2;
        cv::Rect2d predBox2;
    };
    struct CSRT
    {
        Ptr<TrackerCSRT> trk;
        cv::Rect2d roi; 
        bool active;
    };
    struct Track
    {
        int id;
        int age;
        int hitStreak;
        KF kf;
        CSRT csrt;
    };

    Track createTrack(int detIdx, cv::Mat img, ros::Time timeStamp)
    {
        TrackerCSRT::Params param;
        param.template_size = templateSize;
        param.filter_lr = filterLR;
        param.psr_threshold = psrThresh;
        param.padding = Padding;
        Ptr<TrackerCSRT> csrt = TrackerCSRT::create(param);
        csrt->init(img, dets[detIdx]);
        //initial a kf
        cv::Point3d det3 = proj.pixel2world(dets[detIdx], timeStamp);
        KalmanTracker3D kf = KalmanTracker3D(det3);
        Track track;
        track.csrt.trk = csrt;
        track.csrt.active = false;
        track.csrt.roi = dets[detIdx];
        track.kf.trk = kf;
        track.age = 0;
        track.hitStreak = 0;
        return track;
    }
    
private:
    ros::Publisher pub;
    cv_bridge::CvImagePtr cv_ptr;
    bool viewImg;
    int std_w = 640;  
    int std_h = 480;
    vector<Rect2d> dets;
    vector<Rect2d> trks;
    Eigen::Matrix3d K_intr;
    set<int> newDets;
    set<int> inActive;
    vector<pair<int,int>> Active;
    // map<int, Track> Buffer;
    vector<Track> Buffer;
    int minHits;
    int maxAge;
    float psrThresh;
    float Padding;
    float filterLR;
    int templateSize;
    bool firstFrame = true;
    int globalMaxID = 0;
    int ERROR_DETECT = 0;
};