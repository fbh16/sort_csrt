#include <boost/thread/thread.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>

#include "Hungarian.h"
#include "KalmanTracker3D.h"
#include "projection_sim.h"
#include "SortAssociate.h"

using namespace cv;

class SubPub
{
public:
    SubPub()
    {

    }

    // SubPub(ros::NodeHandle& n, bool view_img, int max_age, int min_hits);
    SubPub(ros::NodeHandle& n, bool view_img);
    
    ~SubPub()
    {

    }
    
    void callback
    (
        const sensor_msgs::Image::ConstPtr &img_ptr, 
        const darknet_ros_msgs::BoundingBoxes::ConstPtr &det_msg,
        const nav_msgs::Odometry::ConstPtr &drone_pose
    );

    Projection proj;
    SortAssociate sort;
    
    struct KF
    {
        KalmanTracker3D trk;
        cv::Point3d predPoint3;
        cv::Rect2d predBox2;
    };
    struct CSRT
    {
        Ptr<TrackerCSRT> trk;
        cv::Rect2d roi;
    };
    struct Track
    {
        KF kf;
        CSRT csrt;
    };
    
private:
    ros::Publisher pub;
    ros::Publisher csrt_pub;
    static tf::TransformListener LS;
    static tf::TransformBroadcaster BS; 
    cv_bridge::CvImagePtr cv_ptr;   
    // int maxAge;
    // int minHits;
    bool viewImg;
    int std_w = 640;  
    int std_h = 480;
    vector<Rect2d> dets;
    vector<Rect2d> trks;
    Eigen::Matrix3d K_intr;
    set<int> newDets;
    vector<pair<int, int>> pairs;
    map<int, Track> trackBuffer;
};