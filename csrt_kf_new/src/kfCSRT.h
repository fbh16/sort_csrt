#include "sortCSRT.h"

class SubPub : public sortCSRT
{
public:
    SubPub()
    {

    }

    SubPub(ros::NodeHandle& n, bool view_img, int max_age, int min_hits);
    
    ~SubPub()
    {

    }
    
    void callback
    (
        const sensor_msgs::Image::ConstPtr &img, 
        const darknet_ros_msgs::BoundingBoxes::ConstPtr &detMsg
    );

private:
    ros::NodeHandle nh;
    image_transport::Publisher pub;
    int maxAge;
    int minHits;
    bool viewImg;
    float iouThresh = 0.3;
    vector<Rect2d> dets;
    vector<Rect2d> trks;
    set<int> umDets;
    set<int> umTrks;
    vector<pair<int, int>> mPairs;
    map<int, Track> allTracks;
    cv_bridge::CvImagePtr cv_ptr;
    ofstream outFile;
    bool initialized = true;
    int trkFrame = 0;
};