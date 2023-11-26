#include <boost/thread/thread.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>

#include "Hungarian.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"

using namespace cv;
using namespace std;
using namespace darknet_ros_msgs;
using namespace sensor_msgs;
using namespace message_filters;

class MultiCSRT
{
public:
    
    MultiCSRT(ros::NodeHandle& n, bool view_img, int max_age, int min_hits, float psr_thresh, float padding, float filter_lr, int template_size);

    ~MultiCSRT() {};

    void callback
    (
        const sensor_msgs::Image::ConstPtr &img_ptr, 
        const darknet_ros_msgs::BoundingBoxes::ConstPtr &det_msg
    );

    struct Track
    {
        int id;
        int age;
        int hitStreak;
        Rect2d roi;
        bool active;
        Ptr<TrackerCSRT> csrDCF;
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

        Track track;
        track.csrDCF = csrt;
        track.active = false;
        track.roi = dets[detIdx];
        track.age = 0;
        track.hitStreak = 0;
        return track;
    };
    
    double GetIOU(Rect2d bb_test, Rect2d bb_gt)
    {
        float in = (bb_test & bb_gt).area();           
        float un = bb_test.area() + bb_gt.area() - in; 
        if (un < DBL_EPSILON)    
        {
            return 0; 
        }                      
        return (double)(in / un);
    };

    void AssocDets2Trks(const vector<Rect2d> dets, const vector<Rect2d> trks, 
        set<int> &newDets, set<int> &unmatched, vector<pair<int, int>> &matched, const float iouThresh=0.3)
    {
        matched.clear();
        newDets.clear();
        unmatched.clear();
        int trkNum = trks.size();
        int detNum = dets.size();

        ////////////////////////////
        // std::cout << "detNum: " << detNum << std::endl;
        // std::cout << "trkNum: " << trkNum << std::endl;
        ////////////////////////////

        if (detNum == 0)
        {
            for (int i=0; i < trkNum; i++)
            {
                unmatched.insert(i);
            }
        }
        else if (trkNum == 0)
        {
            for (int i = 0; i < dets.size(); i++)
            {
                newDets.insert(i);
            }
        } 
        else if (detNum==0 && trkNum==0)
        {
            ;
        }
        else {
            // cout << "else"  << endl;
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
                // assignment[i]表示与第i个track相匹配的检测索引
                HungAlgo.Solve(iouMatrix, assignment, false); 
            }

            ////////////////////////////
            // std::cout << "assignment:" << std::endl;
            // for (int i=0; i<assignment.size(); i++)
            // {
            //     std::cout << assignment[i] << " ";
            // }
            // std::cout << std::endl;
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
                            matched.push_back({i, assignment[i]});
                        } 
                        else {
                            newDets.insert(assignment[i]);
                            unmatched.insert(i);
                        }
                        // matchedDets.insert(assignment[i]);
                        // matched.push_back({i, assignment[i]});
                    } else {
                        unmatched.insert(i);
                    }
                }
            }
            ////////////////////////
            // std::cout << "unmatched: " << " ";
            // for (auto trk : unmatched)
            // {
            //     std::cout << trk << " ";
            // }
            // std::cout << std::endl;

            // std::cout << "matched: " << " ";
            // for (auto pair : matched)
            // {
            //     std::cout << "(" << pair.first << " " << pair.second << ")" << ", ";
            // }
            // std::cout << std::endl;
            // ////////////////////////
            // for (int i = 0; i < dets.size(); i++)
            // {
            //     allDets.insert(i);
            // }
            // std::set_difference(
            //     allDets.begin(), allDets.end(), 
            //     matchedDets.begin(), matchedDets.end(), 
            //     insert_iterator<set<int>>(newDets, newDets.begin())
            // );

            // ////////////////////////////
            // std::cout << "newDets: " << " ";
            // for (auto det : newDets)
            // {
            //     std::cout << det << " ";
            // }
            // std::cout << std::endl; 
            ////////////////////////////

        }
    }
    
private:
    ros::Publisher pub;
    cv_bridge::CvImagePtr cv_ptr;
    int std_w = 640;  
    int std_h = 480;
    vector<Rect2d> dets;
    vector<Rect2d> trks;

    bool viewImg;
    int minHits;
    int maxAge;

    set<int> newDets;
    set<int> inActive;
    vector<pair<int,int>> Active;
    vector<Track> buffer;
    
    float psrThresh;
    float Padding;
    float filterLR;
    int templateSize;
    bool firstFrame = true;
    int globalMaxID = 0;

    set<int> matchedDets, matchedTrks;
    set<int> allTrks, allDets;
    vector<int> assignment;
    vector<vector<double>> iouMatrix;
    HungarianAlgorithm HungAlgo;
};