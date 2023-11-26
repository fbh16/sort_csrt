#include <iostream>
#include <ros/ros.h>
#include <set>
#include <vector>
#include <boost/thread/thread.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "KalmanTracker.h"
#include "Hungarian.h"

using namespace cv;
using namespace std;
using namespace darknet_ros_msgs;
using namespace sensor_msgs;
using namespace message_filters;

class Sort
{
public:

    Sort(ros::NodeHandle& n, bool view_img, int max_age, int min_hits);

    ~Sort() {};

    void callback
    (
        const sensor_msgs::Image::ConstPtr &img, 
        const darknet_ros_msgs::BoundingBoxes::ConstPtr &detMsg
    );

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

    void AssocDets2Trks(const vector<Rect2d> dets, const vector<Rect2d> trks, \
    set<int> &newDets, set<int> &unmatched, vector<pair<int, int>> &matched, const float iouThresh=0.3)
    {
        matched.clear();
        newDets.clear();
        unmatched.clear();
        int trkNum = trks.size();
        int detNum = dets.size();

        // //////////////////////////
        // std::cout << "detNum: " << detNum << std::endl;
        // std::cout << "trkNum: " << trkNum << std::endl;
        // //////////////////////////

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

            // //////////////////////////
            // std::cout << "assignment:" << std::endl;
            // for (int i=0; i<assignment.size(); i++)
            // {
            //     std::cout << assignment[i] << " ";
            // }
            // std::cout << std::endl;
            // //////////////////////////


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
            // //////////////////////
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
            // //////////////////////////

        }
    }

private:
    ros::Publisher pub;
    cv_bridge::CvImagePtr cv_ptr;
    bool viewImg;
    int std_w = 640;  
    int std_h = 480;

    int minHits;
    int maxAge;
    vector<Rect2d> dets;
    vector<Rect2d> trks;
    vector<KalmanTracker> buffer;
    set<int> newDets;
    set<int> inActive;
    vector<pair<int, int>> Active;
    bool firstFrame = true;

    set<int> matchedDets, matchedTrks;
    set<int> allTrks, allDets;
    vector<int> assignment;
    vector<vector<double>> iouMatrix;
    HungarianAlgorithm HungAlgo;
};