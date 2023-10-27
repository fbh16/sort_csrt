#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

class Projection
{
public:
    Projection()
    {
        K_intr << 319.9988245765257,  0.000000000000000,  320.500,
                  0.000000000000000,  319.9988245765257,  240.500,
                  0.000000000000000,  0.000000000000000,  1.00000;
    };

    struct Result
    {
        Eigen::Matrix4d Rt_wc;
        cv::Point3d point;
    };
    
    cv::Point2d pixel2normalized(const cv::Point2d detect, const Eigen::Matrix3d K_intr);

    cv::Point2d normalized2pixel(const cv::Point3d Cam, const Eigen::Matrix3d K);

    Result pixel2world(const cv::Rect2d detect, ros::Time timeStamp);

    cv::Rect2d world2pixel(const Eigen::Matrix4d Rt, const cv::Point3d point, const double width, const double height);

    void transformCoordinate(darknet_ros_msgs::BoundingBox &bbx, const int std_w, const int std_h, const int ori_w, const int ori_h);


    cv::Vec2d Pixel2Cam(const cv::Vec2d& p, const cv::Mat& K);
    
private:
	tf::TransformListener listener;
    // tf::TransformBroadcaster BC;
    Eigen::Matrix3d K_intr;
    cv::Mat K;
    
};