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
        /** intrinsic matrix */
        K_intr << 319.9988245765257,  0.000000000000000,  320.500,
                  0.000000000000000,  319.9988245765257,  240.500,
                  0.000000000000000,  0.000000000000000,  1.00000;
    };

    struct Result
    {
        Eigen::Matrix4d Rt_wc;
        cv::Point3d point;
    };

    cv::Point3d pixel2world(const cv::Rect2d detect, ros::Time timeStamp);

    cv::Point2d world2pixel(const cv::Point3d point, ros::Time timeStamp);

    void transformCoordinate(darknet_ros_msgs::BoundingBox &bbx, const int std_w, const int std_h, const int ori_w, const int ori_h);
    
private:
	tf::TransformListener listener;
    // tf::TransformBroadcaster BC;
    Eigen::Matrix3d K_intr;
    cv::Mat K;
    
};