#include "projection_sim.h"

// cv::Point2d Projection::normalized2pixel(const cv::Point3d Cam, const Eigen::Matrix3d K)
// {
//     cv::Point2d bbx_centre;
//     double fx = K(0, 0);
//     double fy = K(1, 1);
//     double cx = K(0, 2);
//     double cy = K(1, 2);
//     bbx_centre.x = fx * Cam.x / Cam.z + cx;
//     bbx_centre.y = fy * Cam.y / Cam.z + cy;
//     return bbx_centre;
// }

// cv::Point2d Projection::pixel2normalized(const cv::Point2d bbx_centre, const Eigen::Matrix3d K)
// {
//     cv::Point2d Cam;
//     double fx = K(0, 0);
//     double fy = K(1, 1);
//     double cx = K(0, 2);
//     double cy = K(1, 2);
//     Cam.x = (bbx_centre.x - cx) / fx;
//     Cam.y = (bbx_centre.y - cy) / fy;

//     return Cam;
// }

///////////////1024///////////////////

// Projection::Result Projection::pixel2world(const cv::Rect2d detect, ros::Time timeStamp)
// {
//     /** 计算框底边中心 */
//     double centreX = detect.x + detect.width / 2;
//     double centreY = detect.y + detect.height; 
//     cv::Point2d P_pix(centreX, centreY);
//     /** 像素系转换到图像系 */
//     double fx = K_intr(0,0);
//     double fy = K_intr(1,1);
//     double u0 = K_intr(0,2);
//     double v0 = K_intr(1,2);
//     double X_nlz = (P_pix.x - u0) / fx;
//     double Y_nlz = (P_pix.y - v0) / fy;
//     cv::Point3d P_nlz(X_nlz, Y_nlz, 1);
//     /** 计算相机系转换至世界系的旋转平移矩阵 */
//     tf::StampedTransform TF_wc;
//     listener.waitForTransform("world", "front_cam_optical_frame", timeStamp, ros::Duration(0.06));
//     listener.lookupTransform("world", "front_cam_optical_frame", timeStamp, TF_wc);
//     Eigen::Vector3d t_wc;
//     t_wc << TF_wc.getOrigin().getX(), TF_wc.getOrigin().getY(), TF_wc.getOrigin().getZ();
//     Eigen::Matrix3d R_wc;
//     tf::Matrix3x3 R_wc_ = TF_wc.getBasis();
//     R_wc << R_wc_.getRow(0).getX(), R_wc_.getRow(0).getY(), R_wc_.getRow(0).getZ(),
//             R_wc_.getRow(1).getX(), R_wc_.getRow(1).getY(), R_wc_.getRow(1).getZ(),
//             R_wc_.getRow(2).getX(), R_wc_.getRow(2).getY(), R_wc_.getRow(2).getZ();
//     Eigen::Matrix4d Rt_wc = Eigen::Matrix4d::Identity();
//     Rt_wc.block<3,3>(0,0) = R_wc;
//     Rt_wc.block<3,1>(0,3) = t_wc;
//     /** 计算尺度因子 */
//     double Zc = -t_wc[2] / (R_wc(2,0)*P_nlz.x + R_wc(2,1)*P_nlz.y + R_wc(2,2));
//     /** 像素系转换到图像系 */
//     Eigen::Vector4d P_cam;
//     P_cam[0] = P_nlz.x * Zc;
//     P_cam[1] = P_nlz.y * Zc;
//     P_cam[2] = P_nlz.z * Zc;
//     P_cam[3] = 1;
//     /** 相机系转换至世界系 */
//     Eigen::Vector4d P_world_;
//     P_world_ = Rt_wc * P_cam;
//     /** fill up result */
//     cv::Point3d P_world;
//     P_world.x = P_world_[0];
//     P_world.y = P_world_[1];
//     P_world.z = P_world_[2];
//     Result result;
//     result.point = P_world;
//     result.Rt_wc = Rt_wc;
//     return result;
// }


/////////////////1107////////////////
cv::Point3d Projection::pixel2world(const cv::Rect2d detect, ros::Time timeStamp)
{
    /** 计算框底边中心 */
    double centreX = detect.x + detect.width / 2;
    double centreY = detect.y + detect.height; 
    cv::Point2d P_pix(centreX, centreY);
    /** 像素系转换到图像系 */
    double fx = K_intr(0,0);
    double fy = K_intr(1,1);
    double u0 = K_intr(0,2);
    double v0 = K_intr(1,2);
    double X_nlz = (P_pix.x - u0) / fx;
    double Y_nlz = (P_pix.y - v0) / fy;
    cv::Point3d P_nlz(X_nlz, Y_nlz, 1);
    /** 计算相机系转换至世界系的旋转平移矩阵 */
    tf::StampedTransform TF_wc;
    listener.waitForTransform("world", "front_cam_optical_frame", timeStamp, ros::Duration(1.5));
    listener.lookupTransform("world", "front_cam_optical_frame", timeStamp, TF_wc);
    Eigen::Vector3d t_wc;
    t_wc << TF_wc.getOrigin().getX(), TF_wc.getOrigin().getY(), TF_wc.getOrigin().getZ();
    Eigen::Matrix3d R_wc;
    tf::Matrix3x3 R_wc_ = TF_wc.getBasis();
    R_wc << R_wc_.getRow(0).getX(), R_wc_.getRow(0).getY(), R_wc_.getRow(0).getZ(),
            R_wc_.getRow(1).getX(), R_wc_.getRow(1).getY(), R_wc_.getRow(1).getZ(),
            R_wc_.getRow(2).getX(), R_wc_.getRow(2).getY(), R_wc_.getRow(2).getZ();
    Eigen::Matrix4d Rt_wc = Eigen::Matrix4d::Identity();
    Rt_wc.block<3,3>(0,0) = R_wc;
    Rt_wc.block<3,1>(0,3) = t_wc;
    /** 计算尺度因子 */
    double Zc = -t_wc[2] / (R_wc(2,0)*P_nlz.x + R_wc(2,1)*P_nlz.y + R_wc(2,2));
    /** 像素系转换到图像系 */
    Eigen::Vector4d P_cam;
    P_cam[0] = P_nlz.x * Zc;
    P_cam[1] = P_nlz.y * Zc;
    P_cam[2] = P_nlz.z * Zc;
    P_cam[3] = 1;
    /** 相机系转换至世界系 */
    Eigen::Vector4d P_world_;
    P_world_ = Rt_wc * P_cam;
    /** fill up result */
    cv::Point3d P_world;
    P_world.x = P_world_[0];
    P_world.y = P_world_[1];
    P_world.z = P_world_[2];
    return P_world;
}

cv::Point2d Projection::world2pixel(const cv::Point3d P_world, ros::Time timeStamp)
{
    /** 计算世界系转换至相机系的旋转平移矩阵 */
    tf::StampedTransform TF_cw;
    listener.waitForTransform("front_cam_optical_frame", "world", timeStamp, ros::Duration(1.5));
    listener.lookupTransform("front_cam_optical_frame", "world", timeStamp, TF_cw);
    Eigen::Vector3d t_cw;
    t_cw << TF_cw.getOrigin().getX(), TF_cw.getOrigin().getY(), TF_cw.getOrigin().getZ();
    Eigen::Matrix3d R_cw;
    tf::Matrix3x3 R_cw_ = TF_cw.getBasis();
    R_cw << R_cw_.getRow(0).getX(), R_cw_.getRow(0).getY(), R_cw_.getRow(0).getZ(),
             R_cw_.getRow(1).getX(), R_cw_.getRow(1).getY(), R_cw_.getRow(1).getZ(),
             R_cw_.getRow(2).getX(), R_cw_.getRow(2).getY(), R_cw_.getRow(2).getZ();
    Eigen::Matrix4d Rt_cw = Eigen::Matrix4d::Identity();
    Rt_cw.block<3,3>(0,0) = R_cw;
    Rt_cw.block<3,1>(0,3) = t_cw;
    Eigen::Vector4d P_world_;
    P_world_ << P_world.x, P_world.y, P_world.z, 1;
    Eigen::Vector4d P_cam_;
    P_cam_ = Rt_cw * P_world_;
    // cv::Point3d P_nlz(P_cam_[0]/P_cam_[2], P_cam_[1]/P_cam_[2], 1);
    cv::Point3d P_cam(P_cam_[0], P_cam_[1], P_cam_[2]);
    double fx = K_intr(0,0);
    double fy = K_intr(1,1);
    double u0 = K_intr(0,2);
    double v0 = K_intr(1,2);
    cv::Point2d P_pix;
    P_pix.x = fx * P_cam.x / P_cam.z + u0;
    P_pix.y = fy * P_cam.y / P_cam.z + v0;
    // cv::Rect2d region(P_pix.x - 0.5 * width, P_pix.y - 0.5 * height, width, height);  // lt-x, lt-y, w, h
    return P_pix;
}
///////////////////////1107/////////////////////////////

// cv::Rect2d Projection::world2pixel(const Eigen::Matrix4d Rt_wc, const cv::Point3d P_world, \
//     const double width, const double height)
// {
//     /** 求解旋转平移矩阵的逆 */
//     Eigen::Matrix3d R_wc = Rt_wc.block<3, 3>(0, 0);
//     Eigen::Vector3d t_wc = Rt_wc.block<3, 1>(0, 3);
//     Eigen::Matrix3d R_cw = R_wc.transpose(); 
//     Eigen::Vector3d t_cw = -R_cw * t_wc; 
//     /** 构建逆变换矩阵 */
//     Eigen::Matrix4d Rt_cw = Eigen::Matrix4d::Identity();
//     Rt_cw.block<3, 3>(0, 0) = R_cw; 
//     Rt_cw.block<3, 1>(0, 3) = t_cw;
//     /** 构建齐次目标三维点 */ 
//     Eigen::Vector4d P_world_ = Eigen::Vector4d::Ones();
//     P_world_.segment<3>(0) = Eigen::Vector3d(P_world.x, P_world.y, P_world.z);
//     /** 世界系转换到相机系 */
//     Eigen::Vector4d P_cam_ = Rt_cw * P_world_;
//     cv::Point3d P_cam(P_cam_[0], P_cam_[1], P_cam_[2]);
//     /** 图像系转换到像素系 */
//     cv::Point2d P_pix;
//     double fx = K_intr(0, 0);
//     double fy = K_intr(1, 1);
//     double u0 = K_intr(0, 2);
//     double v0 = K_intr(1, 2);
//     P_pix.x = fx * (P_cam.x / P_cam.z) + u0;
//     P_pix.y = fy * (P_cam.y / P_cam.z) + v0;
//     /** 根据点设置区域 */
//     cv::Rect2d region(P_pix.x, P_pix.y, width, height);
//     return region;
// }

void Projection::transformCoordinate(darknet_ros_msgs::BoundingBox &bbx, 
        const int std_w, const int std_h, const int ori_w, const int ori_h)
{
    double w_ratio = (double)std_w / ori_w;
    double h_ratio = (double)std_h / ori_h;
    bbx.xmin = int(w_ratio * bbx.xmin);
    bbx.xmax = int(w_ratio * bbx.xmax);
    bbx.ymin = int(h_ratio * bbx.ymin);
    bbx.ymax = int(h_ratio * bbx.ymax); 
}