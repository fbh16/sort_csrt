#include "callback.h"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, \
    darknet_ros_msgs::BoundingBoxes, nav_msgs::Odometry> SyncPolicy;
typedef shared_ptr<message_filters::Synchronizer<SyncPolicy>> Synchronizer;

SubPub::SubPub (ros::NodeHandle &n, bool view_img) : viewImg(view_img)
{
    pub = n.advertise<geometry_msgs::PointStamped>("/result", 1);
}

void SubPub::callback ( const sensor_msgs::ImageConstPtr &img_ptr, 
                        const darknet_ros_msgs::BoundingBoxes::ConstPtr &detect_msg, 
                        const nav_msgs::Odometry::ConstPtr &drone_pose)
{
    /** process image stream */ 
    auto timeStamp = std::min(detect_msg->header.stamp, drone_pose->header.stamp);
    cout << "==========" << timeStamp << "==========" << endl;
    cv_ptr = cv_bridge::toCvCopy(img_ptr, sensor_msgs::image_encodings::BGR8);
    int ori_w = cv_ptr->image.cols;
    int ori_h = cv_ptr->image.rows;
    cv::Mat img;
    cv::resize(cv_ptr->image, img, cv::Size(std_w, std_h));
    /** group detects */
    dets.clear();
    int bbxNum = detect_msg->bounding_boxes.size();
    for (int i = 0; i < bbxNum; i++)
    {
        auto bbx = detect_msg->bounding_boxes[i];
        // proj.transformCoordinate(bbx, std_w, std_h, ori_w, ori_h);
        Rect2d det(bbx.xmin, bbx.ymin, bbx.xmax-bbx.xmin, bbx.ymax-bbx.ymin); // l,t,w,h
        dets.push_back(det);
    }
    /** group tracks */
    trks.clear();
    for (auto it=trackBuffer.begin(); it!=trackBuffer.end(); it++)
    {
        Rect2d roi = (*it).second.csrt.roi;
        trks.push_back(roi);
    }
    /** Hungarain association */
    sort.AssocDets2Trks(dets, trks, newDets, pairs);
    /** initialization */
    int maxID, trackID;
    int globalMaxID = 0;
    for (auto idx : newDets)
    {
        // 初始化csrt
        TrackerCSRT::Params param;
        param.psr_threshold = 0.1f; // default to 0.035f
        param.template_size = 250; // default to 200
        Ptr<TrackerCSRT> csrtTrack = TrackerCSRT::create(param);
        csrtTrack->init(img, dets[idx]);
        // fill up csrt
        Track track;
        track.csrt.trk = csrtTrack;
        track.csrt.roi = dets[idx];
        // 初始化kf 
        Projection::Result result;
        result = proj.pixel2world(dets[idx], timeStamp);
        KalmanTracker3D kfTrack = KalmanTracker3D(result.point);
        Point3d predPoint3 = kfTrack.predict();
        Rect2d predBox2 = proj.world2pixel(result.Rt_wc, predPoint3, dets[idx].width, dets[idx].height);
        // fill up kf 
        track.kf.trk = kfTrack;
        track.kf.predBox2 = predBox2;
        track.kf.predPoint3 = predPoint3;
        // set ID 
        if (trackBuffer.empty())
        {
            trackID = globalMaxID;
        }
        else 
        {
            auto maxIt = trackBuffer.rbegin();
            maxID = maxIt->first;
            globalMaxID = max(globalMaxID, maxID);
            trackID = globalMaxID + 1;
        }
        trackBuffer[trackID] = track;
    }
    /** update */
    for (auto pair : pairs)
    {
        int trkIdx = pair.first;
        int detIdx = pair.second;
        auto it = trackBuffer.begin();
        std::advance(it, trkIdx);
        int trackID = (*it).first;
        auto &track = trackBuffer[trackID];
        /** update csrt */
        track.csrt.trk->update(img, track.kf.predBox2);
        /** update kf */
        Point3d det3d;
        Projection::Result result;
        if (detIdx != -1)
        {
            // 当前帧目标被检测到，用检测信息更新KF
            result = proj.pixel2world(dets[detIdx], timeStamp);
            det3d = result.point;
            track.kf.trk.update(det3d);
        } else {
            // 当前帧目标被遮挡或消失,用上一帧的预测更新KF
            track.kf.trk.update(track.kf.predPoint3);
        }
        /** re-projection */
        Point3d predPoint3 = track.kf.trk.predict();
        Eigen::Matrix4d &Rt_wc = result.Rt_wc;
        Rect2d predBox2 = proj.world2pixel(Rt_wc, predPoint3, dets[detIdx].width, dets[detIdx].height);
        /** replacement */ 
        track.csrt.roi = dets[detIdx];
        track.kf.predPoint3 = det3d;
        track.kf.predBox2 = predBox2;
        track.kf.predPoint3 = predPoint3;
    }
    /** debug & visualizition */
    for (auto it=trackBuffer.begin(); it!=trackBuffer.end(); it++)
    {
        cv::Point3d &kfPred = (*it).second.kf.predPoint3;
        geometry_msgs::PointStamped result_stamp;
        result_stamp.header.frame_id = "world";
        result_stamp.header.stamp = timeStamp;
        result_stamp.point.x = kfPred.x;
        result_stamp.point.y = kfPred.y;
        result_stamp.point.z = kfPred.z;  
        pub.publish(result_stamp);
    }
    if (viewImg)
    {
        cv::Scalar txtRGB(255,255,0);
        cv::Scalar boxRGB(0,255,0);
        int txtThick = 1;
        int boxThick = 2;
        float txtScale = 0.5f;
        for (auto it=trackBuffer.begin(); it!=trackBuffer.end(); it++)
        {
            int id = (*it).first;
            string idContent = "id-" + to_string(id);
            cv::Rect2d roi = (*it).second.csrt.roi;
            cv::Point lt(roi.x, roi.y);
            cv::Point rb(roi.x + roi.width, roi.y + roi.height);
            cv::rectangle(img, lt, rb, boxRGB, boxThick);
            cv::putText(img, idContent, Point(roi.x, roi.y-3), FONT_HERSHEY_SIMPLEX, txtScale, txtRGB, txtThick, LINE_AA);
        }
        cv::imshow("TheBoatOnHorizon", img);
        cv::waitKey(1);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "FusionSearchZoneTracking");
    ros::NodeHandle n;
    bool view_img;
    int max_age; 
    int min_hits;
    ros::param::get("~view_image", view_img);

    SubPub subpub(n, view_img);

    shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> yolo_img;
    shared_ptr<message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes>> yolo_bbx;
    shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> drone_pose;
    Synchronizer sync_img_bbx_pose;
    
    yolo_img.reset(new message_filters::Subscriber<sensor_msgs::Image>(n, "/yolo/img", 1));
    yolo_bbx.reset(new message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes>(n, "/yolo/bbx", 1));
    drone_pose.reset(new message_filters::Subscriber<nav_msgs::Odometry>(n, "/ground_truth/state", 1));
    sync_img_bbx_pose.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *yolo_img, *yolo_bbx, *drone_pose));
    sync_img_bbx_pose->registerCallback(boost::bind(&SubPub::callback, &subpub, _1, _2, _3));

    ros::spin();
    return 0;
}