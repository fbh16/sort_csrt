#include "callback_v3.h"

SubPub::SubPub (ros::NodeHandle &n, bool view_img, int max_age, int min_hits, \
    float psr_thresh, float padding, float filter_lr, int template_size) : 
    viewImg(view_img), maxAge(max_age), minHits(min_hits), psrThresh(psr_thresh), 
    Padding(padding), filterLR(filter_lr), templateSize(template_size)
{
    pub = n.advertise<geometry_msgs::PoseArray>("/result", 1);
}

void SubPub::callback (const nav_msgs::Odometry::ConstPtr &drone_pose,
                       const sensor_msgs::ImageConstPtr &img_ptr, 
                       const darknet_ros_msgs::BoundingBoxes::ConstPtr &detect_msg)
{ 

    auto timeStamp = std::min(detect_msg->header.stamp, drone_pose->header.stamp);
    cv_ptr = cv_bridge::toCvCopy(img_ptr, sensor_msgs::image_encodings::BGR8);
    int ori_w = cv_ptr->image.cols;
    int ori_h = cv_ptr->image.rows;
    cv::Mat img;
    cv::resize(cv_ptr->image, img, cv::Size(std_w, std_h));
    ////////////////////
    // cout << "==========" << timeStamp << "==========" << endl;
    /** P0: group detects */
    dets.clear();
    int bbxNum = detect_msg->bounding_boxes.size();
    for (int i = 0; i < bbxNum; i++)
    {
        auto bbx = detect_msg->bounding_boxes[i];
        Rect2d det(bbx.xmin, bbx.ymin, bbx.xmax-bbx.xmin, bbx.ymax-bbx.ymin); // l,t,w,h
        dets.push_back(det);
    }
    cout << "det Num:" << dets.size() << endl;
    for (int i=0; i<dets.size(); i++)
    {
        cout << dets[i].x << " " << dets[i].y << endl;
    }
    /**  */
    if (firstFrame)
    {
        int id = 0;
        for (int detIdx=0; detIdx < dets.size(); detIdx++)
        {
            Track track = createTrack(detIdx, img, timeStamp);
            track.id = id;
            Buffer.push_back(track);
            id += 1;
        }
        firstFrame = false;
    }
    else
    {
    /**  */
    trks.clear();
    int zoneW = 250, zoneH = 250;
    // cout << "KF prediction: "<< endl;
    for (auto it=Buffer.begin(); it!=Buffer.end(); ++it)
    {
        cv::Point3d pred3 = it->kf.trk.predict();
        // cout << pred3.x << " " << pred3.y << " " << pred3.z << endl;
        it->kf.predPoint3 = pred3;
        cv::Point2d pred2 = proj.world2pixel(pred3, timeStamp);
        cout << "pred2: " << pred2.x << " " << pred2.y << endl;
        //显示卡尔曼预测的反投影
        cv::Point kf_predic(pred2.x, pred2.y);
        cv::circle(img, kf_predic, 5, cv::Scalar(0, 0, 255), -1);
        //显示搜索区域
        // cv::Point search_lt(zone.x, zone.y);
        // cv::Point search_rb(zone.x+zone.width, zone.y+zone.height);
        // cv::rectangle(img, search_lt, search_rb, cv::Scalar(255,255,0), 2);
        cv::Rect2d predBox(pred2.x, pred2.y, 0, 0); //cx, cy, w, h
        bool active = it->csrt.trk->update(img, predBox);
        it->csrt.active = active;
        if (active)
        {
            cv::Rect2d roi = predBox;
            it->csrt.roi = roi;
            trks.push_back(roi);
            //显示跟踪结果
            cv::Point lt(roi.x, roi.y);
            cv::Point rb(roi.x + roi.width, roi.y + roi.height);
            cv::rectangle(img, lt, rb, cv::Scalar(0,255,0), 2);
        }
        else 
        {
            trks.push_back(it->csrt.roi); //上一帧跟踪结果
        }
    }

    /**  */
    sort.AssocDets2Trks(dets, trks, newDets, inActive, Active);

    /**  */
    // int maxID=0, TrackID=0;
    // for (int detIdx : newDets)
    // {
    //     Track track = createTrack(detIdx, img, timeStamp);
    //     if (Buffer.size() == 0)
    //     {
    //         TrackID = globalMaxID + 1;
    //     }
    //     else {
    //         auto maxIt = Buffer.rbegin();
    //         maxID = maxIt->id;
    //         TrackID = maxID + 1;
    //         globalMaxID = TrackID;
    //     }
    //     track.id = TrackID;
    //     Buffer.push_back(track);
    // }

    /** P9 */
    for (int trkIdx : inActive)
    {
        auto trkIt = Buffer.begin();
        std::advance(trkIt, trkIdx);
        trkIt->age += 1;
        trkIt->hitStreak = 0;
    }
    
    /** P8 */
    for (auto it=Active.begin(); it!=Active.end(); ++it)
    {
        int & trkIdx = it->first;
        int & detIdx = it->second;
        auto trkIt = (Buffer.begin() + trkIdx);
        trkIt->age = 0;
        trkIt->hitStreak += 1;
        cv::Point3d det3 = proj.pixel2world(dets[detIdx], timeStamp);
        trkIt->kf.trk.update(det3);
    }

    /** P11 */
    for (auto it=Buffer.begin(); it!=Buffer.end();)
    {
        cout << "track" << it->id << ", age: " << it->age << ", hitStreak: " << it->hitStreak << ", BufferSize: " << Buffer.size() << ", roi: " << it->csrt.roi.x << " " << it->csrt.roi.y << " " << it->csrt.roi.width << " " << it->csrt.roi.height << endl;
        if (it->age >= maxAge)
        {
            cout << "//////erase track//////" << it->id << endl;
            it = Buffer.erase(it);
        }
        else
        {
            ++it;
        }
    }
    }
    /////////////////////////
    //////////rviz///////////
    /////////////////////////
    geometry_msgs::PoseArray result;
    result.header.frame_id = "world";
    result.header.stamp = timeStamp;
    for (auto it=Buffer.begin(); it!=Buffer.end(); ++it)
    {
        geometry_msgs::Pose target;
        Track & track = (*it);
        if (track.csrt.active)
        {
            target.position.x = track.kf.predPoint3.x;
            target.position.y = track.kf.predPoint3.y;
            target.position.z = track.kf.predPoint3.z;
            result.poses.push_back(target);
        }
    }
    pub.publish(result);
    /////////////////////////
    ////////imshow///////////
    /////////////////////////
    int txtThick = 1;
    int boxThick = 2;
    float txtScale = 0.5f;
    cv::Scalar txtRGB(255,255,0);
    cv::Scalar boxRGB(0,255,0);
    for (auto it=Buffer.begin(); it!=Buffer.end(); ++it)
    {
        Track &track = (*it);  //45
        if (track.csrt.active)
        {
            int id = track.id;
            string idContent = "ID-" + to_string(id);
            cv::Rect2d roi = track.csrt.roi;  //45
            Point idPosition(roi.x, roi.y-3);
            cv::Point lt(roi.x, roi.y), rb(roi.x + roi.width, roi.y + roi.height);
            cv::rectangle(img, lt, rb, boxRGB, boxThick);
            cv::putText(img, idContent, idPosition, FONT_HERSHEY_SIMPLEX, txtScale, txtRGB, txtThick, LINE_AA);
        }
        string topContent = "MaxID:" + to_string(globalMaxID);
        Point topPosition(500, 40);
        cv::putText(img, topContent, topPosition, FONT_HERSHEY_SIMPLEX, 1.5*txtScale, txtRGB, 2*txtThick, LINE_AA);
    }
    cv::imshow("tracking", img);
    cv::waitKey(1);
    ////////////////////////
    // if (dets.size() > 2)
    // {
    //     ERROR_DETECT += 1;
    // }
    // cout << "ERROR DETECT:" << ERROR_DETECT << endl;
    ///////////////////////
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "Tracking");
    ros::NodeHandle n;
    bool view_img;
    int max_age; 
    int min_hits;
    float psr_thresh;
    float padding;
    float filter_lr;
    int template_size;
    ros::param::get("~view_image", view_img);
    ros::param::get("~psr_thresh", psr_thresh);
    ros::param::get("~padding", padding);
    ros::param::get("~max_age", max_age);
    ros::param::get("~min_hits", min_hits);
    ros::param::get("~filter_lr", filter_lr);
    ros::param::get("~template_size", template_size);
    SubPub subpub(n, view_img, max_age, min_hits, psr_thresh, padding, filter_lr, template_size);

    Subscriber<Odometry> pose_sub(n, "/ground_truth/state", 10);
    Subscriber<Image> image_sub(n, "/yolo/img", 10);
    Subscriber<BoundingBoxes> detect_sub(n, "/yolo/bbx", 10);
    
    typedef sync_policies::ApproximateTime<Odometry, Image, BoundingBoxes> SyncPolicy;
    Synchronizer<SyncPolicy> sync(SyncPolicy(10), pose_sub, image_sub, detect_sub);
    sync.registerCallback(boost::bind(&SubPub::callback, &subpub, _1, _2, _3));

    ros::spin();
    
    return 0;
}