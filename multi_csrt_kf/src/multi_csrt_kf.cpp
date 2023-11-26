#include "multi_csrt_kf.h"

Rect2d weighting(Rect2d detBox, Rect2d csrBox)
{
    Rect2d weightedBox;
    weightedBox.x = 0.5 * detBox.x + 0.5 * csrBox.x;
    weightedBox.y = 0.5 * detBox.y + 0.5 * csrBox.y;
    weightedBox.width = 0.5 * detBox.width + 0.5 * csrBox.width;
    weightedBox.height = 0.5 * detBox.height + 0.5 * csrBox.height;
    return weightedBox;
}

MultiCsrtKf::MultiCsrtKf(ros::NodeHandle &n, bool view_img, int max_age, int min_hits, \
    float psr_thresh, float padding, float filter_lr, int template_size) : 
    viewImg(view_img), maxAge(max_age), minHits(min_hits), psrThresh(psr_thresh), 
    Padding(padding), filterLR(filter_lr), templateSize(template_size)
{
    // pub = n.advertise<geometry_msgs::PoseArray>("/result", 1);
}

void MultiCsrtKf::callback (const sensor_msgs::ImageConstPtr &img_ptr, 
                            const darknet_ros_msgs::BoundingBoxes::ConstPtr &detect_msg)
{
    auto timeStamp = std::min(detect_msg->header.stamp, img_ptr->header.stamp);
    cv_ptr = cv_bridge::toCvCopy(img_ptr, sensor_msgs::image_encodings::BGR8);
    int ori_w = cv_ptr->image.cols;
    int ori_h = cv_ptr->image.rows;
    cv::Mat img;
    cv::resize(cv_ptr->image, img, cv::Size(std_w, std_h));

    dets.clear();
    int bbxNum = detect_msg->bounding_boxes.size();
    for (int i = 0; i < bbxNum; i++)
    {
        auto bbx = detect_msg->bounding_boxes[i];
        Rect2d det(bbx.xmin, bbx.ymin, bbx.xmax-bbx.xmin, bbx.ymax-bbx.ymin); // l,t,w,h
        dets.push_back(det);
    }

    if (firstFrame)
    {
        int id = 0;
        for (int detIdx=0; detIdx < dets.size(); detIdx++)
        {
            Track track = createTrack(detIdx, img, timeStamp);
            track.id = id;
            buffer.push_back(track);
            id += 1;
        }
        firstFrame = false;
    }
    else
    {
        trks.clear();
        for (auto it=buffer.begin(); it!=buffer.end();)
        {
            Rect2d pred = it->kf.predict();
            if (pred.x >=0 && pred.y >=0)
            {
                it->pred = pred;
                it++;
            }
            else {
                it++;
            }
        }
        for (auto it=buffer.begin(); it!=buffer.end(); ++it)
        {
            Rect2d pred = it->pred;
            bool active = it->csrDCF->update(img, pred);
            it->active = active;
            it->roi = pred; //pred:更新成功为当前帧最大相应值位置，更新失败为kf上一帧预测位置
            trks.push_back(pred);
        }

        AssocDets2Trks(dets, trks, newDets, inActive, Active);

        for (auto it=Active.begin(); it!=Active.end(); it++)
        {
            int & trkIdx = it->first;
            int & detIdx = it->second;
            //用csrt的搜索结果与检测加权取平均，更新卡尔曼滤波器
            Rect2d weightedBox = weighting(dets[detIdx], buffer[trkIdx].roi);
            buffer[trkIdx].kf.update(weightedBox);
            buffer[trkIdx].age =0;
            buffer[trkIdx].hitStreak +=1;   
        }

        int maxID=0, trackID=0;
        for (int detIdx : newDets)
        {
            Track track = createTrack(detIdx, img, timeStamp);
            if (buffer.size() ==0)
            {
                trackID = globalMaxID +1;
            }
            else {
                auto maxIt = buffer.rbegin();
                maxID = maxIt->id;
                trackID = maxID +1;
                globalMaxID = trackID;
            }
            track.id = trackID;
            buffer.push_back(track);
        }

        for (int trkIdx : inActive)
        {
            auto trkIt = buffer.begin();
            std::advance(trkIt, trkIdx);
            trkIt->age +=1;
            trkIt->hitStreak =0;
        }

        for (auto it=buffer.begin(); it!=buffer.end();)
        {
            if ((it != buffer.end()) && (it->age >= maxAge))
            {
                it = buffer.erase(it);
            }
            else
            {
                it++;
            }
        }
    }
    //////////////////////////
    //////////imshow//////////
    //////////////////////////
    int txtThick = 1;
    int boxThick = 2;
    float txtScale = 0.5f;
    cv::Scalar txtRGB(255,255,0);
    cv::Scalar boxRGB(0,255,0);
    for (auto it=buffer.begin(); it!=buffer.end(); ++it)
    {
        Track &track = (*it); 
        if (track.active)
        {
            int id = track.id;
            string idContent = "ID-" + to_string(id);
            cv::Rect2d roi = track.roi; 
            Point idPosition(roi.x, roi.y-3);
            cv::Point lt(roi.x, roi.y), rb(roi.x + roi.width, roi.y + roi.height);
            cv::rectangle(img, lt, rb, boxRGB, boxThick);
            cv::putText(img, idContent, idPosition, FONT_HERSHEY_SIMPLEX, txtScale, txtRGB, txtThick, LINE_AA);
        }
        string topContent = "IDSW:" + to_string(globalMaxID);
        Point topPosition(500, 40);
        cv::putText(img, topContent, topPosition, FONT_HERSHEY_SIMPLEX, 1.5*txtScale, txtRGB, 2*txtThick, LINE_AA);
    }
    cv::imshow("tracking", img);
    cv::waitKey(1);
}


int main (int argc, char** argv)
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

    MultiCsrtKf mck(n, view_img, max_age, min_hits, psr_thresh, padding, filter_lr, template_size);
    cout << "multi_csrt_kf is ready" << endl;

    Subscriber<Image> image_sub(n, "/yolo/img", 10);
    Subscriber<BoundingBoxes> detect_sub(n, "/yolo/bbx", 10);
    
    typedef sync_policies::ApproximateTime<Image, BoundingBoxes> SyncPolicy;
    Synchronizer<SyncPolicy> sync(SyncPolicy(10), image_sub, detect_sub);
    sync.registerCallback(boost::bind(&MultiCsrtKf::callback, &mck, _1, _2));

    ros::spin();
    
    return 0;
}