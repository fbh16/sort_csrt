#include "sort.h"

Rect2d weighting(Rect2d detBox, Rect2d predBox)
{
    Rect2d weightedBox;
    weightedBox.x = 0.5 * detBox.x + 0.5 * predBox.x;
    weightedBox.y = 0.5 * detBox.y + 0.5 * predBox.y;
    weightedBox.width = 0.5 * detBox.width + 0.5 * predBox.width;
    weightedBox.height = 0.5 * detBox.height + 0.5 * predBox.height;
    return weightedBox;
}

Sort::Sort(ros::NodeHandle &n, bool view_img, int max_age, int min_hits) : viewImg(view_img), maxAge(max_age), minHits(min_hits)
{
    // pub = n.advertise<geometry_msgs::PoseArray>("/result", 1);
}

void Sort::callback(const sensor_msgs::ImageConstPtr &img_ptr,
                    const darknet_ros_msgs::BoundingBoxes::ConstPtr &detect_msg)
{
    auto timeStamp = std::min(detect_msg->header.stamp, img_ptr->header.stamp);
    cv_ptr = cv_bridge::toCvCopy(img_ptr, sensor_msgs::image_encodings::BGR8);
    int ori_w = cv_ptr->image.cols;
    int ori_h = cv_ptr->image.rows;
    cv::Mat img;
    cv::resize(cv_ptr->image, img, cv::Size(std_w, std_h));
    ////////////////////
    // cout << "==========" << timeStamp << "==========" << endl;

    dets.clear();
    for (int i = 0; i < detect_msg->bounding_boxes.size(); i++)
    {
        auto bbx = detect_msg->bounding_boxes[i];
        Rect2d det(bbx.xmin, bbx.ymin, bbx.xmax - bbx.xmin, bbx.ymax - bbx.ymin); // l,t,w,h
        dets.push_back(det);
    }

    if (firstFrame)
    {
        int id = 0;
        for (int detIdx = 0; detIdx < dets.size(); detIdx++)
        {
            KalmanTracker track = KalmanTracker(dets[detIdx]);
            buffer.push_back(track);
        }
        firstFrame = false;
    }
    else
    {
        trks.clear();
        for (auto it = buffer.begin(); it != buffer.end();)
        {
            Rect2d pred = it->predict();
            if (pred.x >= 0 && pred.y >= 0)
            {
                trks.push_back(pred);
                it++;
            }
            else
            {
                it = buffer.erase(it);
            }
        }

        AssocDets2Trks(dets, trks, newDets, inActive, Active);

        for (unsigned int i = 0; i < Active.size(); i++)
        {
            int trkIdx = Active[i].first;
            int detIdx = Active[i].second;
            Rect2d weightedBox = weighting(dets[detIdx], trks[trkIdx]);
            buffer[trkIdx].update(weightedBox);
        }

        for (int detIdx : newDets)
        {
            KalmanTracker track = KalmanTracker(dets[detIdx]);
            buffer.push_back(track);
        }

        for (auto it = buffer.begin(); it != buffer.end();)
        {
            if ((it != buffer.end()) && (it->time_since_update > maxAge))
            {
                it = buffer.erase(it);
            }
            else {
                it++;
            }
        }
    }
    //////////////////
    //////imshow//////
    //////////////////
    int txtThick = 1;
    int boxThick = 2;
    float txtScale = 0.5f;
    cv::Scalar txtRGB(255, 255, 0);
    cv::Scalar boxRGB(0, 255, 0);
    for (auto it = buffer.begin(); it != buffer.end(); ++it)
    {
        if ((it->time_since_update < 1) && (it->hit_streak >= minHits))
        {
            Rect2d box = it->get_state();
            int id = it->id;
            string idContent = "ID-" + to_string(id);
            Point idPosition(box.x, box.y - 3);
            cv::Point lt(box.x, box.y), rb(box.x + box.width, box.y + box.height);
            cv::rectangle(img, lt, rb, boxRGB, boxThick);
            cv::putText(img, idContent, idPosition, FONT_HERSHEY_SIMPLEX, txtScale, txtRGB, txtThick, LINE_AA);
        }
        string topContent = "IDSW: " + to_string(it->kf_count-1);
        Point topPosition(500, 40);
        cv::putText(img, topContent, topPosition, FONT_HERSHEY_SIMPLEX, 1.5*txtScale, txtRGB, 2*txtThick, LINE_AA);
    }
    cv::imshow("tracking", img);
    cv::waitKey(1);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "sort");
    ros::NodeHandle n;

    bool view_img;
    int max_age;
    int min_hits;
    ros::param::get("~view_image", view_img);
    ros::param::get("~max_age", max_age);
    ros::param::get("~min_hits", min_hits);

    Sort sort(n, view_img, max_age, min_hits);
    std::cout << "SORT is ready" << std::endl;

    Subscriber<Image> image_sub(n, "/yolo/img", 10);
    Subscriber<BoundingBoxes> detect_sub(n, "/yolo/bbx", 10);

    typedef sync_policies::ApproximateTime<Image, BoundingBoxes> SyncPolicy;
    Synchronizer<SyncPolicy> sync(SyncPolicy(10), image_sub, detect_sub);
    sync.registerCallback(boost::bind(&Sort::callback, &sort, _1, _2));

    ros::spin();

    return 0;
}