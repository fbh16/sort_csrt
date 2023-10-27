#include "kfCSRT.h"
#include <fstream>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> SyncPolicy;
typedef shared_ptr<message_filters::Synchronizer<SyncPolicy>> Synchronizer;

SubPub::SubPub(ros::NodeHandle &n, bool view_img, int max_age, int min_hits) : viewImg(view_img), maxAge(max_age), minHits(min_hits)
{

}

void SubPub::callback (const sensor_msgs::ImageConstPtr &img, const darknet_ros_msgs::BoundingBoxes::ConstPtr &detMsg)
{
    if (initialized) 
    {
        string filename = "pets.txt";
        outFile.open("/home/fbh/2023_goal/factor_graph/src/csrt_kf_new/evaluate/" + filename);
        initialized = false;
    }
    cout << "====================" << endl;
    int std_w = 640;  
    int std_h = 480; 
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    int ori_w = cv_ptr->image.cols;
    int ori_h = cv_ptr->image.rows;
    cv::Mat stdImg;
    cv::resize(cv_ptr->image, stdImg, cv::Size(std_w, std_h));
    int bbs_num = detMsg->bounding_boxes.size();

    // Group detections
    dets.clear();
    for (int i = 0; i < bbs_num; i++)
    {
        auto bbx = detMsg->bounding_boxes[i];
        sortCSRT::transformCoordinate(bbx, std_w, std_h, ori_w, ori_h);
        Rect2d det(bbx.xmin, bbx.ymin, bbx.xmax - bbx.xmin, bbx.ymax - bbx.ymin); // l,t,w,h
        dets.push_back(det);
    }
    cout <<"detections number: " << bbs_num << endl;
    trks.clear();
    for (auto it=allTracks.begin(); it!=allTracks.end(); it++)
    {
        Rect2d kfBox = (*it).second.kfPrediction;
        (*it).second.csrtTrack->update(stdImg, kfBox);
        // Rect2d csrtState = kfBox;
        (*it).second.csrtInfo.roi = kfBox;
        Rect2d std_kfBox = sortCSRT::adjustBbx(kfBox, stdImg);
        trks.push_back(std_kfBox);
    }
    cout << "trks: " << endl;
    for (int i=0; i<trks.size(); i++)
    {
        cout << trks[i].x << " " << trks[i].y << " " << trks[i].width << " " << trks[i].height << endl;
    }
    cout << endl;
    cout << "allTracks: " << endl;
    for (auto it=allTracks.begin(); it!=allTracks.end(); it++)
    {
        cout << (*it).first << " ";
    }
    cout << endl;
    sortCSRT::associateDets2Trks(dets, trks, umDets, mPairs, umTrks);
    cout << "umDets: " << endl;
    for (auto ud : umDets)
    {
        cout << ud << " ";
    }
    cout << endl;
    cout << "mPairs: " << endl;
    for (auto m : mPairs)
    {
        cout << m.first << " " << m.second << endl;
    }
    cout << "umTrks:" << endl;
    for (auto ut : umTrks)
    {
        cout << ut << " ";
    }
    cout << endl;

    sortCSRT::update(dets, allTracks, mPairs, stdImg);

    sortCSRT::initialize(dets, allTracks, umDets, stdImg);

    sortCSRT::predict(allTracks, umTrks);

    sortCSRT::manage(allTracks, umTrks, maxAge);

    if (viewImg)
    {
        int txtThick = 1;
        int boxThick = 2;
<<<<<<< HEAD
        float txtScale = 0.5f;
=======
        float txtScale = 0.5;
>>>>>>> d4c836ab845d2b4773b917bff426f12242d77461
        Scalar boxRGB(0, 255, 0);
        Scalar txtRGB(255,255,0);
        if (!allTracks.empty())
        {
            for (auto it = allTracks.begin(); it != allTracks.end(); it++)
            {
                if ((it->second.csrtInfo.age < maxAge)) //  && (it->second.csrtInfo.hitStreak >= minHits)
                {
                    int id = (*it).first;
                    int age = (*it).second.csrtInfo.age; 
                    string idContent = "ID-" + to_string(id);
                    Rect2d bbx = (*it).second.csrtInfo.roi;
                    Point lt(bbx.x, bbx.y);
                    Point rb(bbx.x + bbx.width, bbx.y + bbx.height);
                    rectangle(stdImg, lt, rb, boxRGB, boxThick);
                    putText(stdImg, idContent, Point(bbx.x, bbx.y-3), FONT_HERSHEY_SIMPLEX, txtScale, txtRGB, txtThick, LINE_AA);
                }
            }
            cv::imshow("ABoatOnTheSkyline", stdImg);
            cv::waitKey(1);
        }
    }

    if (!allTracks.empty())
    {
        for (auto it = allTracks.begin(); it != allTracks.end(); it++)
        {
            if ((it->second.csrtInfo.age < maxAge)) //  && (it->second.csrtInfo.hitStreak >= minHits
            {
                int id = (*it).first;
                int age = (*it).second.csrtInfo.age; 
                Rect2d bbx = (*it).second.csrtInfo.roi;
                outFile << trkFrame << "," << (id+1) << ","
                        << std::fixed << std::setprecision(2)
                        << bbx.x << "," << bbx.y << ","
                        << bbx.width << "," << bbx.height
                        << ",1,-1,-1,-1" << std::endl;
            }
        }
    }
    trkFrame++;
    
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "csrt_kf");
    ros::NodeHandle n;

    bool view_img = false;
    int max_age; 
    int min_hits;
    string img_sub;
    string bbx_sub;
    ros::param::get("~view_img", view_img);
    ros::param::get("~max_age", max_age);
    ros::param::get("~min_hits", min_hits);
    // ros::param::get("~img_sub", img_sub);
    // ros::param::get("~bbx_sub", bbx_sub);   

    SubPub subpub(n, view_img, max_age, min_hits);

    shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> yolo_img;
    shared_ptr<message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes>> yolo_bbx;
    Synchronizer sync_img_bbx;

    yolo_img.reset(new message_filters::Subscriber<sensor_msgs::Image>(n, "yolo/img", 1));
    yolo_bbx.reset(new message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes>(n, "yolo/bbx", 1));
    sync_img_bbx.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *yolo_img, *yolo_bbx));
    sync_img_bbx->registerCallback(boost::bind(&SubPub::callback, &subpub, _1, _2));

    ros::spin();
    return 0;
}