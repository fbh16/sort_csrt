///////////////////////////////////////////////////////////////////////////////
// KalmanTracker.h: KalmanTracker Class Declaration

#ifndef KALMAN_H
#define KALMAN_H 2

#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

#define StateType Point3d


// This class represents the internel state of individual tracked objects observed as bounding box.
class KalmanTracker3D
{
public:
	KalmanTracker3D()
	{
		init_kf(StateType());
		time_since_update = 0;
		hits = 0;
		hit_streak = 0;
		age = 0;
		id = kf_count;
	}
	KalmanTracker3D(StateType initPoint)
	{
		init_kf(initPoint);
		time_since_update = 0;
		hits = 0;
		hit_streak = 0;
		age = 0;
		id = kf_count;
		kf_count++;
	}

	~KalmanTracker3D()
	{
		history.clear();
	}

	StateType predict();
	void update(StateType stateMat);
	
	StateType get_state();
	// StateType get_rect_xysr(float cx, float cy, float s, float r);

	static int kf_count;

	int time_since_update;
	int hits;
	int hit_streak;
	int age;
	int id;

private:
	void init_kf(StateType stateMat);

	cv::KalmanFilter kf;
	cv::Mat measurement;

	std::vector<StateType> history;
};




#endif