#ifndef KALMAN_OBJECT_H
#define KALMAN_OBJECT_H


// Module "core"
#include <opencv2/core/core.hpp>

// Module "highgui"
#include <opencv2/highgui/highgui.hpp>

// Module "imgproc"
#include <opencv2/imgproc/imgproc.hpp>

// Module "video"
#include <opencv2/video/video.hpp>
#include <opencv2/photo.hpp>

// Output
#include <iostream>

// Vector
#include <vector>
using namespace cv;
using namespace std;

class KalmanObject{
public:
KalmanObject();
~KalmanObject();
cv::Mat kalmanpredict(cv::Mat res ,double dT);

void kalmanupdate(vector<vector<cv::Point> > balls,vector<cv::Rect> ballsBox);

private:
    int stateSize = 6;
    int measSize = 4;
    int contrSize = 0;
    unsigned int type = CV_32F;
    cv::KalmanFilter kf;
    bool found=false;
    cv::Mat state;  // [x,y,v_x,v_y,w,h]
    cv::Mat meas;    // [z_x,z_y,z_w,z_h]   
    int notFoundCount=0;
	

};

#endif
