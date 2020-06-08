/******************************************
 * OpenCV Tutorial: Ball Tracking using   *
 * Kalman Filter                          *
 ******************************************/

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

using namespace std;

//  Color to be tracked
#define MIN_H_BLUE 200
#define MAX_H_BLUE 300
//  Color to be tracked

using namespace cv;


class KalmanObject{
public:
KalmanObject():kf(stateSize,measSize,contrSize,type),state(stateSize,1,type),meas(stateSize,1,type){

    unsigned int type = CV_32F;
    cv::setIdentity(kf.transitionMatrix);

    //kf(stateSize,measSize,contrSize,type);
    kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
    kf.measurementMatrix.at<float>(0,0) = 1.0f;
    kf.measurementMatrix.at<float>(1,1) = 1.0f;
    kf.measurementMatrix.at<float>(2,4) = 1.0f;
    kf.measurementMatrix.at<float>(3,5) = 1.0f;


    //cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
    kf.processNoiseCov.at<float>(0,0) = 1e-2;
    kf.processNoiseCov.at<float>(1,1) = 1e-2;
    kf.processNoiseCov.at<float>(2,2) = 5.0f;
    kf.processNoiseCov.at<float>(3,3) = 5.0f;
    kf.processNoiseCov.at<float>(4,4) = 1e-2;
    kf.processNoiseCov.at<float>(5,5) = 1e-2;
    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));

};
void kalmanpredict(cv::Mat frame){};
void kalmanupdate(){};



private:
    int stateSize = 6;
    int measSize = 4;
    int contrSize = 0;
    unsigned int type = CV_32F;
    cv::KalmanFilter kf;

    cv::Mat state;  // [x,y,v_x,v_y,w,h]
    cv::Mat meas;    // [z_x,z_y,z_w,z_h]

    // Transition State Matrix A




};

int main()
{

   int slider_hue_lo= 0;
   int slider_sat_lo= 0;
   int slider_val_lo= 0;
  int slider_hue_hi= 0;
   int slider_sat_hi= 0;
   int slider_val_hi= 0;
   int slider_red_lo= 0;
   int slider_blue_lo= 0;
   int slider_green_lo= 0;
  int slider_red_hi= 0;
   int slider_blue_hi= 0;
   int slider_green_hi= 0;
   namedWindow("color thresh"); // Create Window
   namedWindow("hsv thresh"); // Create Window
   char TrackbarName[50];
   //sprintf( TrackbarName, "Alpha x %d", alpha_slider_max );
   createTrackbar( "Hue Low", "hsv thresh", &slider_hue_lo, 255);
   createTrackbar( "Saturation Low", "hsv thresh", &slider_sat_lo, 255);
   createTrackbar( "ValueLow", "hsv thresh", &slider_val_lo, 255);
   createTrackbar( "Hue High", "hsv thresh", &slider_hue_hi, 255);
   createTrackbar( "Saturation high", "hsv thresh", &slider_sat_hi, 255);
   createTrackbar( "Value High", "hsv thresh", &slider_val_hi, 255);
   createTrackbar( "Red Low", "color thresh", &slider_red_lo, 255);
   createTrackbar( "Blue Low", "color thresh", &slider_blue_lo, 255);
   createTrackbar( "Green Low", "color thresh", &slider_green_lo, 255);
   createTrackbar( "Red High", "color thresh", &slider_red_hi, 255);
   createTrackbar( "Blue High", "color thresh", &slider_blue_hi, 255);
   createTrackbar( "Green High", "color thresh", &slider_green_hi, 255);
   //on_trackbar( slider_red, 0 );
   //on_trackbar( slider_blue, 0 );
   //on_trackbar( slider_green, 0 );
    // Camera frame
    cv::Mat frame;

    //  Kalman Filter
    int stateSize = 6;
    int measSize = 4;
    int contrSize = 0;

    unsigned int type = CV_32F;
    cv::KalmanFilter kf(stateSize, measSize, contrSize, type);

    cv::Mat state(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
    cv::Mat meas(measSize, 1, type);    // [z_x,z_y,z_w,z_h]

    // Transition State Matrix A
    cv::setIdentity(kf.transitionMatrix);


    kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
    kf.measurementMatrix.at<float>(0,0) = 1.0f;
    kf.measurementMatrix.at<float>(1,1) = 1.0f;
    kf.measurementMatrix.at<float>(2,4) = 1.0f;
    kf.measurementMatrix.at<float>(3,5) = 1.0f;


    //cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
    kf.processNoiseCov.at<float>(0,0) = 1e-2;
    kf.processNoiseCov.at<float>(1,1) = 1e-2;
    kf.processNoiseCov.at<float>(2,2) = 5.0f;
    kf.processNoiseCov.at<float>(3,3) = 5.0f;
    kf.processNoiseCov.at<float>(4,4) = 1e-2;
    kf.processNoiseCov.at<float>(5,5) = 1e-2;

    // Measures Noise Covariance Matrix R
    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
    // <<<< Kalman Filter

    // Camera Index
    int idx = 0;

    // Camera Capture
    cv::VideoCapture source(0);
    //  Camera Settings
    if (!source.isOpened())
    {
        cout << "Webcam not connected.\n" << "Please verify\n";
        return EXIT_FAILURE;
    }

    //source.set(CAP_PROP_FRAME_WIDTH, 1024);
    //source.set(CAP_PROP_FRAME_HEIGHT, 768);
    //  Camera Settings

    cout << "\nHit 'q' to exit...\n";

    char ch = 0;

    double ticks = 0;
    bool found = false;

    int notFoundCount = 0;

    //  Main loop
    while (ch != 'q' && ch != 'Q')
    {
      
        //std::cout<<slider_hue<<"\n";
        double precTick = ticks;
        ticks = (double) cv::getTickCount();

        double dT = (ticks - precTick) / cv::getTickFrequency(); //seconds

        // Frame acquisition
        source >> frame;
	//imshow("Window",frame);
      cv::Mat res;
        frame.copyTo( res );


        //  Noise smoothing
        cv::Mat blur;
        cv::GaussianBlur(frame, blur, cv::Size(5, 5), 3.0, 3.0);


//Optional Color thresholding
cv::Mat frame2;
cv::inRange(blur, cv::Scalar(slider_red_lo,slider_green_lo,slider_blue_lo),cv::Scalar(slider_red_hi, slider_green_hi,slider_blue_hi ), frame2);


//cv::imshow("color thresh window",frame2);

//frame.copyTo(frame2, frame2);

// 
//cv::imshow("color masked",frame2);
        //  HSV conversion
        cv::Mat frmHsv;
        cv::cvtColor(frame, frmHsv, COLOR_BGR2HSV);
        //  HSV conversion

        //HSV thresholding
        // Note: change parameters for different colors
        cv::Mat rangeRes = cv::Mat::zeros(frame.size(), CV_8UC1);
        //frmHsv=frame;
        cv::inRange(frmHsv, cv::Scalar(slider_hue_lo,slider_sat_lo,slider_val_lo),
                    cv::Scalar(slider_hue_hi, slider_sat_hi,slider_val_hi ), rangeRes);

        //  Improving the result
        //cv::erode(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
          Mat element1 = getStructuringElement( 0 , Size( 5,5));
           Mat element2 = getStructuringElement( 0 , Size( 15,15));
        cv::erode(rangeRes, rangeRes, element1, cv::Point(-1, -1), 2);
        //morphologyEx( rangeRes, rangeRes, MORPH_OPEN, element1 );
        morphologyEx( rangeRes, rangeRes, MORPH_OPEN, element1 );
        morphologyEx( rangeRes, rangeRes, MORPH_CLOSE, element2 );
        
        //cv::dilate(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
        //  Improving the result
	//fastNlMeansDenoisingMulti(rangeRes,rangeRes,1,3);

        // Thresholding viewing

        cv::imshow("hsv thresh view", rangeRes);


        //  Contours detection
        vector<vector<cv::Point> > contours;
        cv::findContours(rangeRes, contours, RETR_EXTERNAL,CHAIN_APPROX_NONE);
        //  Contours detection

        //  Filtering
        vector<vector<cv::Point> > balls;
        vector<cv::Rect> ballsBox;
        for (size_t i = 0; i < contours.size(); i++)
        {
         if (contourArea(contours[i])>=400 && contourArea(contours[i])>=3000 )
         {
          std::cout<<contourArea(contours[i])<<"\n";
            cv::Rect bBox;
            bBox = cv::boundingRect(contours[i]);

            float ratio = (float) bBox.width / (float) bBox.height;
            if (ratio > 1.0f)
                {ratio = 1.0f / ratio;}

            // Searching for a bBox almost square
            if (ratio > 0.75 && bBox.area() >= 400)
            {
                balls.push_back(contours[i]);
                ballsBox.push_back(bBox);
            }
          }
        }
        //  Filtering

        cout << "Balls found:" << ballsBox.size() << endl;

      
        for (size_t i = 0; i < balls.size(); i++)
        {
	   
            cv::drawContours(res, balls, i, CV_RGB(20,150,20), 1);
            cv::rectangle(res, ballsBox[i], CV_RGB(0,255,0), 2);

            cv::Point center;
            center.x = ballsBox[i].x + ballsBox[i].width / 2;
            center.y = ballsBox[i].y + ballsBox[i].height / 2;
            cv::circle(res, center, 2, CV_RGB(20,150,20), -1);

            stringstream sstr;
            sstr << "(" << center.x << "," << center.y << ")";
            cv::putText(res, sstr.str(),
                        cv::Point(center.x + 3, center.y - 3),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(20,150,20), 2);
        }
    



	        if (found)
  {
            //  Matrix A
            kf.transitionMatrix.at<float>(0,2) = dT;
            kf.transitionMatrix.at<float>(1,3) = dT;
            // <<<< Matrix A

            cout << "dT:" << endl << dT << endl;

            state = kf.predict();
            cout << "State post:" << endl << state << endl;

            cv::Rect predRect;
            predRect.width = state.at<float>(4);
            predRect.height = state.at<float>(5);
            predRect.x = state.at<float>(0) - predRect.width / 2;
            predRect.y = state.at<float>(1) - predRect.height / 2;

            cv::Point center;
            center.x = state.at<float>(0);
            center.y = state.at<float>(1);
            cv::circle(res, center, 2, CV_RGB(255,0,0), -1);

            cv::rectangle(res, predRect, CV_RGB(255,0,0), 2);
	      
}


        //  Kalman Update
        if (balls.size() == 0)
        {
            notFoundCount++;
            cout << "notFoundCount:" << notFoundCount << endl;
            if( notFoundCount >= 100 )
            {
                found = false;
            }
 
        }
	else{        

/*
	    for(int i=0; i<ballsBox.size();i++)
          { 
//To make kalman filter work as expected get this out of the if-else and paste it before ifelse
  if(found){
           //  Matrix A
            kf.transitionMatrix.at<float>(0,2) = dT;
            kf.transitionMatrix.at<float>(1,3) = dT;
            // <<<< Matrix A

            cout << "dT:" << endl << dT << endl;

            state = kf.predict();
            cout << "State post:" << endl << state << endl;

            cv::Rect predRect;
            predRect.width = state.at<float>(4);
            predRect.height = state.at<float>(5);
            predRect.x = state.at<float>(0) - predRect.width / 2;
            predRect.y = state.at<float>(1) - predRect.height / 2;

            cv::Point center;
            center.x = state.at<float>(0);
            center.y = state.at<float>(1);
            cv::circle(res, center, 2, CV_RGB(255,0,0), -1);

            cv::rectangle(res, predRect, CV_RGB(255,0,0), 2);
           }    

*/
            notFoundCount = 0;

            meas.at<float>(0) = ballsBox[0].x + ballsBox[0].width / 2;
            meas.at<float>(1) = ballsBox[0].y + ballsBox[0].height / 2;
            meas.at<float>(2) = (float)ballsBox[0].width;
            meas.at<float>(3) = (float)ballsBox[0].height;

            /*if (!found) // First detection!
            {
                // Initialization
                kf.errorCovPre.at<float>(0) = 1; // px
                kf.errorCovPre.at<float>(7) = 1; // px
                kf.errorCovPre.at<float>(14) = 1;
                kf.errorCovPre.at<float>(21) = 1;
                kf.errorCovPre.at<float>(28) = 1; // px

                kf.errorCovPre.at<float>(35) = 1; // px
                state.at<float>(0) = meas.at<float>(0);
                state.at<float>(1) = meas.at<float>(1);
                state.at<float>(2) = 0;
                state.at<float>(3) = 0;
                state.at<float>(4) = meas.at<float>(2);
                state.at<float>(5) = meas.at<float>(3);
                // <<<< Initialization

                kf.statePost = state;
 
                found = true;
            }
            else*/
                kf.correct(meas); // Update

            cout << "Measure matrix:" << endl << meas << endl;
        }


       //kalman tracked frame
        cv::imshow("Tracking", res);

        //getKey
        ch = cv::waitKey(1);
    }

    return EXIT_SUCCESS;
}
