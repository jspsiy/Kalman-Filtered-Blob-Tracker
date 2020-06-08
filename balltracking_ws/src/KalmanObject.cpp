
#include<KalmanObject.h>


KalmanObject::KalmanObject():kf(stateSize,measSize,contrSize,type),state(stateSize,1,type),meas(measSize,1,type){//:kf(stateSize,measSize,contrSize,type),state(stateSize,1,type),meas(measSize,1,type){

//std::cout<<"I was here 1"<<"\n";
    unsigned int type = CV_32F;
    cv::setIdentity(kf.transitionMatrix);

    //kf(stateSize,measSize,contrSize,type);
    kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
    kf.measurementMatrix.at<float>(0,0) = 1.0f;
    kf.measurementMatrix.at<float>(1,1) = 1.0f;
    kf.measurementMatrix.at<float>(2,4) = 1.0f;
    kf.measurementMatrix.at<float>(3,5) = 1.0f;
//std::cout<<"I was here 2"<<"\n";

    //cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
    kf.processNoiseCov.at<float>(0,0) = 1e-2;
    kf.processNoiseCov.at<float>(1,1) = 1e-2;
    kf.processNoiseCov.at<float>(2,2) = 5.0f;
    kf.processNoiseCov.at<float>(3,3) = 5.0f;
    kf.processNoiseCov.at<float>(4,4) = 1e-2;
    kf.processNoiseCov.at<float>(5,5) = 1e-2;
    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
//std::cout<<"I was here 3"<<"\n";
};
KalmanObject::~KalmanObject(){std::cout<<"Kalman Object got destroyed\n";};
cv::Mat KalmanObject::kalmanpredict(cv::Mat res ,double dT){
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

return res;
};

void KalmanObject::kalmanupdate(vector<vector<cv::Point> > balls,vector<cv::Rect> ballsBox){

        if (balls.size() == 0)
        {
            notFoundCount++;
            cout << "notFoundCount:" << notFoundCount << endl;
            if( notFoundCount >= 100 )
            {
                found = false;
            }
 
        }
else
{
            notFoundCount = 0;

            meas.at<float>(0) = ballsBox[0].x + ballsBox[0].width / 2;
            meas.at<float>(1) = ballsBox[0].y + ballsBox[0].height / 2;
            meas.at<float>(2) = (float)ballsBox[0].width;
            meas.at<float>(3) = (float)ballsBox[0].height;

            if (!found) // First detection!
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
            else
                kf.correct(meas); // Update

            cout << "Measure matrix:" << endl << meas << endl;
 
       }



};


