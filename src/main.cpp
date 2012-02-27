#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/CvBridge.h>
#include <cv_bridge/cv_bridge.h>


#include <sensor_msgs/image_encodings.h>

using namespace std;

int cnt=0;

void colCb(const sensor_msgs::ImageConstPtr& msg){


	sensor_msgs::CvBridge bridge;

	IplImage* col = bridge.imgMsgToCv(msg, "bgr8");
	IplImage* gray = cvCreateImage(cvGetSize(col),col->depth, 1);

	cvCvtColor(col,gray, CV_BGR2GRAY);


	CvPoint2D32f corners[70];
	int c_cnt=0;

	int found = cvFindChessboardCorners(col, cvSize(6,4),corners, &c_cnt);
	cout << "found " << c_cnt << " corners" << endl;
	cvFindCornerSubPix(gray, corners, c_cnt,cvSize(5,5),cvSize(1,1),cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10,0.1));
	cvDrawChessboardCorners(col, cvSize(6,4), corners, c_cnt,found);

	cvShowImage("view", col);

	cvWaitKey(10);


}



int main(int argc, char ** argv)
{
	ros::init(argc, argv, "subscriber");
	ros::NodeHandle nh;
		cvNamedWindow("view");

	cvStartWindowThread();

	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("camera/rgb/image_color",10,colCb);


	ros::spin();

	cvDestroyWindow("view");

	return 0;



}

