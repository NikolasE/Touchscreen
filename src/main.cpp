#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/CvBridge.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <message_filters/time_synchronizer.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

#include <sensor_msgs/image_encodings.h>

#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>



using namespace std;
using namespace sensor_msgs;
using namespace message_filters;


int cnt=0;
vector<CvPoint> mask;

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> Cloud;

const CvSize C_checkboard_size = cvSize(8,6);


void on_mouse( int event, int x, int y, int flags, void* param ){

	vector<CvPoint>* foo = (vector<CvPoint>*)param;

	if (event == CV_EVENT_LBUTTONUP){
		printf("%i %i\n",x,y);
		foo->push_back(cvPoint(x,y));
	}

}



void callback(const ImageConstPtr& img_ptr, const sensor_msgs::PointCloud2ConstPtr& cloud_ptr){


	sensor_msgs::CvBridge bridge;

	IplImage* col = bridge.imgMsgToCv(img_ptr, "bgr8");
	IplImage* gray = cvCreateImage(cvGetSize(col),col->depth, 1);



	cvCvtColor(col,gray, CV_BGR2GRAY);


	CvPoint2D32f corners[70];
	int c_cnt=0;

	int found = cvFindChessboardCorners(col, C_checkboard_size,corners, &c_cnt);
	//	cout << "found " << c_cnt << " corners" << endl;
	cvFindCornerSubPix(gray, corners, c_cnt,cvSize(5,5),cvSize(1,1),cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10,0.1));
	cvDrawChessboardCorners(col, C_checkboard_size, corners, c_cnt,found);


	// draw selected range:
//	for (uint i=0; i<mask.size();++i){
//		CvPoint a = mask[i];
//		CvPoint b = mask[(i+1)%mask.size()];
//		cvLine(col,a,b,CV_RGB(255,0,0),2);
//		cout << a.x << endl;
//	}

	// draw mask over selected area
	cvFillConvexPoly(col, &mask[0],mask.size(),CV_RGB(255,0,0));

	cout << mask.size() << endl;

	cvShowImage("view", col);
	cvWaitKey(10);


  Cloud cloud;
  pcl::fromROSMsg(*cloud_ptr, cloud);


//	ROS_INFO("Got Pointcloud with %i points", cloud.points.size());

}



int main(int argc, char ** argv)
{
	ros::init(argc, argv, "subscriber");
	ros::NodeHandle nh;
	cvNamedWindow("view");

	cvSetMouseCallback("view",on_mouse,&mask);

	cvStartWindowThread();

//	image_transport::ImageTransport it(nh);
//	image_transport::Subscriber sub = it.subscribe(,10,colCb);

	typedef sync_policies::ApproximateTime<Image, PointCloud2> policy;
  message_filters::Subscriber<Image> image_sub(nh, "/camera/rgb/image_color", 5);
  message_filters::Subscriber<PointCloud2> cloud_sub(nh, "/camera/rgb/points", 5);
  Synchronizer<policy> sync(policy(10), image_sub, cloud_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));




	ros::spin();

	cvDestroyWindow("view");

	return 0;



}

