/*
 * main.cpp
 *
 *	TODO: Text
 *
 *  Created on: Feb 26, 2012
 *      Author: Nikolas Engelhard
 */



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
#include <pcl/common/transform.h>

#include "cloud_processing.h"


ros::Publisher pub;

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;


int cnt=0;
vector<CvPoint> mask_points;



const CvSize C_checkboard_size = cvSize(6,4);
IplImage *mask_image;
bool depth_mask_valid;

// trafo cloud s.t. checkerboard is z=0,  middle of board at x=y=0 and parallel to image axis
// the first trafo is stored and used for all following frames
Eigen::Affine3f kinect_trafo;
bool kinect_trafo_valid = false;


// define this to compute the depth-mask from the detected checkerbord
// if not defined, the user can select four points manually to define the mask
// #define MASK_FROM_DETECTIONS


void on_mouse( int event, int x, int y, int flags, void* param ){
	vector<CvPoint>* vec = (vector<CvPoint>*)param;
	if (event == CV_EVENT_LBUTTONUP) vec->push_back(cvPoint(x,y));
}

// 255 for interesting region
void createMaskFromPoints(IplImage* mask,const vector<CvPoint>& points){
	cvSet(mask,cvScalarAll(0));
	cvFillConvexPoly(mask, &points[0],points.size(),CV_RGB(255,255,255));
}

// assumption: checkerboard is aligned with image axis!
void createMaskFromCheckerBoardDetections(IplImage* mask,const CvPoint2D32f* pts, CvSize boardSize){
	cvSet(mask,cvScalarAll(0));
	int w = boardSize.width; int h = boardSize.height;
	int l = pts[1].x-pts[0].x; // length of a square

	float min_x = pts[0].x-l; 		float min_y = pts[0].y-l;
	float max_x = pts[w*h-1].x+l; float max_y = pts[w*h-1].y+l;
	cvRectangle(mask, cvPoint(min_x,min_y),cvPoint(max_x,max_y), CV_RGB(255,255,255),-1);
}


void showMaskOnImage(IplImage* col, IplImage* mask){
	assert(col); assert(mask); assert(col->width == mask->width);
	for (int x=0; x<col->width; ++x)
		for (int y=0; y<col->height; ++y){
			if (cvGet2D(mask, y,x).val[0] == 255) continue;
			CvScalar c = cvGet2D(col,y,x);
			for (int i=0; i<3; ++i) c.val[i]/=2;
			cvSet2D(col, y,x,c);
		}
}


void callback(const ImageConstPtr& img_ptr, const sensor_msgs::PointCloud2ConstPtr& cloud_ptr){

	//	cout << "CALLBACK" << endl;
	sensor_msgs::CvBridge bridge;

	IplImage* col = bridge.imgMsgToCv(img_ptr, "bgr8");


	CvPoint2D32f corners[C_checkboard_size.width*C_checkboard_size.height];
	int c_cnt=0;
	bool board_found = false;

	if (!kinect_trafo_valid){

		cout << "no trafo" << endl;

		IplImage* gray = cvCreateImage(cvGetSize(col),col->depth, 1);
		cvCvtColor(col,gray, CV_BGR2GRAY);
		int found = cvFindChessboardCorners(col, C_checkboard_size,corners, &c_cnt);
		cvFindCornerSubPix(gray, corners, c_cnt,cvSize(5,5),cvSize(1,1),cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10,0.1));
		cvDrawChessboardCorners(col, C_checkboard_size, corners, c_cnt,found);

		board_found = (c_cnt == C_checkboard_size.width*C_checkboard_size.height);
	}


#ifdef MASK_FROM_DETECTIONS
	if (board_found && !depth_mask_valid){
		createMaskFromCheckerBoardDetections(mask_image,corners,C_checkboard_size);
		cvNamedWindow("mask");
		cvShowImage("mask", mask_image);
		cvSaveImage("data/mask.png", mask_image);
		cout << "new mask was saved to data/mask.png" << endl;
		depth_mask_valid = true;
	}
#else
	if (!depth_mask_valid){
		if (mask_points.size() == 4){
			createMaskFromPoints(mask_image, mask_points);
			cvSaveImage("data/mask.png", mask_image);
			cout << "new mask was saved to data/mask.png" << endl;
			depth_mask_valid = true;
		}
		cvNamedWindow("mask");
		cvShowImage("mask", mask_image);
	}
#endif

	if (depth_mask_valid)
		showMaskOnImage(col, mask_image);


	cvShowImage("view", col);
	cvWaitKey(10);


	if (!board_found && !kinect_trafo_valid){
		ROS_INFO("Checkerboard was not found!");
		return;
	}

	if (!depth_mask_valid) return;

	// fit plane to pointcloud:
	Cloud cloud;
	pcl::fromROSMsg(*cloud_ptr, cloud);

	Cloud filtered;
	applyMask(cloud, filtered,mask_image);


	if (!kinect_trafo_valid){

		Eigen::Vector4f model;
		fitPlaneToCloud(filtered, model);

		// project the detected corners to the plane
		Cloud projected;
		bool valid = projectToPlane(corners, C_checkboard_size, cloud, model, projected); // false if one corner has no depth
		if (!valid) {ROS_WARN("invalid"); return; }

		Vector3f center, upwards, right;
		defineAxis(projected, center, upwards, right);

		pcl::getTransformationFromTwoUnitVectorsAndOrigin(-right, (right).cross(upwards), center, kinect_trafo);
		kinect_trafo_valid = true;
		ROS_INFO("found checkerboard and computed trafo");

	}

	Cloud trans;
	pcl::getTransformedPointCloud(filtered, kinect_trafo, trans);


	// mark points close to the board
#define MINMAX
#ifdef MINMAX
	float minz = 100;
	float maxz = -100;
#endif

	for (uint i=0; i<trans.points.size(); ++i){
		Point p = trans.points[i];
#ifdef MINMAX
		minz = min(minz,p.z);
		maxz = max(maxz,p.z);
#endif
		if (p.z < 0.02 || p.z > 0.06) continue;

		uint8_t r,g,b; r = g= b= 0;

		if (p.z < 0.04){
			g = 255;
		}
		else
			if (p.z < 0.06){
				r = 255;
			}

		int32_t rgb = (r << 16) | (g << 8) | b;
		trans.points[i].rgb =  *(float *)(&rgb);

	}

#ifdef MINMAX
	printf("min,max %f %f \n", minz, maxz);
#endif

	Cloud::Ptr msg = trans.makeShared();
	msg->header.frame_id = "/openni_rgb_optical_frame";
	msg->header.stamp = ros::Time::now ();
	pub.publish(msg);

}



int main(int argc, char ** argv)
{
	ros::init(argc, argv, "subscriber");
	ros::NodeHandle nh;
	cvNamedWindow("view");

	pub = nh.advertise<Cloud> ("projected", 1);



	// check if depth-mask exists:
	mask_image = cvLoadImage("data/mask.png",0);
	depth_mask_valid = (mask_image != NULL);
	if (!depth_mask_valid){ // file was not found
		mask_image = cvCreateImage(cvSize(640,480), IPL_DEPTH_8U, 1);
		cerr << "no depth-mask found" << endl;
#ifdef MASK_FROM_DETECTIONS
		cout << "mask will be computed from first full view of checkerboard" << endl;
#else
		cout << "select for points in image to create new mask!" << endl;
		cvSetMouseCallback("view",on_mouse,&mask_points);
#endif
	}else{
		cout << "depth-mask was loaded from data/mask.png" << endl;
	}



	cvStartWindowThread();







	typedef sync_policies::ApproximateTime<Image, PointCloud2> policy;
	message_filters::Subscriber<Image> image_sub(nh, "/camera/rgb/image_color", 5);
	message_filters::Subscriber<PointCloud2> cloud_sub(nh, "/camera/rgb/points", 5);
	Synchronizer<policy> sync(policy(10), image_sub, cloud_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2));




	ros::spin();

	cvDestroyWindow("view");

	return 0;



}

