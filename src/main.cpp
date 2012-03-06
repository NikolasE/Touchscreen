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
#include "calibration.h"
#include "user_input.h"

ros::Publisher pub;

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;



enum Calib_state  {GET_HOMOGRAPHY,COLLECT_PATTERNS,GET_PROJECTION, EVERYTHING_SETUP};

Calib_state prog_state = GET_HOMOGRAPHY;


int cnt=0;
vector<CvPoint> mask_points;

Cloud full_cloud_moved;

CvMat* proj_Hom = cvCreateMat(3,3,CV_32FC1);
IplImage* col;

const CvSize C_checkboard_size = cvSize(6,4);
IplImage *mask_image;
bool depth_mask_valid;

// trafo cloud s.t. checkerboard is z=0,  middle of board at x=y=0 and parallel to image axis
// the first trafo is stored and used for all following frames
Eigen::Affine3f kinect_trafo;
bool kinect_trafo_valid = false;

// region of the projector image where the checkerboard will be drawn
IplImage* board_mask = NULL;
IplImage* projector_image = NULL;
const CvSize C_proj_size = cvSize(1280,1024);
//const CvSize C_proj_size = cvSize(640,480);

vector<CvPoint2D32f> projector_corners; // position of internal corners on the projector image


// coordinate system in plane
Vector3f pl_center, pl_upwards, pl_right;

Cloud corners_3d;



// define this to compute the depth-mask from the detected checkerbord
// if not defined, the user can select four points manually to define the mask
#define MASK_FROM_DETECTIONS


void on_mouse_mask( int event, int x, int y, int flags, void* param ){
	vector<CvPoint>* vec = (vector<CvPoint>*)param;
	if (event == CV_EVENT_LBUTTONUP) vec->push_back(cvPoint(x,y));
}



void on_mouse_projector( int event, int x, int y, int flags, void* param ){

	Cloud* cloud = (Cloud*)param;
	if (event != CV_EVENT_LBUTTONUP) return;

	// get 3d point at this position
	pcl_Point p = cloud->at(x,y);

	if (p.z != p.z) {ROS_WARN("Point has no depth!"); return;}
	if (abs(p.z) > 0.03) {ROS_WARN("Point is not in plane!"); return;}

	CvPoint proj;
	applyHomography(cvPoint2D32f(p.x,p.y),proj_Hom,proj);

	cvCircle(projector_image, proj,20, CV_RGB(255,0,0),-1);

	cvShowImage("board", projector_image);

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


	sensor_msgs::CvBridge bridge;

	col = bridge.imgMsgToCv(img_ptr, "bgr8");

	if (depth_mask_valid){
		showMaskOnImage(col, mask_image);
		cvShowImage("camera", col);
	}else{
		cvShowImage("camera", col);
	}

	short c = cv::waitKey(50);

	if (c == -1) return;

	if (c == 'a')
		prog_state = GET_PROJECTION;



	CvPoint2D32f corners[C_checkboard_size.width*C_checkboard_size.height];
	int c_cnt=0;
	bool board_found = false;

	if (prog_state == GET_HOMOGRAPHY || prog_state == COLLECT_PATTERNS){

		ROS_INFO("Searching for checkerboard");

		cvFindChessboardCorners(col, C_checkboard_size,corners, &c_cnt,CV_CALIB_CB_ADAPTIVE_THRESH);
		board_found = (c_cnt == C_checkboard_size.width*C_checkboard_size.height);

		if (board_found){ ROS_WARN("Checkerboard was detected");
		}else { ROS_WARN("Board was not found!"); return; }

		IplImage* gray = cvCreateImage(cvGetSize(col),col->depth, 1);
		cvCvtColor(col,gray, CV_BGR2GRAY);
		cvFindCornerSubPix(gray, corners, c_cnt,cvSize(5,5),cvSize(1,1),cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10,0.1));
		//		cvDrawChessboardCorners(col, C_checkboard_size, corners, c_cnt,found);
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


	Cloud cloud;
	pcl::fromROSMsg(*cloud_ptr, cloud);


	if (prog_state == GET_HOMOGRAPHY){
		if (!depth_mask_valid){ ROS_INFO("No depth mask!"); return; }

		// fit plane to pointcloud:


		Cloud filtered;
		applyMask(cloud, filtered,mask_image);

		Eigen::Vector4f model;
		fitPlaneToCloud(filtered, model);

		// project the detected corners to the plane
		Cloud projected;
		bool valid = projectToPlane(corners, C_checkboard_size, cloud, model, projected); // false if one corner has no depth
		if (!valid) {ROS_WARN("One of the corner points had no depth!"); return; }


		defineAxis(projected, pl_center, pl_upwards, pl_right);
		pcl::getTransformationFromTwoUnitVectorsAndOrigin(-pl_right,model.head<3>(), pl_center, kinect_trafo);
		// compute homography between screen and Beamer:
		Cloud corners_in_xy_plane;
		pcl::getTransformedPointCloud(projected, kinect_trafo, corners_in_xy_plane);

		computeHomography(projector_corners,corners_in_xy_plane,proj_Hom);
		//		for (uint i=0; i<3; ++i){
		//			for (uint j=0; j<3; ++j){
		//				cout << cvmGet(proj_Hom,i,j) << " "; }
		//			cout << endl; 	}
		ROS_INFO("Computed Homography");

		prog_state = COLLECT_PATTERNS;

		// TODO: Save Homography
	}


	Cloud transformed_cloud;
	if (prog_state == COLLECT_PATTERNS){


		Cloud c_3d;
		// get 3dPose at the corner positions:
		vector<int> inlier;
		for (int i=0; i<c_cnt; ++i){
			pcl_Point p = cloud.at(corners[i].x, corners[i].y);
			if (!(p.x == p.x)){ROS_WARN("projectToPlane: Found Corner without depth!"); return; }
			c_3d.points.push_back(p);
		}


		// trafo into first frame:
		pcl::getTransformedPointCloud(c_3d, kinect_trafo, c_3d);
		// and append to the list of all detections
		corners_3d.points.insert(corners_3d.points.end(),c_3d.points.begin(), c_3d.points.end());

		cout << "corners_3d.size: " << corners_3d.points.size() << endl;


	}


	if (prog_state == GET_PROJECTION){
		computeProjectionMatrix(corners_3d, projector_corners);
	}



	// everything is set up!
	//	pcl::getTransformedPointCloud(cloud,kinect_trafo,full_cloud_moved);
	//	cvSetMouseCallback("view",on_mouse_projector,&full_cloud_moved);


	/*
	// mark points close to the board
	// #define MINMAX
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
		if (p.z < 0.03 || p.z > 0.06) continue;

		uint8_t r,g,b; r = g= b= 0;

		if (p.z < 0.05){
			g = 255;
			// save point on plane
			//			Vector2f px;
			//			transformInPlaneCoordinates(cvPoint3D32f(p.x,p.y,p.z),px, pl_center, pl_upwards, pl_right);

			CvPoint proj;
			applyHomography(cvPoint2D32f(p.x,p.y),proj_Hom,proj);


			//			ROS_INFO("pt: %f %f %f px: %f %f", p.x,p.y,p.z, px.x(), px.y());
			ROS_INFO("projected to: %i %i", proj.x, proj.y);
			cvCircle(projector_image, proj,20, CV_RGB(255,0,0),-1);
			cvShowImage("board", projector_image);

			break;


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

	 */

	Cloud::Ptr msg = transformed_cloud.makeShared();
	msg->header.frame_id = "/openni_rgb_optical_frame";
	msg->header.stamp = ros::Time::now ();
	pub.publish(msg);

}



int main(int argc, char ** argv)
{
	ros::init(argc, argv, "subscriber");
	ros::NodeHandle nh;
	cvNamedWindow("camera", 1);
	//	cvNamedWindow("mask",1);

	// load projector mask and show fullscreen on secondary screen:
	IplImage* board_mask = cvLoadImage("data/proj_mask.png",0);
	if (!board_mask){
		board_mask = cvCreateImage(C_proj_size, IPL_DEPTH_8U,1);
		cvSet(board_mask, cvScalarAll(255));
		ROS_INFO("Found no projector mask, using total area");
	}
	if (board_mask->width !=  C_proj_size.width){
		ROS_ERROR("mask for projector image has not the same size as the projector screen!!");
	}
	// draw board on image and store position of internal corners
	projector_image = cvCreateImage(C_proj_size, IPL_DEPTH_32F, 3);
	drawCheckerboard(projector_image, board_mask, C_checkboard_size,projector_corners);

	cvNamedWindow("board", 0);
	cvMoveWindow("board", 1500, 100); // assuming secondary monitor is right of primary
	cvSetWindowProperty("board", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
	cvShowImage("board", projector_image);




	pub = nh.advertise<Cloud>("projected", 1);



	// check if depth-mask exists:
	mask_image = cvLoadImage("data/mask.png",0);
	depth_mask_valid = (mask_image != NULL);
	if (!depth_mask_valid){ // file was not found
		mask_image = cvCreateImage(cvSize(640,480), IPL_DEPTH_8U, 1);
		ROS_WARN("no depth-mask found");
#ifdef MASK_FROM_DETECTIONS
		ROS_INFO("mask will be computed from first full view of checkerboard");
#else
		ROS_INFO("select four points in image to create new mask!");
		cvSetMouseCallback("view",on_mouse_mask,&mask_points);
#endif
	}else{
		ROS_INFO("depth-mask was loaded from data/mask.png");
	}



	cvStartWindowThread();


	typedef sync_policies::ApproximateTime<Image, PointCloud2> policy;
	message_filters::Subscriber<Image> image_sub(nh, "/camera/rgb/image_color", 5);
	message_filters::Subscriber<PointCloud2> cloud_sub(nh, "/camera/rgb/points", 5);
	Synchronizer<policy> sync(policy(5), image_sub, cloud_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2));




	ros::spin();

	cvDestroyWindow("camera");

	return 0;



}

