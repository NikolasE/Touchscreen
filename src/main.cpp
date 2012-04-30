/*
 * main.cpp
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

#include <pcl/io/pcd_io.h>

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
#include "misc.h"
#include "meshing.h"
#include "find_rect.h"


ros::Publisher pub;
ros::Publisher pub_full_moved;
using namespace std;
using namespace sensor_msgs;
using namespace message_filters;



enum Calib_state  {GET_KINECT_TRAFO,COLLECT_PATTERNS,GET_PROJECTION, EVERYTHING_SETUP};

Calib_state prog_state;


cv::Mat proj_Matrix;

int cnt=0;
vector<CvPoint> mask_points;

Cloud full_cloud_moved;
Eigen::Vector4f plane_model;

IplImage* col;

const cv::Size C_checkboard_size = cv::Size(8,6);
IplImage *mask_image;
bool depth_mask_valid;

// trafo cloud s.t. checkerboard is z=0,  middle of board at x=y=0 and parallel to image axis
// the first trafo is stored and used for all following frames
Eigen::Affine3f kinect_trafo;
bool kinect_trafo_valid = false;

// region of the projector image where the checkerboard will be drawn
IplImage* board_mask = NULL;
cv::Mat projector_image;

const CvSize C_proj_size = cvSize(1152,864);
//const CvSize C_proj_size = cvSize(640,480);

vector<cv::Point2f> projector_corners; // position of internal corners on the projector image


// coordinate system in plane
Vector3f pl_center, pl_upwards, pl_right;

Cloud corners_3d;

Mesh_visualizer* mv;
Rect_finder* rect_finder;
#include <iostream>
#include <fstream>

void createQUAD_PCD(){

 ofstream myfile;

 float width = 0.13;
 float length = 0.14;
 float height = 0.065;
 float res = 0.002; // 2 mm

 float z_min = 0.5;

 vector<Eigen::Vector3f> pts;

 // top
 for (float x=0; x<= width; x+=res)
  for (float y=0; y<= length; y+=res){
   pts.push_back(Eigen::Vector3f(x,y,height));
  }

 // front
 for (float x=0; x<= width; x+=res)
  for (float z=height-3*res; z >= 0; z-=res){
   pts.push_back(Eigen::Vector3f(x,0,z));
  }


 // side
 for (float y=3*res; y<= length; y+=res)
  for (float z=height-3*res; z >= 0; z-=res){
   pts.push_back(Eigen::Vector3f(0,y,z));
  }


 // ROS_INFO("creating %i pts", pnt_cnt);

 myfile.open ("out.pcd");
 myfile << "# .PCD v.7 - Point Cloud Data file format\n";
 myfile << "VERSION .7" << endl;
 myfile << "FIELDS x y z _" << endl;
 myfile << "SIZE 4 4 4 1" << endl;
 myfile << "TYPE F F F U" << endl;
 myfile << "COUNT 1 1 1 4" << endl;
 myfile << "WIDTH " << pts.size() <<  endl;
 myfile << "HEIGHT 1" << endl;
 myfile << "VIEWPOINT 0 0 0 1 0 0 0" << endl;
 myfile << "POINTS " << pts.size() << endl;
 myfile << "DATA ascii" << endl;


 for (uint i=0; i<pts.size(); ++i)
  myfile << pts[i].x() << " " << pts[i].y() << " " << (pts[i].z()+z_min) << " 0 0 256 100" << endl;


 myfile.close();

}







// define this to compute the depth-mask from the detected checkerbord
// if not defined, the user can select four points manually to define the mask
#define MASK_FROM_DETECTIONS


void on_mouse_mask( int event, int x, int y, int flags, void* param ){
 vector<CvPoint>* vec = (vector<CvPoint>*)param;
 if (event == CV_EVENT_LBUTTONUP) vec->push_back(cvPoint(x,y));
}



void on_mouse_projector( int event, int x, int y, int flags, void* param ){


 if (proj_Matrix.rows == 0) return;
 if (event != CV_EVENT_LBUTTONUP) return;

 Cloud* cloud = (Cloud*)param;

 // get 3d point at this position
 pcl_Point p = cloud->at(x,y);

 if (p.z != p.z) {ROS_WARN("Point has no depth!"); return;}
 if (abs(p.z) > 0.1) {ROS_WARN("Point is not in plane! (%.0f cm)",abs(100*p.z)); }

 cv::Point2f proj;
 applyPerspectiveTrafo(cv::Point3f(p.x,p.y,p.z),proj_Matrix,proj);

 // ROS_INFO("PX: %i %i", x,y);
 // ROS_INFO("3d: %f %f %f", p.x,p.y,p.z);
 // ROS_INFO("projected: %f %f", proj.x, proj.y);

 cv::circle(projector_image, proj,20, CV_RGB(255,0,0),-1);
 IplImage img_ipl = projector_image;
 cvShowImage("fullscreen_ipl", &img_ipl);

}


void showImageZone(const Cloud& full_moved){



 if (proj_Matrix.rows == 0) return;

// ROS_INFO("show image zone");

 projector_image.setTo(0);


 // load testimage:
 cv::Mat test_img = cv::imread("/usr/gast/engelhan/ros/Touchscreen/imgs/Testbild.png");
 int width_px = test_img.cols;
 int height_px = test_img.rows;

// ROS_INFO("test image: %i %i", width_px, height_px);

 // create rect according to coordinate system and draw it
 float width = 0.4;
 float height = width*height_px/width_px; // use ratio of input-image

 float off_x = 0;
 float off_y = 0;

 vector<Eigen::Vector3f> d3;

 d3.push_back(Eigen::Vector3f(-width,-height,0)+Eigen::Vector3f(off_x, off_y, 0));
 d3.push_back(Eigen::Vector3f( width,-height,0)+Eigen::Vector3f(off_x, off_y, 0));
 d3.push_back(Eigen::Vector3f( width, height,0)+Eigen::Vector3f(off_x, off_y, 0));
 d3.push_back(Eigen::Vector3f(-width, height,0)+Eigen::Vector3f(off_x, off_y, 0));

// vector<cv::Point2f> proj_vec;
// cv::Point2f proj;
// for (uint i=0; i<d3.size(); ++i){
//  applyPerspectiveTrafo(cv::Point3f(d3[i].x(),d3[i].y(),d3[i].z()),proj_Matrix,proj);
//
//  ROS_INFO("rect: %f %f %f", d3[i].x(),d3[i].y(),d3[i].z());
//  //  ROS_INFO("px: %f %f", proj_vec[i].x, proj_vec[i].y);
//
//  proj_vec.push_back(proj);
// }
//
// for (uint i=0; i< proj_vec.size()-1; ++i){
//  cv::line(projector_image,proj_vec[i], proj_vec[(i+1)%proj_vec.size()], CV_RGB(0,255,0), 2);
// }
//
// cv::circle(projector_image, proj_vec[0], 20, CV_RGB(0,0,255),3);

 // TODO
// drawImageinRect(proj_Matrix, d3,projector_image);
 // TODO: image region should have same width/height-ratio as image
 // projectProjectorIntoImage(proj_Matrix, plane_model, d3, projector_image);

 cv::Mat mask = projector_image.clone();
 mask.setTo(0);

 vector<cv::Point> proj_vec;
 vector<cv::Point2f> proj_vecf;
 for (uint i=0; i<d3.size(); ++i){
  cv::Point2f proj;
  applyPerspectiveTrafo(cv::Point3f(d3[i].x(),d3[i].y(),d3[i].z()),proj_Matrix,proj);
  proj_vec.push_back(cv::Point(proj.x, proj.y));
  proj_vecf.push_back(proj);
 }

// cv::fillConvexPoly(mask, &proj_vec[0], proj_vec.size(), cv::Scalar::all(255));
// cv::rectangle(mask, cv::Point(20,20),cv::Point(200,100), CV_RGB(0,255,0), 5);


 vector<cv::Point2f> src;
 src.push_back(cv::Point2f(0,0));
 src.push_back(cv::Point2f(width_px,0));
 src.push_back(cv::Point2f(width_px, height_px));
 src.push_back(cv::Point2f(0, height_px));

 cv::Mat trafo = cv::getPerspectiveTransform(&src[0], &proj_vecf[0]);

 vector<cv::Point2f> src_trafoed;
 cv::perspectiveTransform(src, src_trafoed, trafo);

// for (uint i=0; i<4-1; ++i){
//  ROS_INFO("source: %f %f", src[i].x, src[i].y);
//  cv::line(mask,src[i], src[(i+1)%proj_vec.size()], CV_RGB(0,255,0), 2);
//  ROS_INFO("trafoed: %f %f, goal: %f %f",src_trafoed[i].x,src_trafoed[i].y,proj_vecf[i].x,proj_vecf[i].y);
// }
//
// for (uint i=0; i< proj_vec.size()-1; ++i){
//  cv::line(mask,proj_vec[i], proj_vec[(i+1)%proj_vec.size()], CV_RGB(0,255,0), 2);
// }
//
// cv::circle(mask, src[0], 20, CV_RGB(0,0,255),3);


 // cout << "trafo " << trafo <<  endl;

 cv::Mat rotated = mask.clone();

 cv::warpPerspective(test_img, rotated, trafo, cv::Size(rotated.cols, rotated.rows), cv::INTER_LINEAR, cv::BORDER_CONSTANT);

// cv::namedWindow("warped");
// cv::imshow("warped", test_img);

 IplImage img_ipl = rotated;
 cvShowImage("fullscreen_ipl", &img_ipl);


//
// IplImage img_ipl = projector_image;
// cvShowImage("fullscreen_ipl", &img_ipl);

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

 // cout << "callback" << endl;

 Cloud cloud;
 pcl::fromROSMsg(*cloud_ptr, cloud);

 // cout << cloud_ptr->header.frame_id << endl;


 // Cloud filtered = rect_finder->find_rect(cloud);
 //
 //
 // pcl::io::savePCDFile("scene.pcd", filtered);

 // pcl::PolygonMesh mesh = createMeshFromPointcloud(cloud);
 //
 // mv->visualizeMesh(cloud, mesh);
 //// mv->visualizeMeshLines(filtered, mesh);
 //
 // ROS_INFO("created mesh");



 sensor_msgs::CvBridge bridge;

 col = bridge.imgMsgToCv(img_ptr, "bgr8");

 //	if (depth_mask_valid){
 //		cv::Mat cpy = col;
 //		showMaskOnImage(cpy, mask_image);
 //		cvShowImage("camera", cpy);
 //	}else{
 cvShowImage("camera", col);
 //	}

 short c = cv::waitKey(100);

 if (prog_state != EVERYTHING_SETUP && c == -1) return;

 if (c == 'a')
  prog_state = GET_PROJECTION;


 if (prog_state == COLLECT_PATTERNS)
  cout << "State: COLLECT_PATTERNS" << endl;

 if (prog_state == GET_PROJECTION)
  cout << "State: GET_PROJECTION" << endl;


 CvPoint2D32f corners[C_checkboard_size.width*C_checkboard_size.height];
 int c_cnt=0;
 bool board_found = false;

 if (prog_state == GET_KINECT_TRAFO || prog_state == COLLECT_PATTERNS){

  ROS_INFO("Searching for checkerboard");

  cvFindChessboardCorners(col, C_checkboard_size,corners, &c_cnt,CV_CALIB_CB_ADAPTIVE_THRESH);
  board_found = (c_cnt == C_checkboard_size.width*C_checkboard_size.height);

  if (board_found){ ROS_WARN("Checkerboard was detected");
  }else { ROS_WARN("Board was not found!"); return; }

  IplImage* gray = cvCreateImage(cvGetSize(col),col->depth, 1);
  cvCvtColor(col,gray, CV_BGR2GRAY);
  cvFindCornerSubPix(gray, corners, c_cnt,cvSize(5,5),cvSize(1,1),cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10,0.1));
  //  cvDrawChessboardCorners(col, C_checkboard_size, corners, c_cnt,board_found);
  //  cvShowImage("camera", col);
  //  cv::waitKey(0);
 }



#ifdef MASK_FROM_DETECTIONS
 if (board_found && !depth_mask_valid){
  ROS_INFO("Creating Mask");
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



 if (prog_state == GET_KINECT_TRAFO){

  if (!depth_mask_valid){ ROS_INFO("No depth mask!"); return; }

  // fit plane to pointcloud:
  Cloud filtered;
  applyMask(cloud, filtered,mask_image);


  fitPlaneToCloud(filtered, plane_model);

  // project the detected corners to the plane
  Cloud projected;
  bool valid = projectToPlane(corners, C_checkboard_size, cloud, plane_model, projected); // false if one corner has no depth
  if (!valid) {ROS_WARN("One of the corner points had no depth!"); return; }

  defineAxis(projected, pl_center, pl_upwards, pl_right);

  // TODO:
  // use kinect as definition for horizontal line
  // pl_right = Eigen::Vector3f(1,0,0);

  // cout << "plane: " << plane_model.head<3>() << endl;

  float plane_direction = 1;
  if (plane_model.head<3>()[2] < 0){
   // ROS_ERROR("wrong normal direction!");
   plane_direction  = -1;
  }


  pcl::getTransformationFromTwoUnitVectorsAndOrigin(-pl_upwards,plane_direction*plane_model.head<3>(), pl_center, kinect_trafo);


//  saveMatrix(kinect_trafo,"data/kinect_trafo.txt");
//  ROS_INFO("Wrote kinect_trafo to data/kinect_trafo.txt");


  kinect_trafo_valid = true;
  prog_state = COLLECT_PATTERNS;
 }


 // Cloud transformed_cloud;
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

  // ROS_INFO("Get projection");

  computeProjectionMatrix(proj_Matrix, corners_3d, projector_corners);
  prog_state = COLLECT_PATTERNS;


  cv::Mat H_cv, H_svd;

 computeHomography_OPENCV(projector_corners, corners_3d, H_cv);
 computeHomography_SVD(projector_corners, corners_3d, H_cv);




  // show all corners on image:

  for (uint i=0; i<corners_3d.size(); ++i){
   cv::Point2f proj;
   applyPerspectiveTrafo(cv::Point3f(corners_3d[i].x,corners_3d[i].y,corners_3d[i].z),proj_Matrix,proj);

   // ROS_INFO("PX: %i %i", x,y);
   // ROS_INFO("3d: %f %f %f", corners_3d[i].x,corners_3d[i].y,corners_3d[i].z);
   // ROS_INFO("projected: %f %f", proj.x, proj.y);

   cv::circle(projector_image, proj,10, CV_RGB(255,0,0),2);

  }


//  IplImage img_ipl = projector_image;
//  cvShowImage("fullscreen_ipl", &img_ipl);


  // cout << "write" << endl << proj_Matrix << endl;
//  cv::FileStorage fs("data/projection_matrix.yml", cv::FileStorage::WRITE);
//  assert(fs.isOpened());
//  fs << "ProjectionMatrix" << proj_Matrix;
//  fs.release();
 }

 if (proj_Matrix.cols > 0){
  pcl::getTransformedPointCloud(cloud,kinect_trafo,full_cloud_moved);
  cvSetMouseCallback("camera",on_mouse_projector,&full_cloud_moved);
  showImageZone(full_cloud_moved);
 }



 if (prog_state == EVERYTHING_SETUP){
  pcl::getTransformedPointCloud(cloud,kinect_trafo,full_cloud_moved);

  //		// project cloud into image:
  projectCloudIntoProjector(full_cloud_moved,proj_Matrix, projector_image);
  //  cv::imshow("board", projector_image);
  //  cv::setWindowProperty("board", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

  //				cv::namedWindow("foo");
  //				cv::imshow("foo", projector_image);
 }


 if (kinect_trafo_valid){

  //		cout << "sending cloud" << endl;
  Cloud filtered;

  //  applyMask(cloud, filtered,mask_image);
  pcl::getTransformedPointCloud(cloud,kinect_trafo,filtered);

  // ROS_INFO("sending on projected (%zu points)", filtered.size());

  Cloud::Ptr msg = filtered.makeShared();
  msg->header.frame_id = "/openni_rgb_optical_frame";
  msg->header.stamp = ros::Time::now ();
  pub.publish(msg);



 }

 // everything is set up!


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


}



int main(int argc, char ** argv)
{


 // createQUAD_PCD();
 // return 0;


 ros::init(argc, argv, "subscriber");
 ros::NodeHandle nh;
 cvNamedWindow("camera", 1);
 //	cvNamedWindow("mask",1);

 // cv::namedWindow("board");

 // load projector mask and show fullscreen on secondary screen:
 cv::Mat board_mask = cv::imread("data/proj_mask.png",0);

 if (!board_mask.data){
  board_mask = cvCreateImage(C_proj_size, IPL_DEPTH_8U,1);
  board_mask.setTo(255);
  ROS_INFO("Found no projector mask, using total area");
 }
 if (board_mask.cols !=  C_proj_size.width){
  ROS_ERROR("mask for projector image has not the same size as the projector screen!!");
 }
 // draw board on image and store position of internal corners
 projector_image = cv::Mat(C_proj_size, CV_8UC3);
 drawCheckerboard(&projector_image, &board_mask, C_checkboard_size,projector_corners);


 // using old style for fullscreen image
 cvNamedWindow("fullscreen_ipl",0);
 cvMoveWindow("fullscreen_ipl", 2000, 100);
 cvSetWindowProperty("fullscreen_ipl", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
 IplImage proj_ipl = projector_image;
 cvShowImage("fullscreen_ipl", &proj_ipl);


 pub = nh.advertise<Cloud>("projected", 1);
 pub_full_moved = nh.advertise<Cloud>("full_moved", 1);

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

 prog_state = GET_KINECT_TRAFO;

 // read kinect_trafo
 kinect_trafo_valid = loadMatrix(kinect_trafo,"data/kinect_trafo.txt");

 if (kinect_trafo_valid){
  ROS_INFO("found kinect_trafo");
  for (uint i=0; i<4; ++i){
   for (uint j=0; j<4; ++j)
    cout << kinect_trafo(i,j) << " ";
   cout << endl;
  }
 }



 // look for Projection matrix:
 cv::FileStorage fs2("data/projection_matrix.yml", cv::FileStorage::READ);
 fs2["ProjectionMatrix"] >> proj_Matrix;
 if (proj_Matrix.cols > 0){
  cout << "projection" << endl << proj_Matrix << endl;
 }

 // if the projection matrix was deleted, also reestimate the kinect pose
 if (!kinect_trafo_valid && proj_Matrix.cols > 0){
  ROS_INFO("Found kinect-trafo but no Projection matrix, reestimating kinect pose");
  kinect_trafo_valid = false;
 }

 if (kinect_trafo_valid && proj_Matrix.cols > 0){
  ROS_INFO("Loaded everything from file");
  prog_state = EVERYTHING_SETUP;
 }


 mv = new Mesh_visualizer();
 rect_finder = new Rect_finder();


 cvStartWindowThread();

 typedef sync_policies::ApproximateTime<Image, PointCloud2> policy;
 message_filters::Subscriber<Image> image_sub(nh, "/camera/rgb/image_color", 2);
 message_filters::Subscriber<PointCloud2> cloud_sub(nh, "/camera/rgb/points", 2);
 Synchronizer<policy> sync(policy(2), image_sub, cloud_sub);
 sync.registerCallback(boost::bind(&callback, _1, _2));


 ros::spin();
 cvDestroyWindow("camera");
 return 0;

}

