/*
 * main.cpp
 *
 *  Created on: Feb 26, 2012
 *      Author: Nikolas Engelhard
 */



#include <ros/ros.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>

#include "type_definitions.h"
#include "projector_calibrator.h"
#include "user_input.h"
#include "meshing.h"

#include <stdlib.h>

ros::Publisher pub;
ros::Publisher pub_full_moved;
ros::Publisher pub_3dPoints;
ros::Publisher pub_colored;

Mesh_visualizer* mesher;


using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
namespace enc = sensor_msgs::image_encodings;

enum Calib_state  {GET_KINECT_TRAFO,COLLECT_PATTERNS,GET_PROJECTION, EVERYTHING_SETUP};

Calib_state prog_state;


Projector_Calibrator calibrator;

vector<float> measured_angles;

User_Input user_input;


float kinect_tilt_angle_deg = 0;
bool kinect_tilt_angle_valid = false;


// uncomment to show testImage, otherwise content of Main screen is shown on projector
#define SHOW_TEST_IMAGE

#ifndef SHOW_TEST_IMAGE
cv::Size mainScreenSize(1920,1200);
#endif

cv_RectF optimalRect;
ros::Subscriber sub_imu;


#include <iostream>
#include <fstream>


void on_mouse_mask( int event, int x, int y, int flags, void* param ){
 vector<CvPoint>* vec = (vector<CvPoint>*)param;
 if (event == CV_EVENT_LBUTTONUP) vec->push_back(cvPoint(x,y));
}


void on_mouse_projector( int event, int x, int y, int flags, void* param ){



 if (event != CV_EVENT_LBUTTONUP) return;

 if (!calibrator.projMatrixSet()) return;

 Cloud* cloud = (Cloud*)param;

 // get 3d point at this position
 pcl_Point p = cloud->at(x,y);

 if (p.z != p.z) {ROS_WARN("Point has no depth!"); return;}
 // if (abs(p.z) > 0.1) {ROS_WARN("Point is not in plane! (%.0f cm)",abs(100*p.z)); }

 cv::Point2f proj;
 applyPerspectiveTrafo(cv::Point3f(p.x,p.y,p.z),calibrator.proj_Matrix,proj);

 // ROS_INFO("PX: %i %i", x,y);
 // ROS_INFO("3d: %f %f %f", p.x,p.y,p.z);
 // ROS_INFO("projected: %f %f", proj.x, proj.y);

 cv::circle(calibrator.projector_image, proj,20, CV_RGB(255,0,0),-1);
 IplImage img_ipl = calibrator.projector_image;
 cvShowImage("fullscreen_ipl", &img_ipl);

}


void imgCBHeightLines(const ImageConstPtr& img_ptr, const sensor_msgs::PointCloud2ConstPtr& cloud_ptr){
 Cloud cloud;
 pcl::fromROSMsg(*cloud_ptr, cloud);

 ROS_INFO("Mesh start");
 pcl::PolygonMesh mesh = mesher->createMesh(cloud);

 mesher->visualizeMesh(cloud, mesh);

 std::vector<Line_collection> height_lines;

 float height_step = 0.2; // given in meters

 mesher->createHeightLines(mesh,cloud, height_lines, height_step);

 mesher->visualizeHeightLines(height_lines);


}

void imgCB(const ImageConstPtr& img_ptr, const sensor_msgs::PointCloud2ConstPtr& cloud_ptr){
 // cout << "img cb" << endl;

 Cloud cloud;
 pcl::fromROSMsg(*cloud_ptr, cloud);
 calibrator.setInputCloud(cloud);

 cvSetMouseCallback("camera",on_mouse_projector,&calibrator.cloud_moved);


 cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_ptr , enc::BGR8);
 calibrator.setInputImage(cv_ptr->image);

 cv::imshow("camera", cv_ptr->image);
 // cv::waitKey(10);

 // Kinect's orientation is needed to compute the transformation
 if (!calibrator.isKinectTrafoSet() && !calibrator.isKinectOrientationSet()){
  ROS_INFO("waiting for tilt angle!");
  return;
 }


 // find user input and draw on Testimage
 if (calibrator.isKinectOrientationSet()){

  // find hand of user
  user_input.setCloud(calibrator.cloud_moved);

  // project finger into image
  if (user_input.finger_found){
   pcl_Point p = user_input.fingertip;

   cv::Point2f proj;
   if (calibrator.projMatrixSet())
    applyPerspectiveTrafo(cv::Point3f(p.x,p.y,p.z),calibrator.proj_Matrix,proj);

   if (calibrator.homOpenCVSet())
    applyHomography(cv::Point2f(p.x,p.y), calibrator.hom_CV, proj);

   cv::circle(calibrator.projector_image, proj,20, CV_RGB(255,0,0),-1);
   IplImage img_ipl = calibrator.projector_image;
   cvShowImage("fullscreen_ipl", &img_ipl);
  }

 }




 short c = cv::waitKey(100);

 if (prog_state != EVERYTHING_SETUP && c == -1) return;

 if (c == 'a')
  prog_state = GET_PROJECTION;


 // if (prog_state == COLLECT_PATTERNS)
 //  cout << "State: COLLECT_PATTERNS" << endl;
 //
 // if (prog_state == GET_PROJECTION)
 //  cout << "State: GET_PROJECTION" << endl;


 // look for corners
 if (prog_state == GET_KINECT_TRAFO || prog_state == COLLECT_PATTERNS){

  calibrator.showFullscreenCheckerboard();

  if (!calibrator.findCheckerboardCorners()) {
   ROS_WARN("Corners were not detected!");
   return;
  }

 }

 // create mask from detections to mark part of pointcloud that belongs to the wall-plane
 if (!calibrator.mask_valid()){  calibrator.createMaskFromDetections(); }





 if (prog_state == GET_KINECT_TRAFO){


  if (!calibrator.mask_valid()){ ROS_INFO("No depth mask!"); return; }

  calibrator.computeKinectTransformation();

  // store area of checkerboard in the user-interface
  calibrator.getCheckerboardArea(user_input.checkerboard_area);


  calibrator.setInputCloud(cloud);// apply trafo on input cloud

  prog_state = COLLECT_PATTERNS;
 }



 //  calibrator.setInputCloud(cloud);
 if (optimalRect.width == 0 && calibrator.isKinectTrafoSet()){
  ROS_INFO("Computing optimal area!");

#ifdef SHOW_TEST_IMAGE

  assert(calibrator.test_img.rows>0);

  if (!calibrator.findOptimalProjectionArea(calibrator.test_img.cols*1.0/calibrator.test_img.rows, optimalRect)){
#else
   if (!calibrator.findOptimalProjectionArea(mainScreenSize.width*1.0/mainScreenSize.height, optimalRect)){
#endif
    ROS_ERROR("Could not find optimal Projection Area!");
   }
  }


  // Cloud transformed_cloud;
  if (prog_state == COLLECT_PATTERNS){




   calibrator.storeCurrent3DObservations();

   Cloud::Ptr msg = calibrator.observations_3d.makeShared();
   msg->header.frame_id = "/openni_rgb_optical_frame";
   msg->header.stamp = ros::Time::now ();
   pub_3dPoints.publish(msg);

   if (!calibrator.homSVDSet())
    calibrator.computeHomography_SVD();

   if (!calibrator.homOpenCVSet())
    calibrator.computeHomography_OPENCV();

  }


  if (prog_state == GET_PROJECTION){

   // ROS_INFO("Get projection");



   calibrator.computeProjectionMatrix();


   prog_state = COLLECT_PATTERNS;

  }







  if (calibrator.projMatrixSet()){


   if (!calibrator.warpMatrixSet()){

#ifdef SHOW_TEST_IMAGE
    //   float width = 0.8;
    //   float height = width/calibrator.getTestImg()->cols*calibrator.getTestImg()->rows; // use ratio of input-image
    //   float off_x = -width/2;
    //   float off_y = -height/2;
    //   calibrator.setupImageProjection(width,height,  off_x, off_y, calibrator.getTestImg()->size());
    //#else
    calibrator.setupImageProjection(optimalRect, cv::Size(calibrator.test_img.cols, calibrator.test_img.rows));

#else
    calibrator.setupImageProjection(optimalRect, mainScreenSize );
#endif

   }

   //#ifdef SHOW_TEST_IMAGE
   //  if (calibrator.warpMatrixSet()){
   //   calibrator.showUnWarpedImage(*calibrator.getTestImg());
   //  }
   //#endif

  }

  if (calibrator.isKinectTrafoSet()){

   ROS_INFO("publish");

   //		// project cloud into image:
   //   projectCloudIntoProjector(full_cloud_moved,proj_Matrix, projector_image);

   Cloud::Ptr msg = calibrator.getTransformedCloud()->makeShared();
   msg->header.frame_id = "/openni_rgb_optical_frame";
   msg->header.stamp = ros::Time::now ();
   pub_full_moved.publish(msg);

  }



 }

 void imu_CB(const sensor_msgs::ImuConstPtr& imu_ptr){

  if (!calibrator.isKinectOrientationSet()) {



   kinect_tilt_angle_deg = asin(imu_ptr->linear_acceleration.x/9.81)/M_PI*180;
   measured_angles.push_back(kinect_tilt_angle_deg);

   if (measured_angles.size() == 200){
    kinect_tilt_angle_deg = 0;

    for (uint i=0; i<measured_angles.size(); ++i) kinect_tilt_angle_deg += measured_angles[i]/measured_angles.size();

    ROS_INFO("Set kinect orientation to %.1f deg", kinect_tilt_angle_deg);
    kinect_tilt_angle_valid = true;
    calibrator.setKinectOrientation(kinect_tilt_angle_deg);
   }
  }
 }



 int main(int argc, char ** argv)
 {



  ros::init(argc, argv, "subscriber");

  calibrator.initFromFile();

  mesher = new Mesh_visualizer();

  user_input.init();
  user_input.calibrator = &calibrator;

  ros::NodeHandle nh;
  cv::namedWindow("camera", 1);




  pub = nh.advertise<Cloud>("projected", 1);
  pub_full_moved = nh.advertise<Cloud>("full_moved", 1);
  pub_3dPoints = nh.advertise<Cloud>("points_3d", 1);
  pub_colored = nh.advertise<Cloud>("colored", 1);
  prog_state = GET_KINECT_TRAFO;


  sub_imu = nh.subscribe("/imu", 10, imu_CB);

  cvStartWindowThread();

  typedef sync_policies::ApproximateTime<Image, PointCloud2> policy;
  message_filters::Subscriber<Image> image_sub(nh, "/camera/rgb/image_color", 2);
  message_filters::Subscriber<PointCloud2> cloud_sub(nh, "/camera/rgb/points", 2);
  Synchronizer<policy> sync(policy(2), image_sub, cloud_sub);
  //  sync.registerCallback(boost::bind(&imgCB, _1, _2));

  sync.registerCallback(boost::bind(&imgCBHeightLines, _1, _2));


  optimalRect.width = 0;

  ros::Rate r(10);



  while (ros::ok()){
   ros::spinOnce();
   r.sleep();






   //   if (calibrator.projMatrixSet()){
   //    Cloud c = calibrator.visualizePointCloud();
   //    Cloud::Ptr msg = c.makeShared();
   //    msg->header.frame_id = "/openni_rgb_optical_frame";
   //    msg->header.stamp = ros::Time::now ();
   //    pub_colored.publish(msg);
   //   }





   if (calibrator.imageProjectionSet()){
#ifdef SHOW_TEST_IMAGE
    calibrator.showUnWarpedImage(calibrator.test_img);
#else
    system("xwd -root | convert - /tmp/screenshot.jpg");
    cv::Mat screen = cv::imread("/tmp/screenshot.jpg");
    cv::Mat primary_screen = screen(cv::Range(0,mainScreenSize.height), cv::Range(0,mainScreenSize.width));
    calibrator.showUnWarpedImage(primary_screen);

#endif
   }
  }



  // ros::spin();
  cv::destroyAllWindows();
  return 0;

 }

