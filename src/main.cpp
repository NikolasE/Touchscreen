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
#include "user_input.h"
#include "projector_calibrator.h"


ros::Publisher pub;
ros::Publisher pub_full_moved;
using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
namespace enc = sensor_msgs::image_encodings;

enum Calib_state  {GET_KINECT_TRAFO,COLLECT_PATTERNS,GET_PROJECTION, EVERYTHING_SETUP};

Calib_state prog_state;


Projector_Calibrator calibrator;

vector<float> measured_angles;


float kinect_tilt_angle_deg = 0;
bool kinect_tilt_angle_valid = false;

ros::Subscriber sub_imu;


#include <iostream>
#include <fstream>


void on_mouse_mask( int event, int x, int y, int flags, void* param ){
 vector<CvPoint>* vec = (vector<CvPoint>*)param;
 if (event == CV_EVENT_LBUTTONUP) vec->push_back(cvPoint(x,y));
}


void on_mouse_projector( int event, int x, int y, int flags, void* param ){
 //
 //
 // if (proj_Matrix.rows == 0) return;
 // if (event != CV_EVENT_LBUTTONUP) return;
 //
 // Cloud* cloud = (Cloud*)param;
 //
 // // get 3d point at this position
 // pcl_Point p = cloud->at(x,y);
 //
 // if (p.z != p.z) {ROS_WARN("Point has no depth!"); return;}
 // if (abs(p.z) > 0.1) {ROS_WARN("Point is not in plane! (%.0f cm)",abs(100*p.z)); }
 //
 // cv::Point2f proj;
 // applyPerspectiveTrafo(cv::Point3f(p.x,p.y,p.z),proj_Matrix,proj);
 //
 // // ROS_INFO("PX: %i %i", x,y);
 // // ROS_INFO("3d: %f %f %f", p.x,p.y,p.z);
 // // ROS_INFO("projected: %f %f", proj.x, proj.y);
 //
 // cv::circle(projector_image, proj,20, CV_RGB(255,0,0),-1);
 // IplImage img_ipl = projector_image;
 // cvShowImage("fullscreen_ipl", &img_ipl);
 //
}



void callback(const ImageConstPtr& img_ptr, const sensor_msgs::PointCloud2ConstPtr& cloud_ptr){

 // ROS_INFO("callback");

 // Kinect's orientation is needed to compute the transformation
 if (!calibrator.isKinectTrafoSet() && !calibrator.isKinectOrientationSet()){
  ROS_INFO("waiting for tilt angle!");
  return;
 }


 Cloud cloud;
 pcl::fromROSMsg(*cloud_ptr, cloud);


 cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_ptr , enc::BGR8);


 calibrator.setInputImage(cv_ptr->image);

 cv::imshow("camera", cv_ptr->image);

 short c = cv::waitKey(100);

 if (prog_state != EVERYTHING_SETUP && c == -1) return;

 if (c == 'a')
  prog_state = GET_PROJECTION;


 if (prog_state == COLLECT_PATTERNS)
  cout << "State: COLLECT_PATTERNS" << endl;

 if (prog_state == GET_PROJECTION)
  cout << "State: GET_PROJECTION" << endl;


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


  calibrator.setInputCloud(cloud);
  calibrator.computeKinectTransformation();


  prog_state = COLLECT_PATTERNS;
 }



 // Cloud transformed_cloud;
 if (prog_state == COLLECT_PATTERNS){

  calibrator.storeCurrent3DObservations();

  //if (!calibrator.homOpenCVSet())
   calibrator.computeHomography_SVD();

  //if (!calibrator.homSVDSet())
   calibrator.computeHomography_OPENCV();

 }


 if (prog_state == GET_PROJECTION){

  // ROS_INFO("Get projection");



  calibrator.computeProjectionMatrix();


  prog_state = COLLECT_PATTERNS;

 }

 if (calibrator.projMatrixSet()){

  //   cvSetMouseCallback("camera",on_mouse_projector,&full_cloud_moved);

  if (!calibrator.warpMatrixSet()){

   float width = 0.8;
   float height = width/calibrator.getTestImg()->cols*calibrator.getTestImg()->rows; // use ratio of input-image

   float off_x = -width/2;
   float off_y = -height/2;

   calibrator.setupImageProjection(width,height,  off_x, off_y, calibrator.getTestImg()->size());
  }

  if (calibrator.warpMatrixSet()){
   calibrator.showUnWarpedImage(*calibrator.getTestImg());

  }
 }

 if (calibrator.isKinectTrafoSet()){

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

 //
 // calibrator.initFromFile();
 //
 // return 0;

 ros::NodeHandle nh;
 cv::namedWindow("camera", 1);


 pub = nh.advertise<Cloud>("projected", 1);
 pub_full_moved = nh.advertise<Cloud>("full_moved", 1);

 prog_state = GET_KINECT_TRAFO;

 sub_imu = nh.subscribe("/imu", 10, imu_CB);

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

