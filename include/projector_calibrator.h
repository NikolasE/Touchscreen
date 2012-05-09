/*
 * projector_calibrator.h
 *
 *  Created on: May 9, 2012
 *      Author: Nikolas Engelhard
 */

#ifndef PROJECTOR_CALIBRATOR_H_
#define PROJECTOR_CALIBRATOR_H_

#include "cloud_processing.h"
#include "calibration.h"
#include "user_input.h"
#include "misc.h"
#include "meshing.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>


class Projector_Calibrator {


 cv::Size C_checkboard_size;

 cv::Size C_proj_size;
 cv::Mat proj_Matrix, hom_CV, hom_SVD;

 // trafo cloud s.t. checkerboard is z=0,  middle of board at x=y=0 and parallel to image axis
 // the first trafo is stored and used for all following frames
 Eigen::Affine3f kinect_trafo;
 bool kinect_trafo_valid;

 cv::Mat input_image;
 cv::Mat gray;

 cv::Mat mask;


 Cloud input_cloud;

 float kinect_tilt_angle_deg;
 bool kinect_orientation_valid;

 vector<cv::Point2f> corners;
 void applyMaskOnInputCloud(Cloud& out);
 float fitPlaneToCloud(const Cloud& cloud, Eigen::Vector4f& model);

 cv::Mat proj_matrix;

 Cloud observations_3d;

public:


 bool storeCurrent3DObservations();

 bool isKinectTrafoValid(){return kinect_trafo_valid;}
 void setKinectOrientation(float angle_deg){kinect_tilt_angle_deg = angle_deg;kinect_orientation_valid = true;}

 bool mask_valid() {return mask.cols > 0;}

 bool findCheckerboardCorners();
 void setMaskFromDetections();

 void setInputImage(cv::Mat& image){input_image = image; corners.clear();}
 void setInputCloud(Cloud& cloud){input_cloud = cloud;}


 void computeKinectTransformation();

 // pixel coordinates of checkerboard corners
 void computeProjectionMatrix(const vector<cv::Point2f>& projector_corners);
 void computeHomography_OPENCV(const vector<cv::Point2f>& projector_corners);
 void computeHomography_SVD(const vector<cv::Point2f>& projector_corners);



 Projector_Calibrator(){
  kinect_orientation_valid = false;
  kinect_trafo_valid = false;
  C_checkboard_size = cv::Size(8,6);
  C_proj_size = cv::Size(1152,864);
 }







};




#endif /* PROJECTOR_CALIBRATOR_H_ */
