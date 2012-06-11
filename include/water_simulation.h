/*
 * water_simulation.h
 *
 *  Created on: Jun 9, 2012
 *      Author: Nikolas Engelhard
 */

#ifndef WATER_SIMULATION_H_
#define WATER_SIMULATION_H_

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include "type_definitions.h"
#include "calibration.h"

struct Water_Simulator {

 cv::Mat land_height;
 cv::Mat water_depth;
 cv::Mat dummy;
 cv::Mat mask_;

 Cloud cloud_;


 Water_Simulator(){
  land_height = cv::Mat(480,640,CV_64FC1); // height above zero
  water_depth = cv::Mat(480,640,CV_64FC1); // height above zero
  dummy = cv::Mat(480,640,CV_64FC1); // used for storage of intermediate values

  water_depth.setTo(0);
  land_height.setTo(0);
  dummy.setTo(0);
 }

// void visualizeWater(cv::Mat& img, cv::Mat P);

 void showWaterImages();
 void setWaterHeight(double new_height, int x, int y);

 void updateLandHeight(const Cloud& cloud, const cv::Mat& mask);

 Cloud projectIntoImage(cv::Mat& img, cv::Mat P);

 void flow_step();

 void flow_stepStone();


 void createSimData();

};





#endif /* WATER_SIMULATION_H_ */
