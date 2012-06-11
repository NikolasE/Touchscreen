/*
 * water_simulation.cpp
 *
 *  Created on: Jun 9, 2012
 *      Author: Nikolas Engelhard
 */

#include "water_simulation.h"
using namespace std;

// uses -z!
void Water_Simulator::updateLandHeight(const Cloud& cloud, const cv::Mat& mask){

 assert(int(cloud.width) == land_height.cols && int(cloud.height) == land_height.rows);

 cloud_ = cloud;
 mask_ = mask;

 for (uint x=0; x<cloud.width; ++x)
  for (uint y=0; y<cloud.height; ++y){
   pcl_Point p = cloud.at(x,y);
   if (!(p.x == p.x)) continue; // don't change depth if none was measures (is intially zero)
   land_height.at<double>(y,x) = -p.z;
  }

}

void Water_Simulator::showWaterImages(){

 cv::namedWindow("landHeight", 1);
 cv::namedWindow("waterDepth", 1);
 // cv::namedWindow("sum", 1);

 //
 double max_landheight = 0.3;
 double max_waterdepth = 0.2;

  double foo = 0;
  for (int x=1; x<water_depth.cols-1; ++x)
   for (int y=1; y<water_depth.rows-1; ++y){
    foo = std::max(foo, water_depth.at<double>(y,x));
   }

//  ROS_INFO("max val: %f", foo);


 cv::Mat land_scaled = land_height/max_landheight;
 cv::Mat water_scaled = water_depth/foo;

 water_scaled.setTo(0);
 for (int x=1; x<water_depth.cols-1; ++x)
  for (int y=1; y<water_depth.rows-1; ++y){
   if( water_depth.at<double>(y,x) > 0){
    water_scaled.at<double>(y,x) = 1;
    //    water_depth.at<double>(y,x) = 1;
   }
  }


 // double val = water_scaled.at<double>(480/2,640/2);
 // ROS_INFO("val: %f", val);

 // cv::Mat sum = (land_height+water_depth)/(max_landheight+max_waterdepth);

 cv::imshow("landHeight", land_scaled);
 cv::imshow("waterDepth", water_scaled);
 //  cv::imshow("sum", sum);

}



void Water_Simulator::flow_stepStone(){

 double c = 1;
 dummy.setTo(0);


 double total_water = 0;

 int step = 1;
 int range = 1;


 for (int x=0; x<water_depth.cols;x+=step)
  for (int y=0; y<water_depth.rows; y+=step){

   if (x==0 || y == 0 || x == water_depth.cols || y == water_depth.rows){
//    water_depth.at<double>(y,x) = 0;
    continue;
   }

   if (mask_.at<uchar>(y,x) == 0) continue;

   double water = water_depth.at<double>(y,x);
   double stone = land_height.at<double>(y,x);


   //ROS_INFO("Water: %f, stone: %f", water, stone);


   if (water <= 0) continue;

   //  cout << water << endl;

   //   cout << "stone " << stone << endl;
   //   if (stone < 0.05) continue;

   total_water+=water;


   //   ROS_INFO("at: %i %i: water: %f stone: %f", x,y,water, stone);


   double max_height = water+stone;
   double mean = 0;
   double weight_sum = 0;
   float weight;


   double dist_sum = 0;

   for (int dx=-range; dx<=range; dx++)
    for (int dy=-range; dy<=range; dy++)
   {
     if (dx == 0 && dy == 0) continue;

     double height = water_depth.at<double>(y+dy,x+dx)+land_height.at<double>(y+dy,x+dx);

     //     ROS_INFO("at %i %i: height: %f, neighbour: %f",x,y,max_height, height);


     //     if (land_height.at<double>(y+dy,x+dx) > stone) continue;

     if (height >= (max_height-0.00)) continue;

     weight = 1;

     dist_sum += (max_height-height);

     mean += weight*height;
     weight_sum += weight;

    }

   if (weight_sum==0) continue;

   mean /= weight_sum;

   assert(mean == mean);

   // wie viel Wasser kann abfliessen?
   double mass = std::min((max_height-mean)*c,water);

   dummy.at<double>(y,x) -= mass;

   //   ROS_INFO("at: %i %i: water: %f stone: %f, mass: %f, weight_sum: %f", x,y,water, stone, mass, weight_sum);

   // Verteilung der Masse auf die Nachbarn
   for (int dx=-range; dx<=range; dx++)
    for (int dy=-range; dy<=range; dy++)
     {
     if (dx == 0 && dy == 0) continue;
     double height = water_depth.at<double>(y+dy,x+dx)+land_height.at<double>(y+dy,x+dx);

     if (height >= (max_height-0.00))  continue;
     //     if (land_height.at<double>(y+dy,x+dx) > stone) continue;


     // wassermenge haengt von relativer Hoehe ab
     double flow = (max_height-height)/dist_sum*mass;

     //     double flow = mass/weight_sum;

     dummy.at<double>(y+dy,x+dx) += flow;
    }

  }

 water_depth += dummy;


// water_depth -= 0.00001;

 // cv::namedWindow("flow");
 // cv::imshow("flow", dummy);
 // cv::waitKey(-1);

 // ROS_INFO("END FLOWSTONE mean Water: %f", total_water/(638*478));


}


void Water_Simulator::createSimData(){

 float max_height = 0.3;

 for (int x=1; x<land_height.cols-1; ++x)
  for (int y=1; y<land_height.rows-1; ++y){
   if (x<land_height.cols/2)
    land_height.at<double>(y,x) =max_height- x*2.0/land_height.cols*max_height;
   else
    land_height.at<double>(y,x) = (x-land_height.cols/2)*2.0/land_height.cols*max_height;

  }


 water_depth.setTo(0);
 setWaterHeight(0.1,640/2,480/2);

 // cv::namedWindow("land");
 // cv::imshow("land", land_height/max_height);
 // cv::waitKey(-1);


}


void Water_Simulator::flow_step(){

 double c = 0.9;
 dummy.setTo(0);

 // double sum = cv::sum(water_depth).val[0];
 //
 // ROS_WARN("Image contains: %f",sum);

 int water_pixels = 0;

 for (int x=1; x<water_depth.cols-1; ++x)
  for (int y=1; y<water_depth.rows-1; ++y){

   double height = water_depth.at<double>(y,x);

   double mean = 0;
   double weight_sum = 0;
   float weight;

   for (int dx=-1; dx<=1; ++dx)
    for (int dy=-1; dy<=1; ++dy){
     if (dx == 0 && dy == 0) continue;

     if (dx != 0 && dy !=0){
      weight = 1/sqrt(2);
     }else
      weight = 1;

     mean+= weight*water_depth.at<double>(y+dy,x+dx);
     weight_sum += weight;
    }

   mean /= weight_sum;



   //   double h_1 = water_depth.at<double>(y-1,x);
   //   double h_2 = water_depth.at<double>(y+1,x);
   //   double h_3 = water_depth.at<double>(y,x-1);
   //   double h_4 = water_depth.at<double>(y,x+1);
   //
   //   double mean = (height + h_1 + h_2 + h_3 + h_4)/5;

   double diff = height-mean;


   if (height>0){
    water_pixels++;
    //     ROS_INFO("Height: %f, mean: %f, pos: %i %i, new: %f", height, mean, x,y,height-c*diff);
   }


   dummy.at<double>(y,x) = c*diff;
  }

 ROS_INFO("Waterpixelcnt: %i", water_pixels);

 water_depth -= dummy;

}


Cloud Water_Simulator::projectIntoImage(cv::Mat& img, cv::Mat P){

 Cloud c;

 img.setTo(0);

 for (uint x=0; x<cloud_.width; ++x){
  for (uint y=0; y<cloud_.height; ++y){

   double z = water_depth.at<double>(y,x);

   if (!(z>0)) continue;

   pcl_Point p = cloud_.at(x,y);

   cv::Point2f px;
   applyPerspectiveTrafo(cv::Point3f(p.x,p.y,p.z),P,px);

   int x = px.x; int y = px.y;
   if (x<0 || y<0 || x>=img.cols || y >= img.rows) continue;

   int h = int((z/0.02)*30)%30;
   // ROS_INFO("waterdd z: %f", z);
 //  ROS_INFO("z: %f, col: %i", z,h);

   cv::circle(img, px, 3, cv::Scalar(h,255,255) ,-1);
   //    cv::circle(img, px, 3, cv::Scalar(90,255,255) ,-1);

   // add to cloud:

   p.z -= z;
   c.push_back(p);
  }
 }

 cv::cvtColor(img, img, CV_HSV2BGR);

 // cv::namedWindow("foo");
 // cv::imshow("foo", img);
 // cv::waitKey(10);

 return c;

}



void Water_Simulator::setWaterHeight(double new_height, int x, int y){
 assert(new_height>0);

 cv::circle(water_depth, cv::Point(x,y),5, cv::Scalar::all(new_height),-1);

 //   water_depth.at<double>(y,x) = new_height;
}



