/*
 * projector_calibrator.cpp
 *
 *  Created on: May 9, 2012
 *      Author: engelhan
 */


using namespace std;

#include "projector_calibrator.h"


bool Projector_Calibrator::loadMat(const string name, const string filename, cv::Mat& mat){
 char fn[100]; sprintf(fn,"data/%s.yml", filename.c_str());
 ROS_INFO("Reading %s from %s", name.c_str(),fn);

 cv::FileStorage fs(fn, cv::FileStorage::READ);
 if (!fs.isOpened()){
  ROS_WARN("Could not read %s", fn);
  return false;
 }

 fs[filename] >> mat; fs.release();
 fs.release();
 return true;
}


bool Projector_Calibrator::saveMat(const string name, const string filename, const cv::Mat& mat){
 char fn[100]; sprintf(fn,"data/%s.yml", filename.c_str());
 ROS_INFO("Saving %s to %s", name.c_str(),fn);
 cv::FileStorage fs(fn, cv::FileStorage::WRITE);
 if (!fs.isOpened()){
  ROS_WARN("Could not write to %s", fn);
  return false;
 }

 fs << filename << mat;
 fs.release();
 return true;
}


void Projector_Calibrator::initFromFile(){

 mask = cv::imread("data/kinect_mask.png",0);
 if (mask.data){
  ROS_INFO("Found mask (%i %i)", mask.cols, mask.rows);
 }else {
  ROS_INFO("Could not find mask at data/kinect_mask.png");
 }


 // Check for Kinect trafo:
 char fn[100]; sprintf(fn, "data/%s.txt",kinect_trafo_filename.c_str());
 kinect_trafo_valid = loadAffineTrafo(kinect_trafo,fn);
 if (kinect_trafo_valid)  ROS_INFO("Found kinect trafo");
 else  ROS_WARN("Could not load Kinect Trafo from %s", fn);


 // load Matrices
 loadMat("Projection Matrix", proj_matrix_filename, proj_Matrix);
 loadMat("Homography (OpenCV)", hom_cv_filename, hom_CV);
 loadMat("Homography (SVD)", hom_svd_filename, hom_SVD);
}


void Projector_Calibrator::drawCheckerboard(cv::Mat& img, const cv::Size size, vector<cv::Point2f>& corners_2d){

 corners_2d.clear();

 // draw white border with this size
 // "Note: the function requires some white space (like a square-thick border,
 // the wider the better) around the board to make the detection more robust in various environment"
 float border = 40;

 float width = (img.cols-2*border)/(size.width+1);
 float height = (img.rows-2*border)/(size.height+1);

 img.setTo(255); // all white

 float minx = border;
 float miny = border;

 // ROS_INFO("GRID: W: %f, H: %f", width, height);

 // start with black square
 for (int j = 0; j<=size.height; j++)
  for (int i = (j%2); i<size.width+1; i+=2){

   cv::Point2f lu = cv::Point2f(minx+i*width,miny+j*height);
   cv::Point2f rl = cv::Point2f(minx+(i+1)*width,miny+(j+1)*height);
   cv::rectangle(img, lu, rl ,cv::Scalar::all(0), -1);

   cv::Point2f ru = cv::Point2f(rl.x,lu.y);

   if (j==0) continue;
   if (i>0){
    corners_2d.push_back(cv::Point2f(lu.x, lu.y));
    //        cvCircle(img, cvPoint(lu.x, lu.y),20, CV_RGB(255,0,0),3);
   }
   if (i<size.width){
    corners_2d.push_back(cv::Point2f(ru.x, ru.y));
    //        cvCircle(img, cvPoint(ru.x, ru.y),20, CV_RGB(255,0,0),3);
   }
  }

 assert(int(corners_2d.size()) == size.width*size.height);

}




// set wall_region to same ratio as image
bool Projector_Calibrator::setupImageProjection(float width_m, float off_x_m, float off_y_m, const cv::Size& img_size){
 return setupImageProjection(width_m, width_m/img_size.width*img_size.height, off_x_m, off_y_m, img_size);
}




bool Projector_Calibrator::setupImageProjection(const cv_RectF& wall_area, const cv::Size& img_size){

 if (wall_area.width == 0){
  ROS_ERROR("setupImageProjection: wall area has width of 0!");
  return false;
 }
 return setupImageProjection(wall_area.width, wall_area.height, wall_area.x, wall_area.y, img_size);
}

bool Projector_Calibrator::setupImageProjection(float width_m, float height_m, float off_x_m, float off_y_m, const cv::Size& img_size){


 if(!projMatorHomSet()){
  ROS_WARN("setupImageProjection: Neither Projection Matrix nor Homography computed!");
  return false;
 }

 int width_px  = img_size.width;
 int height_px = img_size.height;

 float height_m_2 = width_m/width_px*height_px;

 if (fabs(height_m_2-height_m) > 0.1){
  ROS_WARN("setupImageProjection: Image and wall-section have different ratios!");
 }


 if (projMatrixSet()){

  cv::Mat px_to_world(cv::Size(3,4), CV_64FC1);
  px_to_world.setTo(0);

//  ROS_INFO("Computing warp from Projection Matrix!");

  px_to_world.at<double>(3,2) = 1;
  px_to_world.at<double>(0,0) = width_m/width_px;
  px_to_world.at<double>(0,2) = off_x_m;
  px_to_world.at<double>(1,1) = height_m/height_px; // == width/width_px
  px_to_world.at<double>(1,2) = off_y_m;

  warp_matrix = proj_Matrix*px_to_world;
  warp_matrix /= warp_matrix.at<double>(2,2); // defined up to scale

//  cout << "Warp_matrix: " << endl << warp_matrix << endl;

  return true;
 }

 // Compute from Homography:
 cv::Mat px_to_world(cv::Size(3,3), CV_64FC1);
 px_to_world.setTo(0);

// ROS_INFO("Computing warp from Homography!");

 px_to_world.at<double>(2,2) = 1;
 px_to_world.at<double>(0,0) = width_m/width_px;
 px_to_world.at<double>(0,2) = off_x_m;
 px_to_world.at<double>(1,1) = height_m/height_px; // == width/width_px
 px_to_world.at<double>(1,2) = off_y_m;

 if (homOpenCVSet())
  warp_matrix = hom_CV*px_to_world;
 else
  warp_matrix = hom_SVD*px_to_world;

 warp_matrix /= warp_matrix.at<double>(2,2); // defined up to scale

// cout << "Warp_matrix" << endl << warp_matrix << endl;


 return true;

}

void Projector_Calibrator::showUnWarpedImage(const cv::Mat& img){

 if (!warpMatrixSet()){
  ROS_INFO("showUnWarpedImage: call setupImageProjection first.."); return;
 }

 // clear projector image
 projector_image.setTo(0);

 cv::Size size(projector_image.cols, projector_image.rows);
 cv::warpPerspective(img, projector_image, warp_matrix, size, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
 IplImage img_ipl = projector_image;
 cvShowImage("fullscreen_ipl", &img_ipl);

}


void Projector_Calibrator::computeHomography_OPENCV(){

 ROS_WARN("COMPUTING HOMOGRAPHY WITH openCV");

 uint N = projector_corners.size();
 if(observations_3d.size() < N){
  ROS_ERROR("computeHomography_OPENCV: less 3d-points than 2d-points, aborting");
  return;
 }
 // count number of 3d points which are more than 2cm away from the z=0-plane
 float z_max = 0.03; int cnt = 0;
 for (uint i=0; i<N; ++i) { if (abs(observations_3d.at(i).z) > z_max) cnt++; }
 if (cnt>N*0.01) {  ROS_WARN("found %i of %i points with dist > 3cm", cnt,N); }


 // copy the 3d observations (x,y,~0) into 2d (x,y)
 vector<cv::Point2f> src; src.reserve(N);
 for (uint i=0; i<N; ++i){src.push_back(cv::Point2f(observations_3d.at(i).x,observations_3d.at(i).y));}

 // 2d = H*3d H*(x,y,1)

#define DO_SCALING

#ifdef DO_SCALING
 cv::Mat T,U;
 vector<cv::Point2f> src_trafoed, d2_trafoed;
 scalePixels(src,  T, src_trafoed);
 scalePixels(projector_corners,  U, d2_trafoed);
 hom_CV = cv::findHomography(src_trafoed,d2_trafoed);
 hom_CV = U.inv()*hom_CV*T;
#else
 hom_CV = cv::findHomography(src,projector_corners);
#endif


 // cout << "Homography with OpenCV: " << endl << hom_CV << endl;

 // compute error:
 float error = 0;

 cv::Point2f px;
 float err_x = 0;
 float err_y = 0;

 float e_x, e_y;

 for (uint i=0; i<N; ++i){
  applyHomography(cv::Point2f(observations_3d.at(i).x,observations_3d.at(i).y),hom_CV, px);

  e_x = abs((px.x-projector_corners.at(i).x));
  e_y = abs((px.y-projector_corners.at(i).y));

  err_x += e_x/N; err_y += e_y/N  ;
  error += sqrt(e_x*e_x+e_y*e_y)/N;

  //  ROS_INFO("Proj: %f %f, goal: %f %f (Error: %f)", px.x,px.y,corners_2d.at(i).x, corners_2d.at(i).y,err);
 }


 ROS_INFO("mean error: %f (x: %f, y: %f)", error, err_x, err_y);

 saveMat("Homography(OpenCV)", hom_cv_filename, hom_CV);

}


void Projector_Calibrator::computeHomography_SVD(){

 ROS_WARN("COMPUTING HOMOGRAPHY WITH SVD");

 uint N = projector_corners.size();
 if(observations_3d.size() < N){
  ROS_ERROR("computeHomography_SVD: less 3d-points than 2d-points, aborting");
  return;
 }

 // count number of 3d points which are more than 2cm away from the z=0-plane
 float z_max = 0.03; int cnt = 0;
 for (uint i=0; i<N; ++i) { if (abs(observations_3d.at(i).z) > z_max) cnt++; }
 if (cnt>N*0.01) { ROS_WARN("found %i of %i points with dist > 3cm", cnt,N); }

 // create Matrices
 // cv::Mat src(2,N,CV_32FC1);
 // cv::Mat dst(2,N,CV_32FC1);

#define SCALE_SVD


#ifdef SCALE_SVD
 cv::Mat T;
 vector<cv::Point2f> d2_trafoed;
 scalePixels(projector_corners,  T, d2_trafoed);
#else
 vector<cv::Point2f> d2_trafoed = projector_corners;
#endif



 // Pointcloud to 2d-points
 vector<cv::Point2f> src, src_trafoed;

 for (uint i=0; i<N; ++i) src.push_back(cv::Point2f(observations_3d.at(i).x,observations_3d.at(i).y));

#ifdef SCALE_SVD
 cv::Mat U;
 scalePixels(src, U,src_trafoed);
#else
 src_trafoed = src;
#endif


 cv::Mat A = cv::Mat(2*N,9,CV_64FC1);
 A.setTo(0);

 // p_ cross H*p = 0
 for (uint i=0; i<N; ++i){
  cv::Point2f P = src_trafoed.at(i);
  cv::Point2f p = d2_trafoed.at(i);

  // ROS_INFO("P: %f %f,  p: %f %f", P.x, P.y, p.x, p.y);

  float f[9] = {0,0,0,-P.x,-P.y,1,p.y*P.x,p.y*P.y,p.y};
  for (uint j=0; j<9; ++j) A.at<double>(2*i,j) = f[j];

  float g[9] = {P.x,P.y,1,0,0,0,-p.x*P.x,-p.x*P.y,-p.x};
  for (uint j=0; j<9; ++j) A.at<double>(2*i+1,j) = g[j];
 }
 // now solve A*h == 0
 // Solution is the singular vector with smallest singular value

 cv::Mat h = cv::Mat(9,1,CV_64FC1);
 cv::SVD::solveZ(A,h);


 // h only fixed up to scale -> set h(3,3) = 1;
 h /= h.at<double>(8);

 // cout << "h: " << h << endl;

 hom_SVD = cv::Mat(3,3,CV_64FC1);

 for (uint i=0; i<3; ++i){
  for (uint j=0; j<3; ++j)
   hom_SVD.at<double>(i,j) =  h.at<double>(3*i+j);
 }

 // cout << "Tinv " << T.inv() << endl;
 // cout << "U " << U << endl;

 // undo scaling
#ifdef SCALE_SVD
 hom_SVD = T.inv()*hom_SVD*U;
#endif

 // 2d = H*3d H*(x,y,1)


//  cout << "Homography with SVD: " << endl << hom_SVD << endl;


 // compute error:
 float error = 0;
 float err_x = 0;
 float err_y = 0;

 float a,b;


 cv::Point2f px;
 for (uint i=0; i<N; ++i){
  applyHomography(src.at(i), hom_SVD, px);

  a = abs(px.x-projector_corners.at(i).x);
  b = abs(px.y-projector_corners.at(i).y);

  err_x += a/N; err_y += b/N;

  error += sqrt(a*a+b*b)/N;

  //ROS_INFO("src: %f %f, Proj: %f %f, goal: %f %f (Error: %f)", src.at(i).x, src.at(i).y, px.x,px.y,corners_2d.at(i).x, corners_2d.at(i).y,err);
 }

 ROS_INFO("mean error: %f (x: %f, y: %f)", error, err_x, err_y);

 saveMat("Homography(SVD)", hom_svd_filename, hom_SVD);
}


void Projector_Calibrator::computeProjectionMatrix(){


 uint c_cnt = observations_3d.points.size();
 uint proj_cnt = projector_corners.size();

 if (c_cnt == 0){
  ROS_WARN("Can't compute projection matrix without 3d points");
 }

 assert(c_cnt % proj_cnt == 0);

 int img_cnt = c_cnt/proj_cnt;

 ROS_WARN("Computing Projection matrix from %i images", img_cnt);


 Cloud trafoed_corners;
 vector<cv::Point2f> trafoed_px;
 cv::Mat U,T;

 scaleCloud(observations_3d, U, trafoed_corners);
 scalePixels(projector_corners, T, trafoed_px);

 cv::Mat A = cv::Mat(2*c_cnt,12,CV_64FC1);
 A.setTo(0);

 //ROS_ERROR("Projection:");

 // p_ cross H*p = 0
 for (uint i=0; i<c_cnt; ++i){
  pcl_Point   P = trafoed_corners.points.at(i);
  cv::Point2f p = trafoed_px.at(i%proj_cnt);

  // ROS_INFO("from %f %f %f to %f %f", P.x,P.y,P.z,p.x,p.y);

  float f[12] = {0,0,0,0,-P.x,-P.y,-P.z,1,p.y*P.x,p.y*P.y,p.y*P.z,p.y};
  for (uint j=0; j<12; ++j) A.at<double>(2*i,j) = f[j];

  float g[12] = {P.x,P.y,P.z,1,0,0,0,0,-p.x*P.x,-p.x*P.y,-p.x*P.z,-p.x};
  for (uint j=0; j<12; ++j) A.at<double>(2*i+1,j) = g[j];
 }

 // now solve A*h == 0
 // Solution is the singular vector with smallest singular value

 cv::Mat h = cv::Mat(12,1,CV_64FC1);
 cv::SVD::solveZ(A,h);

 proj_Matrix = cv::Mat(3,4,CV_64FC1);

 for (uint i=0; i<3; ++i){
  for (uint j=0; j<4; ++j)
   proj_Matrix.at<double>(i,j) =  h.at<double>(4*i+j);
 }

 // cout << "Tinv " << T.inv() << endl;
 // cout << "U " << U << endl;

 // undo scaling
 proj_Matrix = T.inv()*proj_Matrix*U;


 // cout << "Projection Matrix: " << endl <<  P << endl;


 // compute reprojection error:
 double total = 0;
 double total_x = 0;
 double total_y = 0;

 cv::Point2f px;



 for (uint i=0; i<c_cnt; ++i){
  //    ROS_INFO("projection %i", i);

  pcl_Point   p = observations_3d.points.at(i);
  cv::Point2f p_ = projector_corners.at(i%proj_cnt); //

  applyPerspectiveTrafo(cv::Point3f(p.x,p.y,p.z),proj_Matrix,px);


  // ROS_INFO("err: %f %f", (px.x-p_.x),(px.y-p_.y));

  total_x += abs(px.x-p_.x)/c_cnt;
  total_y += abs(px.y-p_.y)/c_cnt;
  total += sqrt(pow(px.x-p_.x,2)+pow(px.y-p_.y,2))/c_cnt;

 }

 ROS_INFO("Projection Matrix: mean error: %f (x: %f, y: %f)", total, total_x, total_y);


 saveMat("Projection Matrix", proj_matrix_filename, proj_Matrix);
}


bool Projector_Calibrator::findCheckerboardCorners(){
 corners.clear();
 if (input_image.rows == 0){  ROS_WARN("can't find corners on empty image!"); return false;  }

 // cv::namedWindow("search",1);
 // cv::imshow("search", input_image);
 // cv::waitKey(-1);

 if (!cv::findChessboardCorners(input_image, C_checkboard_size,corners, CV_CALIB_CB_ADAPTIVE_THRESH)) {
  ROS_WARN("Could not find a checkerboard!");
  return false;
 }


 cv::cvtColor(input_image, gray, CV_BGR2GRAY);
 cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
 return true;
}


bool Projector_Calibrator::storeCurrent3DObservations(){


 if (!kinect_trafo_valid){
  ROS_WARN("can't store Observations in global Frame since it is not defined!");
  return false;
 }

 if (corners.size() == 0){
  ROS_WARN("Can't add Observations since Corners havn't been detected");
  return false;
 }

 if (input_cloud.size() == 0){
  ROS_WARN("storeCurrent3DObservations: input_cloud.size() == 0");
  return false;
 }


 Cloud c_3d;
 for (uint i=0; i<corners.size(); ++i){
  pcl_Point p = input_cloud.at(corners[i].x, corners[i].y);
  if (!(p.x == p.x)){ROS_WARN("storeCurrent3DObservations: Found Corner without depth!"); return false;}
  c_3d.points.push_back(p);
  // ROS_INFO("new 3d point: %f %f %f", p.x, p.y,p.z);
 }
 // transform from kinect-frame to wall-frame

 // pcl_Point  p = c_3d.points[0];
 // ROS_INFO("before: c_3d(0): %f %f %f", p.x,p.y,p.z);

 pcl::getTransformedPointCloud(c_3d, kinect_trafo, c_3d);

 // p = c_3d.points[0];
 // ROS_INFO("after: c_3d(0): %f %f %f", p.x,p.y,p.z);

 observations_3d.points.insert(observations_3d.points.end(),c_3d.points.begin(), c_3d.points.end());
 ROS_INFO("Added %zu points, now %zu 3D-Observations",c_3d.size(), observations_3d.size());

 // p = observations_3d.points[0];
 // ROS_INFO("Observations_3d(0): %f %f %f", p.x,p.y,p.z);

 return true;
}



void Projector_Calibrator::computeKinectTransformation(){

 if (!kinect_orientation_valid){
  ROS_INFO("Can't compute KinectTrafo without Kinect's orientation angle"); return;
 }

 ROS_INFO("Computing Kinect Trafo");

 Cloud filtered;
 applyMaskOnInputCloud(filtered);

 Eigen::Vector4f plane_model;
 fitPlaneToCloud(filtered, plane_model);

 int m = (C_checkboard_size.height/2*C_checkboard_size.width)+(C_checkboard_size.width-1)/2;

 pcl_Point p  = input_cloud.at(corners[m].x, corners[m].y);
 pcl_Point p2 = input_cloud.at(corners[m].x+sin(-kinect_tilt_angle_deg/180*M_PI)*100, corners[m].y-cos(-kinect_tilt_angle_deg/180*M_PI)*100);

 if ( p2.x != p2.x){
  ROS_WARN("NAN in pointcloud, no calculation of new wall-system");
  return;
 }

 Eigen::Vector3f pl_center = Eigen::Vector3f(p.x,p.y,p.z);
 Eigen::Vector3f pl_upwards = Eigen::Vector3f(p2.x-p.x,p2.y-p.y,p2.z-p.z);

 // float plane_direction = 1;
 // if (plane_model.head<3>()[2] < 0){ plane_direction  = -1; }

 float plane_direction = plane_model.head<3>()[2]>0?1:-1;

 pcl::getTransformationFromTwoUnitVectorsAndOrigin(-pl_upwards,plane_direction*plane_model.head<3>(), pl_center, kinect_trafo);


 // save to file
 char fn[100]; sprintf(fn, "data/%s.txt",kinect_trafo_filename.c_str());
 saveAffineTrafo(kinect_trafo,fn);
 ROS_INFO("Wrote kinect_trafo to %s", fn);


 pcl::getTransformedPointCloud(input_cloud,kinect_trafo,cloud_moved);

 kinect_trafo_valid = true;

}



void Projector_Calibrator::getCheckerboardArea(vector<cv::Point2i>& pts){

 pts.clear();
 if (corners.size() == 0){
  ROS_WARN("getCheckerboardArea: no corners!"); return;
 }


 float l = (corners[1].x-corners[0].x);

 cv::Point2i p = corners[0];
 pts.push_back(cv::Point2i(p.x-l,p.y-l));

 p = corners[C_checkboard_size.width-1];
 pts.push_back(cv::Point2i(p.x+l,p.y-l));

 p = corners[C_checkboard_size.height*C_checkboard_size.width-1];
 pts.push_back(cv::Point2i(p.x+l,p.y+l));

 p = corners[C_checkboard_size.width*(C_checkboard_size.height-1)];
 pts.push_back(cv::Point2i(p.x-l,p.y+l));
}



bool Projector_Calibrator::findOptimalProjectionArea(float ratio, cv_RectF& rect){

 if(!kinect_trafo_valid){
  ROS_WARN("findOptimalProjectionArea: no kinect trafo!"); return false;
 }

 if (corners.size() == 0){
  ROS_WARN("findOptimalProjectionArea: no corners!"); return false;
 }

 if (cloud_moved.size() == 0){
  ROS_WARN("findOptimalProjectionArea: no input cloud!"); return false;
 }


 vector<cv::Point2i> c;
 getCheckerboardArea(c);
// cv::fillConvexPoly(mask,c,CV_RGB(255,255,255));


 // get 3d coordinates of corners in the wall-system
 vector<cv::Point2f> rotated;
 float min_x = 100;
 float min_y = 100;
 for (uint i=0; i<c.size(); ++i){
  pcl_Point p = cloud_moved.at(c[i].x,c[i].y);
  rotated.push_back(cv::Point2f(p.x,p.y));
  min_x = min(min_x, p.x);
  min_y = min(min_y, p.y);
  // ROS_INFO("pre: %f %f", rotated[i].x, rotated[i].y);
 }


 vector<cv::Point2i> pt_i;
 int max_x, max_y;
 max_x = max_y = -1;
 // ROS_INFO("min: %f %f", min_x, min_y);
 for (uint i=0; i<c.size(); ++i){
  rotated[i] = cv::Point2f((rotated[i].x-min_x)*100,(rotated[i].y-min_y)*100); // in cm <=> 1px
  pt_i.push_back(cv::Point2i(rotated[i].x,rotated[i].y));
  max_x = max(max_x, pt_i[i].x);
  max_y = max(max_y, pt_i[i].y);
  // ROS_INFO("post: %f %f", rotated[i].x, rotated[i].y);
 }

 cv::Mat search_img(max_y,max_x,CV_8UC1); search_img.setTo(0);
 cv::fillConvexPoly(search_img,pt_i,CV_RGB(255,255,255));


//#define SHOW_SEARCH_IMAGE

#ifdef SHOW_SEARCH_IMAGE
  cv::namedWindow("search_img",1);
  cv::imshow("search_img", search_img);
  cv::waitKey(10);
#endif

 // find largest rect in white area:

 // ratio = width/height
 bool finished = false;

 float step = 0.03; // check every X m

 float width, height; int x, y;
 for (width = max_x; width > 0 && !finished; width -= step){ // check every 5 cm
  height = width/ratio;

  // ROS_INFO("Width: %f, height: %f", width, height);

  // find fitting left upper corner (sliding window)
  for (x = 0; x < max_x-width && !finished; x+= step*100){
   for (y = 0; y < max_y-height; y+=step*100){
    // ROS_INFO("Checking x = %i, y = %i", x, y);

    int x_w = x+width; int y_w = y+height;
    assert(x >= 0 && y >= 0 && x_w < search_img.cols && y< search_img.rows);
    // check if all corners are withing white area:
    if (search_img.at<uchar>(y,x) == 0) continue;
    if (search_img.at<uchar>(y,x_w) == 0) continue;
    if (search_img.at<uchar>(y_w,x_w) == 0) continue;
    if (search_img.at<uchar>(y_w,x) == 0) continue;
    // ROS_INFO("Found fitting pose (w,h: %f %f)", width, height);
#ifdef SHOW_SEARCH_IMAGE
    cv::rectangle(search_img, cv::Point(x,y), cv::Point(x_w, y_w), CV_RGB(125,125,125));
#endif

    finished = true; // break outer loops
    break;
   } // for y
  } // for x
 } // for width

#ifdef SHOW_SEARCH_IMAGE
   cv::imshow("search_img", search_img);
   cv::waitKey(10);
#endif

 if (!finished) return false;

 // restore pose in wall_frame
 rect.width = width/100;
 rect.height = height/100;

 rect.x = x/100.0+min_x;
 rect.y = y/100.0+min_y;

 // show area on input image:

//  ROS_INFO("Optimal rect: x,y: %f %f, w,h: %f %f", rect.x, rect.y, rect.width, rect.height);

 return true;

}




void Projector_Calibrator::createMaskFromDetections(){

 if (corners.size() != uint(C_checkboard_size.width*C_checkboard_size.height)){
  ROS_INFO("can't create mask if the corners were not detected!"); return; }

 mask = cv::Mat(cv::Size(640,480), CV_8UC1);  mask.setTo(0);

 vector<cv::Point2i> c;
 getCheckerboardArea(c);

 //  for (uint i=0; i<c.size(); ++i){
 //   cv::circle(mask, c[i] ,10,CV_RGB(255,255,255));
 //   ROS_INFO("%i %i", c[i].x, c[i].y);
 //  }

 cv::fillConvexPoly(mask,c,CV_RGB(255,255,255));


 ROS_INFO("Writing kinect_mask to data/kinect_mask.png");
 cv::imwrite("data/kinect_mask.png", mask);
 //
 //
 //   cv::namedWindow("Mask on Kinect Image");
 //   cv::imshow("Mask on Kinect Image", mask);
 //   cv::waitKey(-1);
}



float Projector_Calibrator::fitPlaneToCloud(const Cloud& cloud, Eigen::Vector4f& model){
 ROS_INFO("Fitting plane to cloud with %zu points", cloud.size());

 // http://pointclouds.org/documentation/tutorials/random_sample_consensus.php#random-sample-consensus
 pcl::SampleConsensusModelPlane<pcl_Point>::Ptr
 model_p (new pcl::SampleConsensusModelPlane<pcl_Point> (cloud.makeShared()));

 pcl::RandomSampleConsensus<pcl_Point> ransac(model_p);
 ransac.setDistanceThreshold(0.005); // max dist of 5mm
 ransac.computeModel();

 Eigen::VectorXf c;
 ransac.getModelCoefficients(c);
 model = c;

 std::vector<int> inliers;
 ransac.getInliers(inliers);
 float inlier_pct = inliers.size()*100.0/cloud.size();

 if (inlier_pct<0.5){ ROS_WARN("Only %.3f %%  inlier in fitPlaneToCloud!", inlier_pct); }
 return inlier_pct;
}


void Projector_Calibrator::applyMaskOnInputCloud(Cloud& out){

 assert(mask_valid() && int(input_cloud.width) == mask.cols);

 for (int x=0; x<mask.cols; ++x)
  for (int y=0; y<mask.rows; ++y){
   if (mask.at<uchar>(y,x) > 0){
    pcl_Point p = input_cloud.at(x,y);
    if (p.x == p.x) out.points.push_back(p);
   }
  }
}


void Projector_Calibrator::showFullscreenCheckerboard(){

 drawCheckerboard(projector_image, C_checkboard_size, projector_corners);
 IplImage proj_ipl = projector_image;
 cvShowImage("fullscreen_ipl", &proj_ipl);
}

