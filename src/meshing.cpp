/*
 * meshing.cpp
 *
 *  Created on: Apr 20, 2012
 *      Author: Nikolas Engelhard
 */

#include "meshing.h"

pcl::PolygonMesh Mesh_visualizer::createMesh(const Cloud& cloud){


 pcl::PolygonMesh mesh;
 if (cloud.size() == 0) return mesh;

 pcl::OrganizedFastMesh<pcl_Point> mesher;

 Cloud clone = cloud;

 mesher.setInputCloud(clone.makeShared());


// std::vector<pcl::Vertices>& polygons

 ROS_INFO("reconstruct started");
 ROS_INFO("%i %i", cloud.height, cloud.width);

 assert(cloud.isOrganized());
 mesher.reconstruct(mesh);
 ROS_INFO("Mesh has %zu triangles",mesh.polygons.size());
}


void Mesh_visualizer::visualizeMeshLines(const Cloud& cloud, const pcl::PolygonMesh& mesh){

 if (pub_.getNumSubscribers() == 0) {ROS_INFO("mesh: no one is listening"); return;}


 visualization_msgs::Marker marker;

 marker.header.frame_id = "/openni_rgb_optical_frame";
 marker.header.stamp = ros::Time::now();

 marker.type = visualization_msgs::Marker::LINE_LIST;
 marker.action = visualization_msgs::Marker::ADD;

 marker.id = 0;

 geometry_msgs::Point p;
 std_msgs::ColorRGBA col;
 col.a = 1.0;
 col.r = 1.0;
 col.g = 0.0;
 col.b = 0.0;

 marker.scale.x = 0.02;

 marker.color = col;

 marker.lifetime = ros::Duration();

 //  ROS_INFO("Mesh has %zu triangles",mesh.polygons.size());
 for (uint i=0; i<mesh.polygons.size(); ++i){
  pcl::Vertices vtc = mesh.polygons[i];
  assert(vtc.vertices.size() == 3);

  for (uint j=0; j<=3; ++j){

   pcl_Point cp = cloud.points[vtc.vertices[j%3]];

   p.x = cp.x;
   p.y = cp.y;
   p.z = cp.z;

   marker.points.push_back(p);
   marker.colors.push_back(col);

  }

 }

 pub_lines_.publish(marker);



}


void Mesh_visualizer::visualizeMesh(const Cloud& cloud, const pcl::PolygonMesh& mesh){

 // if (pub_.getNumSubscribers() == 0) {ROS_INFO("mesh: no one is listening"); return;}


 visualization_msgs::Marker marker;

 marker.header.frame_id = "/openni_rgb_optical_frame";
 marker.header.stamp = ros::Time::now();

 marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
 marker.action = visualization_msgs::Marker::ADD;

 marker.id = 0;

 geometry_msgs::Point p;
 std_msgs::ColorRGBA col;
 col.a = 1.0;
 col.r = 1.0;
 col.g = 1.0;
 col.b = 0.0;

 marker.scale.x = 1;
 marker.scale.y = 1;
 marker.scale.z = 1;

 marker.color = col;

 marker.lifetime = ros::Duration();

 ROS_INFO("Mesh has %zu triangles",mesh.polygons.size());
 for (uint i=0; i<mesh.polygons.size(); ++i){
  pcl::Vertices vtc = mesh.polygons[i];
  assert(vtc.vertices.size() == 3);

  for (uint j=0; j<3; ++j){

   pcl_Point cp = cloud.points[vtc.vertices[j]];

   p.x = cp.x;
   p.y = cp.y;
   p.z = cp.z;

   marker.points.push_back(p);

   int color = *reinterpret_cast<const int*>(&(cp.rgb));
   int r = (0xff0000 & color) >> 16;
   int g = (0x00ff00 & color) >> 8;
   int b =  0x0000ff & color;

   col.r = r/255.0;
   col.g = g/255.0;
   col.b = b/255.0;
   col.a = 1;


   marker.colors.push_back(col);

  }

 }

 pub_.publish(marker);



}


