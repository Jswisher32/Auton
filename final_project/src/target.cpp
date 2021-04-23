#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Path.h>
#include <ugv_course_libs/gps_conv.h>
#include <math.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <vector>
double v=.3;
int i;
//gps variables
ros::Publisher marker_pub;
ros::Publisher path_pub;
tf::StampedTransform transform;
tf::Vector3 relative_position;
tf::Vector3 marker1_loc;
tf::Vector3 marker2_loc;
tf::Vector3 marker3_loc;
tf::Vector3 marker4_loc;
tf::Vector3 marker5_loc;
tf::Vector3 marker6_loc;
tf::Vector3 marker7_loc;
tf::Vector3 marker8_loc;
tf::Vector3 marker9_loc;
tf::Vector3 marker10_loc;
tf::Vector3 marker11_loc;
tf::Vector3 marker12_loc;
tf::Vector3 marker13_loc;
tf::Vector3 marker14_loc;
tf::Vector3 marker15_loc;
tf::Vector3 marker16_loc;
tf::Vector3 UTMvector_x;
tf::Vector3 UTMvector_y;
tf::Vector3 CARTvector_x;
tf::Vector3 CARTvector_y;
tf::Vector3 target_checkpoint;
tf::Vector3 target_checkpointUTM;
UTMCoords ref_coords;
UTMCoords current_coordinates;
UTMCoords marker1_l;
UTMCoords marker2_l;
UTMCoords marker3_l;
UTMCoords marker4_l;
UTMCoords marker5_l;
UTMCoords marker6_l;
UTMCoords marker7_l;
UTMCoords marker8_l;
UTMCoords marker9_l;
UTMCoords marker10_l;
UTMCoords marker11_l;
UTMCoords marker12_l;
UTMCoords marker13_l;
UTMCoords marker14_l;
UTMCoords marker15_l;
UTMCoords marker16_l;
visualization_msgs::MarkerArray marker_array_msg;
nav_msgs::Path gps_path;
double heading;
double heading_rad;
double theta;
double heading_error;
double heading_correction;
double convergence_angle;
double central_meridian;
geometry_msgs::Twist vehicle_heading;
ros::Publisher vel_pub;
ros::Publisher steer_pub;
ros::Publisher heading_pub;
ros::Publisher brake_pub;
ros::Publisher throttle_pub;





void timerCallback(const ros::TimerEvent& event){

  geometry_msgs::PoseStamped current_pose;
  current_pose.pose.position.x = relative_position.x();
  current_pose.pose.position.y = relative_position.y();
  
  gps_path.poses.push_back(current_pose);
  gps_path.header.frame_id="world";
  gps_path.header.stamp = event.current_real;
  path_pub.publish(gps_path);
  
  marker_array_msg.markers.resize(16);
  visualization_msgs::Marker marker1;
  visualization_msgs::Marker marker2;
  visualization_msgs::Marker marker3;
  visualization_msgs::Marker marker4;
  visualization_msgs::Marker marker5;
  visualization_msgs::Marker marker6;
  visualization_msgs::Marker marker7;
  visualization_msgs::Marker marker8;
  visualization_msgs::Marker marker9;
  visualization_msgs::Marker marker10;
  visualization_msgs::Marker marker11;
  visualization_msgs::Marker marker12;
  visualization_msgs::Marker marker13;
  visualization_msgs::Marker marker14;
  visualization_msgs::Marker marker15;
  visualization_msgs::Marker marker16;


  
  marker1.header.frame_id = "world";
  marker1.header.stamp = event.current_real;
  marker1.type = visualization_msgs::Marker::CUBE;
  marker1.action = visualization_msgs::Marker::ADD;

  LatLon marker1_coords(42.002436,-83.002683,0);
  marker1_l = UTMCoords(marker1_coords);
  marker1_loc = marker1_l - ref_coords;

  marker1.scale.x = 20;
  marker1.scale.y = 20;
  marker1.scale.z = 2;

  marker1.color.r = 1.0;
  marker1.color.g = 0.0;
  marker1.color.b = 0.0;
  marker1.color.a = 0.75;

  marker1.pose.position.x = marker1_loc.x();
  marker1.pose.position.y = marker1_loc.y();
  marker1.pose.position.z = -1;
  marker1.pose.orientation.x = 0.0;
  marker1.pose.orientation.y = 0.0;
  marker1.pose.orientation.z = 0.0;
  marker1.pose.orientation.w = 1.0;
  marker1.id = 0;

  marker_array_msg.markers[0] = marker1;
 
  marker2.header.frame_id = "world";
  marker2.header.stamp = event.current_real;
  marker2.type = visualization_msgs::Marker::CUBE;
  marker2.action = visualization_msgs::Marker::ADD;

  LatLon marker2_coords(42.002445,-83.001918,0);
  marker2_l = UTMCoords(marker2_coords);
  marker2_loc = marker2_l - ref_coords;

  marker2.scale.x = 20.0;
  marker2.scale.y = 20.0;
  marker2.scale.z = 2.0;

  marker2.color.r = 0.0;
  marker2.color.g = 1.0;
  marker2.color.b = 0.0;
  marker2.color.a = 0.75;

  marker2.pose.position.x = marker2_loc.x();
  marker2.pose.position.y = marker2_loc.y();
  marker2.pose.position.z = -1;
  marker2.pose.orientation.x = 0.0;
  marker2.pose.orientation.y = 0.0;
  marker2.pose.orientation.z = 0.0;
  marker2.pose.orientation.w = 1.0;
  marker2.id = 1;

  marker_array_msg.markers[1] = marker2;
  
  marker3.header.frame_id = "world";
  marker3.header.stamp = event.current_real;
  marker3.type = visualization_msgs::Marker::CUBE;
  marker3.action = visualization_msgs::Marker::ADD;

  LatLon marker3_coords(42.002676,-83.000548,0);
  marker3_l = UTMCoords(marker3_coords);
  marker3_loc = marker3_l - ref_coords;

  marker3.scale.x = 20.0;
  marker3.scale.y = 20.0;
  marker3.scale.z = 2.0;

  marker3.color.r = 0.0;
  marker3.color.g = 1.0;
  marker3.color.b = 0.0;
  marker3.color.a = 0.75;

  marker3.pose.position.x = marker3_loc.x();
  marker3.pose.position.y = marker3_loc.y();
  marker3.pose.position.z = -1;
  marker3.pose.orientation.x = 0.0;
  marker3.pose.orientation.y = 0.0;
  marker3.pose.orientation.z = 0.0;
  marker3.pose.orientation.w = 1.0;
  marker3.id = 2;

  marker_array_msg.markers[2] = marker3;
  
  marker4.header.frame_id = "world";
  marker4.header.stamp = event.current_real;
  marker4.type = visualization_msgs::Marker::CUBE;
  marker4.action = visualization_msgs::Marker::ADD;

  LatLon marker4_coords(42.000342,-82.997880,0);
  marker4_l = UTMCoords(marker4_coords);
  marker4_loc = marker4_l - ref_coords;

  marker4.scale.x = 20.0;
  marker4.scale.y = 20.0;
  marker4.scale.z = 2.0;

  marker4.color.r = 0.0;
  marker4.color.g = 1.0;
  marker4.color.b = 0.0;
  marker4.color.a = 0.75;

  marker4.pose.position.x = marker4_loc.x();
  marker4.pose.position.y = marker4_loc.y();
  marker4.pose.position.z = -1;
  marker4.pose.orientation.x = 0.0;
  marker4.pose.orientation.y = 0.0;
  marker4.pose.orientation.z = 0.0;
  marker4.pose.orientation.w = 1.0;
  marker4.id = 3;

  marker_array_msg.markers[3] = marker4;
 
  marker5.header.frame_id = "world";
  marker5.header.stamp = event.current_real;
  marker5.type = visualization_msgs::Marker::CUBE;
  marker5.action = visualization_msgs::Marker::ADD;

  LatLon marker5_coords(42.001373,-82.997912,0);
  marker5_l = UTMCoords(marker5_coords);
  marker5_loc = marker5_l - ref_coords;

  marker5.scale.x = 20.0;
  marker5.scale.y = 20.0;
  marker5.scale.z = 2.0;

  marker5.color.r = 0.0;
  marker5.color.g = 1.0;
  marker5.color.b = 0.0;
  marker5.color.a = 0.75;

  marker5.pose.position.x = marker5_loc.x();
  marker5.pose.position.y = marker5_loc.y();
  marker5.pose.position.z = -1;
  marker5.pose.orientation.x = 0.0;
  marker5.pose.orientation.y = 0.0;
  marker5.pose.orientation.z = 0.0;
  marker5.pose.orientation.w = 1.0;
  marker5.id = 4;

  marker_array_msg.markers[4] = marker5;

  marker6.header.frame_id = "world";
  marker6.header.stamp = event.current_real;
  marker6.type = visualization_msgs::Marker::CUBE;
  marker6.action = visualization_msgs::Marker::ADD;

  LatLon marker6_coords(42.002666,-82.997679,0);
  marker6_l = UTMCoords(marker6_coords);
  marker6_loc = marker6_l - ref_coords;

  marker6.scale.x = 20.0;
  marker6.scale.y = 20.0;
  marker6.scale.z = 2.0;

  marker6.color.r = 0.0;
  marker6.color.g = 1.0;
  marker6.color.b = 0.0;
  marker6.color.a = 0.75;

  marker6.pose.position.x = marker6_loc.x();
  marker6.pose.position.y = marker6_loc.y();
  marker6.pose.position.z = -1;
  marker6.pose.orientation.x = 0.0;
  marker6.pose.orientation.y = 0.0;
  marker6.pose.orientation.z = 0.0;
  marker6.pose.orientation.w = 1.0;
  marker6.id = 5;

  marker_array_msg.markers[5] = marker6;
 
  marker7.header.frame_id = "world";
  marker7.header.stamp = event.current_real;
  marker7.type = visualization_msgs::Marker::CUBE;
  marker7.action = visualization_msgs::Marker::ADD;

  LatLon marker7_coords(42.005284,-82.998273,0);
  marker7_l = UTMCoords(marker7_coords);
  marker7_loc = marker7_l - ref_coords;

  marker7.scale.x = 20.0;
  marker7.scale.y = 20.0;
  marker7.scale.z = 2.0;

  marker7.color.r = 255.0;
  marker7.color.g = 192.0;
  marker7.color.b = 203.0;
  marker7.color.a = 0.75;

  marker7.pose.position.x = marker7_loc.x();
  marker7.pose.position.y = marker7_loc.y();
  marker7.pose.position.z = -1;
  marker7.pose.orientation.x = 0.0;
  marker7.pose.orientation.y = 0.0;
  marker7.pose.orientation.z = 0.0;
  marker7.pose.orientation.w = 1.0;
  marker7.id = 6;

  marker_array_msg.markers[6] = marker7;
 
  marker8.header.frame_id = "world";
  marker8.header.stamp = event.current_real;
  marker8.type = visualization_msgs::Marker::CUBE;
  marker8.action = visualization_msgs::Marker::ADD;

  LatLon marker8_coords(42.000353,-82.997093,0);
  marker8_l = UTMCoords(marker8_coords);
  marker8_loc = marker8_l - ref_coords;

  marker8.scale.x = 20.0;
  marker8.scale.y = 20.0;
  marker8.scale.z = 2.0;

  marker8.color.r = 0.0;
  marker8.color.g = 1.0;
  marker8.color.b = 1.0;
  marker8.color.a = 0.75;

  marker8.pose.position.x = marker8_loc.x();
  marker8.pose.position.y = marker8_loc.y();
  marker8.pose.position.z = -1;
  marker8.pose.orientation.x = 0.0;
  marker8.pose.orientation.y = 0.0;
  marker8.pose.orientation.z = 0.0;
  marker8.pose.orientation.w = 1.0;
  marker8.id = 7;

  marker_array_msg.markers[7] = marker8; 

  marker9.header.frame_id = "world";
  marker9.header.stamp = event.current_real;
  marker9.type = visualization_msgs::Marker::CUBE;
  marker9.action = visualization_msgs::Marker::ADD;

  LatLon marker9_coords(42.000937,-82.997109,0);
  marker9_l = UTMCoords(marker9_coords);
  marker9_loc = marker9_l - ref_coords;

  marker9.scale.x = 20.0;
  marker9.scale.y = 20.0;
  marker9.scale.z = 2.0;

  marker9.color.r = 1.0;
  marker9.color.g = 0.0;
  marker9.color.b = 0.0;
  marker9.color.a = 0.75;

  marker9.pose.position.x = marker9_loc.x();
  marker9.pose.position.y = marker9_loc.y();
  marker9.pose.position.z = -1;
  marker9.pose.orientation.x = 0.0;
  marker9.pose.orientation.y = 0.0;
  marker9.pose.orientation.z = 0.0;
  marker9.pose.orientation.w = 1.0;
  marker9.id = 8;

  marker_array_msg.markers[8] = marker9;
 
  marker10.header.frame_id = "world";
  marker10.header.stamp = event.current_real;
  marker10.type = visualization_msgs::Marker::CUBE;
  marker10.action = visualization_msgs::Marker::ADD;

  LatLon marker10_coords(42.002749,-82.996989,0);
  marker10_l = UTMCoords(marker10_coords);
  marker10_loc = marker10_l - ref_coords;

  marker10.scale.x = 20.0;
  marker10.scale.y = 20.0;
  marker10.scale.z = 2.0;

  marker10.color.r = 0.0;
  marker10.color.g = 1.0;
  marker10.color.b = 0.0;
  marker10.color.a = 0.75;

  marker10.pose.position.x = marker10_loc.x();
  marker10.pose.position.y = marker10_loc.y();
  marker10.pose.position.z = -1;
  marker10.pose.orientation.x = 0.0;
  marker10.pose.orientation.y = 0.0;
  marker10.pose.orientation.z = 0.0;
  marker10.pose.orientation.w = 1.0;
  marker10.id = 9;

  marker_array_msg.markers[9] = marker10;
  
  marker11.header.frame_id = "world";
  marker11.header.stamp = event.current_real;
  marker11.type = visualization_msgs::Marker::CUBE;
  marker11.action = visualization_msgs::Marker::ADD;

  LatLon marker11_coords(42.001399,-82.996523,0);
  marker11_l = UTMCoords(marker11_coords);
  marker11_loc = marker11_l - ref_coords;

  marker11.scale.x = 20.0;
  marker11.scale.y = 20.0;
  marker11.scale.z = 2.0;

  marker11.color.r = 0.0;
  marker11.color.g = 1.0;
  marker11.color.b = 0.0;
  marker11.color.a = 0.75;

  marker11.pose.position.x = marker11_loc.x();
  marker11.pose.position.y = marker11_loc.y();
  marker11.pose.position.z = -1;
  marker11.pose.orientation.x = 0.0;
  marker11.pose.orientation.y = 0.0;
  marker11.pose.orientation.z = 0.0;
  marker11.pose.orientation.w = 1.0;
  marker11.id = 10;

  marker_array_msg.markers[10] = marker11;
  
  marker12.header.frame_id = "world";
  marker12.header.stamp = event.current_real;
  marker12.type = visualization_msgs::Marker::CUBE;
  marker12.action = visualization_msgs::Marker::ADD;

  LatLon marker12_coords(41.999797,-82.995688,0);
  marker12_l = UTMCoords(marker12_coords);
  marker12_loc = marker12_l - ref_coords;

  marker12.scale.x = 20.0;
  marker12.scale.y = 20.0;
  marker12.scale.z = 2.0;

  marker12.color.r = 0.0;
  marker12.color.g = 1.0;
  marker12.color.b = 0.0;
  marker12.color.a = 0.75;

  marker12.pose.position.x = marker12_loc.x();
  marker12.pose.position.y = marker12_loc.y();
  marker12.pose.position.z = -1;
  marker12.pose.orientation.x = 0.0;
  marker12.pose.orientation.y = 0.0;
  marker12.pose.orientation.z = 0.0;
  marker12.pose.orientation.w = 1.0;
  marker12.id = 11;

  marker_array_msg.markers[11] = marker12;
 
  marker13.header.frame_id = "world";
  marker13.header.stamp = event.current_real;
  marker13.type = visualization_msgs::Marker::CUBE;
  marker13.action = visualization_msgs::Marker::ADD;

  LatLon marker13_coords(42.000376,-82.995707,0);
  marker13_l = UTMCoords(marker13_coords);
  marker13_loc = marker13_l - ref_coords;

  marker13.scale.x = 20.0;
  marker13.scale.y = 20.0;
  marker13.scale.z = 2.0;

  marker13.color.r = 0.0;
  marker13.color.g = 1.0;
  marker13.color.b = 0.0;
  marker13.color.a = 0.75;

  marker13.pose.position.x = marker13_loc.x();
  marker13.pose.position.y = marker13_loc.y();
  marker13.pose.position.z = -1;
  marker13.pose.orientation.x = 0.0;
  marker13.pose.orientation.y = 0.0;
  marker13.pose.orientation.z = 0.0;
  marker13.pose.orientation.w = 1.0;
  marker13.id = 12;

  marker_array_msg.markers[12] = marker13;

  marker14.header.frame_id = "world";
  marker14.header.stamp = event.current_real;
  marker14.type = visualization_msgs::Marker::CUBE;
  marker14.action = visualization_msgs::Marker::ADD;

  LatLon marker14_coords(42.001147,-82.995131,0);
  marker14_l = UTMCoords(marker14_coords);
  marker14_loc = marker14_l - ref_coords;

  marker14.scale.x = 20.0;
  marker14.scale.y = 20.0;
  marker14.scale.z = 2.0;

  marker14.color.r = 0.0;
  marker14.color.g = 1.0;
  marker14.color.b = 0.0;
  marker14.color.a = 0.75;

  marker14.pose.position.x = marker14_loc.x();
  marker14.pose.position.y = marker14_loc.y();
  marker14.pose.position.z = -1;
  marker14.pose.orientation.x = 0.0;
  marker14.pose.orientation.y = 0.0;
  marker14.pose.orientation.z = 0.0;
  marker14.pose.orientation.w = 1.0;
  marker14.id = 13;

  marker_array_msg.markers[13] = marker14;
 
  marker15.header.frame_id = "world";
  marker15.header.stamp = event.current_real;
  marker15.type = visualization_msgs::Marker::CUBE;
  marker15.action = visualization_msgs::Marker::ADD;

  LatLon marker15_coords(42.001408,-82.995735,0);
  marker15_l = UTMCoords(marker15_coords);
  marker15_loc = marker15_l - ref_coords;

  marker15.scale.x = 20.0;
  marker15.scale.y = 20.0;
  marker15.scale.z = 2.0;

  marker15.color.r = 255.0;
  marker15.color.g = 192.0;
  marker15.color.b = 203.0;
  marker15.color.a = 0.75;

  marker15.pose.position.x = marker15_loc.x();
  marker15.pose.position.y = marker15_loc.y();
  marker15.pose.position.z = -1;
  marker15.pose.orientation.x = 0.0;
  marker15.pose.orientation.y = 0.0;
  marker15.pose.orientation.z = 0.0;
  marker15.pose.orientation.w = 1.0;
  marker15.id = 14;

  marker_array_msg.markers[14] = marker15;
 
  marker16.header.frame_id = "world";
  marker16.header.stamp = event.current_real;
  marker16.type = visualization_msgs::Marker::CUBE;
  marker16.action = visualization_msgs::Marker::ADD;

  LatLon marker16_coords(42.001639,-82.993159,0);
  marker16_l = UTMCoords(marker16_coords);
  marker16_loc = marker16_l - ref_coords;

  marker16.scale.x = 20.0;
  marker16.scale.y = 20.0;
  marker16.scale.z = 2.0;

  marker16.color.r = 0.0;
  marker16.color.g = 1.0;
  marker16.color.b = 0.0;
  marker16.color.a = 0.75;

  marker16.pose.position.x = marker16_loc.x();
  marker16.pose.position.y = marker16_loc.y();
  marker16.pose.position.z = -1;
  marker16.pose.orientation.x = 0.0;
  marker16.pose.orientation.y = 0.0;
  marker16.pose.orientation.z = 0.0;
  marker16.pose.orientation.w = 1.0;
  marker16.id = 15;

  marker_array_msg.markers[15] = marker16; 

  if(marker_array_msg.markers.size() != 0){
  marker_pub.publish(marker_array_msg);
  // ROS_INFO("Marker array is being published");
  } else{
  ROS_WARN("Marker array is empty");
  }
  
}

void recvHead(const std_msgs::Float64ConstPtr& msg){
  heading = msg->data;
  heading_rad = ((M_PI/180)*heading)-(M_PI/2);

}

void recvFix(const sensor_msgs::NavSatFixConstPtr& msg){
  UTMCoords current_coords(*msg);
  current_coordinates = current_coords;
  relative_position = current_coordinates - ref_coords;
}


void recvCmdVel(const geometry_msgs::Twist::ConstPtr& msg){
  double vel = msg->linear.x;
  double yee_yaw = msg->angular.z; 
   vehicle_heading.linear.x = vel;
   vehicle_heading.angular.z = yee_yaw/2;
   heading_pub.publish(vehicle_heading);

target_checkpoint = marker3_loc; //set target checkpoint

// distance to intersections monitoring
double veh_dis_rel1 = sqrt(((marker1_loc.x()-relative_position.x())*(marker1_loc.x()-relative_position.x()))+((marker1_loc.y()-relative_position.y())*(marker1_loc.y()-relative_position.y())));
double veh_dis_rel2 = sqrt(((marker2_loc.x()-relative_position.x())*(marker2_loc.x()-relative_position.x()))+((marker2_loc.y()-relative_position.y())*(marker2_loc.y()-relative_position.y())));
double veh_dis_rel3 = sqrt(((marker3_loc.x()-relative_position.x())*(marker3_loc.x()-relative_position.x()))+((marker3_loc.y()-relative_position.y())*(marker3_loc.y()-relative_position.y())));
double veh_dis_rel4 = sqrt(((marker4_loc.x()-relative_position.x())*(marker4_loc.x()-relative_position.x()))+((marker4_loc.y()-relative_position.y())*(marker4_loc.y()-relative_position.y())));
double veh_dis_rel5 = sqrt(((marker5_loc.x()-relative_position.x())*(marker5_loc.x()-relative_position.x()))+((marker5_loc.y()-relative_position.y())*(marker5_loc.y()-relative_position.y())));
double veh_dis_rel6 = sqrt(((marker6_loc.x()-relative_position.x())*(marker6_loc.x()-relative_position.x()))+((marker6_loc.y()-relative_position.y())*(marker6_loc.y()-relative_position.y())));
double veh_dis_rel7 = sqrt(((marker7_loc.x()-relative_position.x())*(marker7_loc.x()-relative_position.x()))+((marker7_loc.y()-relative_position.y())*(marker7_loc.y()-relative_position.y())));
double veh_dis_rel8 = sqrt(((marker8_loc.x()-relative_position.x())*(marker8_loc.x()-relative_position.x()))+((marker8_loc.y()-relative_position.y())*(marker8_loc.y()-relative_position.y())));
double veh_dis_rel9 = sqrt(((marker9_loc.x()-relative_position.x())*(marker9_loc.x()-relative_position.x()))+((marker9_loc.y()-relative_position.y())*(marker9_loc.y()-relative_position.y())));
double veh_dis_rel10 = sqrt(((marker10_loc.x()-relative_position.x())*(marker10_loc.x()-relative_position.x()))+((marker10_loc.y()-relative_position.y())*(marker10_loc.y()-relative_position.y())));
double veh_dis_rel11 = sqrt(((marker11_loc.x()-relative_position.x())*(marker11_loc.x()-relative_position.x()))+((marker11_loc.y()-relative_position.y())*(marker11_loc.y()-relative_position.y())));
double veh_dis_rel12 = sqrt(((marker12_loc.x()-relative_position.x())*(marker12_loc.x()-relative_position.x()))+((marker12_loc.y()-relative_position.y())*(marker12_loc.y()-relative_position.y())));
double veh_dis_rel13 = sqrt(((marker13_loc.x()-relative_position.x())*(marker13_loc.x()-relative_position.x()))+((marker13_loc.y()-relative_position.y())*(marker13_loc.y()-relative_position.y())));
double veh_dis_rel14 = sqrt(((marker14_loc.x()-relative_position.x())*(marker14_loc.x()-relative_position.x()))+((marker14_loc.y()-relative_position.y())*(marker14_loc.y()-relative_position.y())));
double veh_dis_rel15 = sqrt(((marker15_loc.x()-relative_position.x())*(marker15_loc.x()-relative_position.x()))+((marker15_loc.y()-relative_position.y())*(marker15_loc.y()-relative_position.y())));
double veh_dis_rel16 = sqrt(((marker16_loc.x()-relative_position.x())*(marker16_loc.x()-relative_position.x()))+((marker16_loc.y()-relative_position.y())*(marker16_loc.y()-relative_position.y())));
// distance to intersections monitoring



// Decision at intersection
// if(veh_dis_rel12 > 50){
 
//    heading_pub.publish(vehicle_heading);
// }else if(veh_dis_rel12 < 60 && veh_dis_rel12 >=8){
//   vehicle_heading.linear.x = veh_dis_rel12/5 +5;
//    heading_pub.publish(vehicle_heading);
// }else if(veh_dis_rel12 < 8 && veh_dis_rel12 >5){
//    vehicle_heading.linear.x = 5;
//    vehicle_heading.angular.z = .75;
//    heading_pub.publish(vehicle_heading);
// }else{
//    vehicle_heading.linear.x = vel;
//    vehicle_heading.angular.z = yee_yaw;
//    heading_pub.publish(vehicle_heading);
double rh_turn_rate = -.6;
double rh_turn_duration =.25;
double lh_turn_rate =  .5;
double lh_turn_duration = .9;
double rh_approach_dist = 8;
double lh_approach_dist = 8;
   if(veh_dis_rel12 > 50){
 
   heading_pub.publish(vehicle_heading);
}else if(veh_dis_rel12 < 60 && veh_dis_rel12 >lh_approach_dist){
  vehicle_heading.linear.x = veh_dis_rel12/5 +5;
   heading_pub.publish(vehicle_heading);
}else if(veh_dis_rel12 = lh_approach_dist){
    ros::Time beginTime = ros::Time::now();
    ros::Duration turn_time = ros::Duration(lh_turn_duration); 
    ros::Time endTime = beginTime + turn_time;
    ros::Time endTime2 = endTime + turn_time;
    while(ros::Time::now() < endTime ){
      vehicle_heading.linear.x = 5;
      vehicle_heading.angular.z = lh_turn_rate;
      heading_pub.publish(vehicle_heading);
      
     }}else if(veh_dis_rel12 <lh_approach_dist){
   vehicle_heading.linear.x = 5;
   vehicle_heading.angular.z = yee_yaw;
   heading_pub.publish(vehicle_heading);
}else{
 vehicle_heading.linear.x = vel;
   vehicle_heading.angular.z = yee_yaw;
   heading_pub.publish(vehicle_heading);

}
   if(veh_dis_rel15 > 50){
 
   heading_pub.publish(vehicle_heading);
}else if(veh_dis_rel15 < 60 && veh_dis_rel15 >rh_approach_dist){
  vehicle_heading.linear.x = veh_dis_rel15/5 +5;
   heading_pub.publish(vehicle_heading);
}else if(veh_dis_rel15 = rh_approach_dist){
    ros::Time beginTime = ros::Time::now();
    ros::Duration turn_time = ros::Duration(rh_turn_duration); 
    ros::Time endTime = beginTime + turn_time;
    ros::Time endTime2 = endTime + turn_time;
    while(ros::Time::now() < endTime ){
      vehicle_heading.linear.x = 5;
      vehicle_heading.angular.z = rh_turn_rate;
      heading_pub.publish(vehicle_heading);
      
     }}else if(veh_dis_rel15 <rh_approach_dist){
   vehicle_heading.linear.x = 5;
   vehicle_heading.angular.z = yee_yaw;
   heading_pub.publish(vehicle_heading);
}else{
 vehicle_heading.linear.x = vel;
   vehicle_heading.angular.z = yee_yaw;
   heading_pub.publish(vehicle_heading);

}
if(veh_dis_rel13 > 50){
 
   heading_pub.publish(vehicle_heading);
}else if(veh_dis_rel13 < 60 && veh_dis_rel13 >lh_approach_dist){
  vehicle_heading.linear.x = veh_dis_rel13/5 +5;
   heading_pub.publish(vehicle_heading);
}else if(veh_dis_rel13 = lh_approach_dist){
    ros::Time beginTime = ros::Time::now();
    ros::Duration turn_time = ros::Duration(lh_turn_duration); 
    ros::Time endTime = beginTime + turn_time;
    ros::Time endTime2 = endTime + turn_time;
    while(ros::Time::now() < endTime ){
      vehicle_heading.linear.x = 5;
      vehicle_heading.angular.z = lh_turn_rate;
      heading_pub.publish(vehicle_heading);
      
     }}else if(veh_dis_rel13 <lh_approach_dist){
   vehicle_heading.linear.x = 5;
   vehicle_heading.angular.z = yee_yaw;
   heading_pub.publish(vehicle_heading);
}else{
 vehicle_heading.linear.x = vel;
   vehicle_heading.angular.z = yee_yaw;
   heading_pub.publish(vehicle_heading);

}if(veh_dis_rel5 > 50){
 
   heading_pub.publish(vehicle_heading);
}else if(veh_dis_rel5 < 60 && veh_dis_rel5 >rh_approach_dist){
  vehicle_heading.linear.x = veh_dis_rel5/5 +5;
   heading_pub.publish(vehicle_heading);
}else if(veh_dis_rel5 = rh_approach_dist){
    ros::Time beginTime = ros::Time::now();
    ros::Duration turn_time = ros::Duration(rh_turn_duration); 
    ros::Time endTime = beginTime + turn_time;
    ros::Time endTime2 = endTime + turn_time;
    while(ros::Time::now() < endTime ){
      vehicle_heading.linear.x = 5;
      vehicle_heading.angular.z = rh_turn_rate;
      heading_pub.publish(vehicle_heading);
      
     }}else if(veh_dis_rel5 <rh_approach_dist){
   vehicle_heading.linear.x = 5;
   vehicle_heading.angular.z = yee_yaw;
   heading_pub.publish(vehicle_heading);
}else{
 vehicle_heading.linear.x = vel;
   vehicle_heading.angular.z = yee_yaw;
   heading_pub.publish(vehicle_heading);

}if(veh_dis_rel4 > 50){
 
   heading_pub.publish(vehicle_heading);
}else if(veh_dis_rel4 < 60 && veh_dis_rel4 >rh_approach_dist){
  vehicle_heading.linear.x = veh_dis_rel4/5 +5;
   heading_pub.publish(vehicle_heading);
}else if(veh_dis_rel4 = rh_approach_dist){
    ros::Time beginTime = ros::Time::now();
    ros::Duration turn_time = ros::Duration(rh_turn_duration); 
    ros::Time endTime = beginTime + turn_time;
    ros::Time endTime2 = endTime + turn_time;
    while(ros::Time::now() < endTime ){
      vehicle_heading.linear.x = 5;
      vehicle_heading.angular.z = rh_turn_rate;
      heading_pub.publish(vehicle_heading);
      
     }}else if(veh_dis_rel4 <rh_approach_dist){
   vehicle_heading.linear.x = 5;
   vehicle_heading.angular.z = yee_yaw;
   heading_pub.publish(vehicle_heading);
}else{
 vehicle_heading.linear.x = vel;
   vehicle_heading.angular.z = yee_yaw;
   heading_pub.publish(vehicle_heading);

}

// ROS_INFO_Debuggers
// ROS_INFO("Current UTM: (%f, %f)", current_coords.getX(), current_coords.getY());
// ROS_INFO("Relative Position: (%f, %f)", relative_position.x(), relative_position.y());
// ROS_INFO("Ref_coords: (%f, %f)", ref_coords.getX(), ref_coords.getY());
// ROS_INFO("Marker1: (%f, %f)", marker1_loc.x(), marker1_loc.y());
// ROS_INFO("Marker1_UTM: (%f, %f)", marker1_l.getX(), marker1_l.getY());
ROS_INFO("Marker12_dist: (%f)", veh_dis_rel12);
ROS_INFO("Marker12_dist: (%f)", veh_dis_rel15);
double rel_ref_x = current_coordinates.getX()- ref_coords.getX();
double rel_ref_y = current_coordinates.getY() - ref_coords.getY();
ROS_INFO("rel_x_dist: (%f)", rel_ref_x);
ROS_INFO("rel_y_dist: (%f)", rel_ref_y);
}

// void recvCmdVel2(const geometry_msgs::Twist& msg){
// }

// void recvPath(const geometry_msgs::Twist& msg){
// }

// void recvPath2(const geometry_msgs::Twist& msg){
// }


int main(int argc, char** argv){
  ros::init(argc,argv,"target");
  ros::NodeHandle nh;
  ros::Timer timer = nh.createTimer(ros::Duration(.5), timerCallback);
  ros::Subscriber gps_loc_sub = nh.subscribe("/audibot/gps/fix",1,recvFix);
  ros::Subscriber gps_head_sub = nh.subscribe("/audibot/gps/heading",1,recvHead);
  // ros::Subscriber vel_sub2 = nh.subscribe("/audibot/cmd_vel",1,recvCmdVel2);
  ros::Subscriber vel_sub = nh.subscribe("/audibot/cmd_vel/remap",1,recvCmdVel);
  // ros::Subscriber path_sub = nh.subscribe("/audibot/target_path",1,recvPath);
  // ros::Subscriber path_sub2 = nh.subscribe("/audibot/path_following",1,recvPath2);
  path_pub = nh.advertise<nav_msgs::Path>("gps_path",1);
  marker_pub = nh.advertise<visualization_msgs::MarkerArray>("marker_array", 1);
  heading_pub = nh.advertise<geometry_msgs::Twist>("/audibot/cmd_vel",1);
  
  


  
  LatLon ref_coords_lat_lon(42.000001,-82.999978,0); //found ref coords by echoing gps coordinates of audibot
  ref_coords = UTMCoords(ref_coords_lat_lon);

 
  
  ros::spin();
}