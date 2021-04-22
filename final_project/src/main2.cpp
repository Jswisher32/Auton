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
tf::Vector3 UTMvector_x;
tf::Vector3 UTMvector_y;
tf::Vector3 CARTvector_x;
tf::Vector3 CARTvector_y;
tf::Vector3 target_checkpoint;
tf::Vector3 target_checkpointUTM;
UTMCoords ref_coords;
UTMCoords marker1_l;
UTMCoords marker2_l;
UTMCoords marker3_l;
UTMCoords marker4_l;
UTMCoords marker5_l;
UTMCoords marker6_l;
UTMCoords marker7_l;
UTMCoords marker8_l;
visualization_msgs::MarkerArray marker_array_msg;

nav_msgs::Path gps_path;
double heading;
double heading_rad;
double theta;
double heading_error;
double heading_correction;
double convergence_angle;
double central_meridian;
double veh_dis_rel1;
geometry_msgs::Twist vehicle_heading;
//ackerman variables
const double ANGLE_RATIO = 17.3;
const double WHEEL_BASE = 2.65;
ros::Publisher vel_pub;
ros::Publisher steer_pub;
ros::Publisher heading_pub;
ros::Publisher brake_pub;
ros::Publisher throttle_pub;
ros::WallTime start_, end_;

void timerCallback(const ros::TimerEvent& event){

 
  geometry_msgs::PoseStamped current_pose;
  current_pose.pose.position.x = relative_position.x();
  current_pose.pose.position.y = relative_position.y();
  
  gps_path.poses.push_back(current_pose);
  gps_path.header.frame_id="world";
  gps_path.header.stamp = event.current_real;
  path_pub.publish(gps_path);

  
  
  marker_array_msg.markers.resize(8);

  visualization_msgs::Marker marker1;
  visualization_msgs::Marker marker2;
  visualization_msgs::Marker marker3;
  visualization_msgs::Marker marker4;
  visualization_msgs::Marker marker5;
  visualization_msgs::Marker marker6;
  visualization_msgs::Marker marker7;
  visualization_msgs::Marker marker8;

  
  marker1.header.frame_id = "world";
  marker1.header.stamp = event.current_real;
  marker1.type = visualization_msgs::Marker::CYLINDER;
  marker1.action = visualization_msgs::Marker::ADD;

  LatLon marker1_coords(42.851358,-83.069485,0);
  marker1_l = UTMCoords(marker1_coords);
  marker1_loc = marker1_l - ref_coords;

  marker1.scale.x = 2.0;
  marker1.scale.y = 2.0;
  marker1.scale.z = 10.0;

  marker1.color.r = 1.0;
  marker1.color.g = 0.0;
  marker1.color.b = 0.0;
  marker1.color.a = 0.75;

  marker1.pose.position.x = marker1_loc.x();
  marker1.pose.position.y = marker1_loc.y();
  marker1.pose.position.z = 5.0;
  marker1.pose.orientation.x = 0.0;
  marker1.pose.orientation.y = 0.0;
  marker1.pose.orientation.z = 0.0;
  marker1.pose.orientation.w = 1.0;
  marker1.id = 0;

  marker_array_msg.markers[0] = marker1;
 
  marker2.header.frame_id = "world";
  marker2.header.stamp = event.current_real;
  marker2.type = visualization_msgs::Marker::CYLINDER;
  marker2.action = visualization_msgs::Marker::ADD;

  LatLon marker2_coords(42.851383,-83.069007,0);
  marker2_l = UTMCoords(marker2_coords);
  marker2_loc = marker2_l - ref_coords;

  marker2.scale.x = 2.0;
  marker2.scale.y = 2.0;
  marker2.scale.z = 10.0;

  marker2.color.r = 0.0;
  marker2.color.g = 1.0;
  marker2.color.b = 0.0;
  marker2.color.a = 0.75;

  marker2.pose.position.x = marker2_loc.x();
  marker2.pose.position.y = marker2_loc.y();
  marker2.pose.position.z = 5.0;
  marker2.pose.orientation.x = 0.0;
  marker2.pose.orientation.y = 0.0;
  marker2.pose.orientation.z = 0.0;
  marker2.pose.orientation.w = 1.0;
  marker2.id = 1;

  marker_array_msg.markers[1] = marker2;
  
  marker3.header.frame_id = "world";
  marker3.header.stamp = event.current_real;
  marker3.type = visualization_msgs::Marker::CYLINDER;
  marker3.action = visualization_msgs::Marker::ADD;

  LatLon marker3_coords(42.852443,-83.068013,0);
  marker3_l = UTMCoords(marker3_coords);
  marker3_loc = marker3_l - ref_coords;

  marker3.scale.x = 2.0;
  marker3.scale.y = 2.0;
  marker3.scale.z = 10.0;

  marker3.color.r = 0.0;
  marker3.color.g = 1.0;
  marker3.color.b = 0.0;
  marker3.color.a = 0.75;

  marker3.pose.position.x = marker3_loc.x();
  marker3.pose.position.y = marker3_loc.y();
  marker3.pose.position.z = 5.0;
  marker3.pose.orientation.x = 0.0;
  marker3.pose.orientation.y = 0.0;
  marker3.pose.orientation.z = 0.0;
  marker3.pose.orientation.w = 1.0;
  marker3.id = 2;

  marker_array_msg.markers[2] = marker3;
  
  marker4.header.frame_id = "world";
  marker4.header.stamp = event.current_real;
  marker4.type = visualization_msgs::Marker::CYLINDER;
  marker4.action = visualization_msgs::Marker::ADD;

  LatLon marker4_coords(42.852021,-83.066888,0);
  marker4_l = UTMCoords(marker4_coords);
  marker4_loc = marker4_l - ref_coords;

  marker4.scale.x = 2.0;
  marker4.scale.y = 2.0;
  marker4.scale.z = 10.0;

  marker4.color.r = 0.0;
  marker4.color.g = 1.0;
  marker4.color.b = 0.0;
  marker4.color.a = 0.75;

  marker4.pose.position.x = marker4_loc.x();
  marker4.pose.position.y = marker4_loc.y();
  marker4.pose.position.z = 5.0;
  marker4.pose.orientation.x = 0.0;
  marker4.pose.orientation.y = 0.0;
  marker4.pose.orientation.z = 0.0;
  marker4.pose.orientation.w = 1.0;
  marker4.id = 3;

  marker_array_msg.markers[3] = marker4;
 
  marker5.header.frame_id = "world";
  marker5.header.stamp = event.current_real;
  marker5.type = visualization_msgs::Marker::CYLINDER;
  marker5.action = visualization_msgs::Marker::ADD;

  LatLon marker5_coords(42.851525,-83.067044,0);
  marker5_l = UTMCoords(marker5_coords);
  marker5_loc = marker5_l - ref_coords;

  marker5.scale.x = 2.0;
  marker5.scale.y = 2.0;
  marker5.scale.z = 10.0;

  marker5.color.r = 0.0;
  marker5.color.g = 1.0;
  marker5.color.b = 0.0;
  marker5.color.a = 0.75;

  marker5.pose.position.x = marker5_loc.x();
  marker5.pose.position.y = marker5_loc.y();
  marker5.pose.position.z = 5.0;
  marker5.pose.orientation.x = 0.0;
  marker5.pose.orientation.y = 0.0;
  marker5.pose.orientation.z = 0.0;
  marker5.pose.orientation.w = 1.0;
  marker5.id = 4;

  marker_array_msg.markers[4] = marker5;

  marker6.header.frame_id = "world";
  marker6.header.stamp = event.current_real;
  marker6.type = visualization_msgs::Marker::CYLINDER;
  marker6.action = visualization_msgs::Marker::ADD;

  LatLon marker6_coords(42.851344,-83.066344,0);
  marker6_l = UTMCoords(marker6_coords);
  marker6_loc = marker6_l - ref_coords;

  marker6.scale.x = 2.0;
  marker6.scale.y = 2.0;
  marker6.scale.z = 10.0;

  marker6.color.r = 0.0;
  marker6.color.g = 1.0;
  marker6.color.b = 0.0;
  marker6.color.a = 0.75;

  marker6.pose.position.x = marker6_loc.x();
  marker6.pose.position.y = marker6_loc.y();
  marker6.pose.position.z = 5.0;
  marker6.pose.orientation.x = 0.0;
  marker6.pose.orientation.y = 0.0;
  marker6.pose.orientation.z = 0.0;
  marker6.pose.orientation.w = 1.0;
  marker6.id = 5;

  marker_array_msg.markers[5] = marker6;
 
  marker7.header.frame_id = "world";
  marker7.header.stamp = event.current_real;
  marker7.type = visualization_msgs::Marker::CYLINDER;
  marker7.action = visualization_msgs::Marker::ADD;

  LatLon marker7_coords(42.850836,-83.066440,0);
  marker7_l = UTMCoords(marker7_coords);
  marker7_loc = marker7_l - ref_coords;

  marker7.scale.x = 2.0;
  marker7.scale.y = 2.0;
  marker7.scale.z = 10.0;

  marker7.color.r = 255.0;
  marker7.color.g = 192.0;
  marker7.color.b = 203.0;
  marker7.color.a = 0.75;

  marker7.pose.position.x = marker7_loc.x();
  marker7.pose.position.y = marker7_loc.y();
  marker7.pose.position.z = 5.0;
  marker7.pose.orientation.x = 0.0;
  marker7.pose.orientation.y = 0.0;
  marker7.pose.orientation.z = 0.0;
  marker7.pose.orientation.w = 1.0;
  marker7.id = 6;

  marker_array_msg.markers[6] = marker7;
 
  marker8.header.frame_id = "world";
  marker8.header.stamp = event.current_real;
  marker8.type = visualization_msgs::Marker::CYLINDER;
  marker8.action = visualization_msgs::Marker::ADD;

  LatLon marker8_coords(42.849644,-83.066060,0);
  marker8_l = UTMCoords(marker8_coords);
  marker8_loc = marker8_l - ref_coords;

  marker8.scale.x = 2.0;
  marker8.scale.y = 2.0;
  marker8.scale.z = 10.0;

  marker8.color.r = 0.0;
  marker8.color.g = 1.0;
  marker8.color.b = 0.0;
  marker8.color.a = 0.75;

  marker8.pose.position.x = marker8_loc.x();
  marker8.pose.position.y = marker8_loc.y();
  marker8.pose.position.z = 5.0;
  marker8.pose.orientation.x = 0.0;
  marker8.pose.orientation.y = 0.0;
  marker8.pose.orientation.z = 0.0;
  marker8.pose.orientation.w = 1.0;
  marker8.id = 7;

  marker_array_msg.markers[7] = marker8; 

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

     
relative_position = current_coords - ref_coords;
  
  // veh_dis_rel1 = sqrt(((marker1_loc.x()-relative_position.x())*(marker1_loc.x()-relative_position.x()))+((marker1_loc.y()-relative_position.y())*(marker1_loc.y()-relative_position.y())));
  veh_dis_rel1 = sqrt(((target_checkpoint[0]-relative_position.x())*(target_checkpoint[0]-relative_position.x()))+((target_checkpoint[1]-relative_position.y())*(target_checkpoint[1]-relative_position.y())));
  // veh_dis_rel1 = sqrt(((loc_x-relative_position.x())*loc_x-relative_position.x()))+((loc_y-relative_position.y())*(loc_y-relative_position.y()));

  // theta = atan2((marker1_l.getY()-current_coords.getY()),marker1_l.getX()-current_coords.getX()); 
  // theta = atan2((marker1_loc.y()-relative_position.y()),marker1_loc.x()-relative_position.x()); 
  theta = atan2((target_checkpoint[1]-relative_position.y()),target_checkpoint[0]-relative_position.x());
  // theta = atan2((target_checkpointUTM.getY()-current_coords.getY()),target_checkpointUTM.getX()-current_coords.getX());  
  // convergence_angle = atan(tan(current_coords.getX()-ref_coords.getCentralMeridian())*sin(current_coords.getY()));
  heading_correction = heading_rad;
  heading_error = theta + heading_correction;

//////////////////////////Controls//////////////////////////////

double speed_turn;

  if(heading_error > M_PI*1.1){
    speed_turn = heading_error - 2*M_PI;
    vehicle_heading.angular.z = heading_error - 2*M_PI;
  }else if(heading_error < -M_PI*1.1){
    speed_turn = heading_error + 2*M_PI;
    vehicle_heading.angular.z - heading_error + 2*M_PI;
    }else{
      speed_turn = heading_error;
      vehicle_heading.angular.z = heading_error;
    }

  if(veh_dis_rel1 <=1){
      vehicle_heading.linear.x = 100;
      
    for (i++ ; i < 9;) {
      target_checkpoint[0] = marker_array_msg.markers[i-1].pose.position.x;
      target_checkpoint[1] = marker_array_msg.markers[i-1].pose.position.y;  
        ROS_INFO("Current Checkpoint Position: (%f, %f)", target_checkpoint[0],target_checkpoint[1]);
        ROS_INFO("Current Checkpoint = (%i)", i);
        ROS_WARN("%s", "Checkpoint Complete!");
        break;
        }
    
    }else if(veh_dis_rel1 <= 20 || 1.5 <=heading_error<=1.5){
      vehicle_heading.linear.x = .3*veh_dis_rel1+12;
      vehicle_heading.angular.z = speed_turn*3;
    }else if(veh_dis_rel1 > 20 && 0.2>heading_error>-0.2){
      vehicle_heading.linear.x = 100;
      }else{
        vehicle_heading.linear.x = .3*veh_dis_rel1+45;
        vehicle_heading.angular.z = speed_turn/4;
      }

      //  if(theta  > 1.9*M_PI ){
      // heading_error = -(theta + heading_correction);
      // vehicle_heading.angular.z = heading_error -2*M_PI;
      //   }
      // else if(theta < -1.9*M_PI){
      //   heading_error = theta + heading_correction;
      //   vehicle_heading.angular.z = heading_error +2*M_PI;
      // }

//  if(heading_error > 1.5*M_PI){
//     heading_error = theta + heading_correction - 2*M_PI;
//     // vehicle_heading.angular.z = heading_error;
//     }else if(heading_error < -1.5*M_PI){
//       heading_error = theta + heading_correction + 2*M_PI;
//       // vehicle_heading.angular.z = heading_error;
//         }
//       else{
//         heading_error = theta + heading_correction;
//         // vehicle_heading.angular.z = heading_error/1.5;
//       }

  // if( theta < 0){
  //   vehicle_heading.angular.z = 1*heading_error;
        
  //   }else if(theta > 0){
  //     vehicle_heading.angular.z = -1*heading_error;
  //   }else{
  //     vehicle_heading.angular.z = 0;
  //     }
  


//   if(veh_dis_rel1 <=1){
//     for (i++ ; i < 9;) {
//       // start_ = ros::WallTime::now();
//       target_checkpoint[0] = marker_array_msg.markers[i-1].pose.position.x;
//       target_checkpoint[1] = marker_array_msg.markers[i-1].pose.position.y;
//       // target_checkpointUTM[0] = marker_array_msg.markers[i-1].pose.position.x;
//       // target_checkpointUTM[1] = marker_array_msg.markers[i-1].pose.position.y;  
//         ROS_INFO("Current Checkpoint Position: (%f, %f)", target_checkpoint[0],target_checkpoint[1]);
//         ROS_INFO("Current Checkpoint = (%i)", i);
//         ROS_WARN("%s", "Checkpoint Complete!");
//         break;
//         }
// }
  // if(veh_dis_rel1 > 200){
  //   vehicle_heading.linear.x = 50;
  //   }else if(veh_dis_rel1 <= 100){
  //     vehicle_heading.linear.x = .45*veh_dis_rel1+5;
  //   }else if(veh_dis_rel1 = 0){
  //     vehicle_heading.linear.x = 50;
  //   }  
  

    // }else if(veh_dis_rel1 <= 10){
    // vehicle_heading.linear.x = 15*v;
    // vehicle_heading.angular.z = heading_error*2;
    // }else if(veh_dis_rel1 <= 20){
    // vehicle_heading.linear.x = 25*v;
    // vehicle_heading.angular.z = heading_error;
    // }else if(veh_dis_rel1 <= 30){
    // vehicle_heading.linear.x = 30*v;
    // vehicle_heading.angular.z = heading_error;
    // }else if(veh_dis_rel1 <= 40){
    // vehicle_heading.linear.x = 40*v;
    // vehicle_heading.angular.z = heading_error;
    // }else if(veh_dis_rel1 <= 50){
    // vehicle_heading.linear.x = 50*v;
    // vehicle_heading.angular.z = heading_error;
    // }else if(veh_dis_rel1 <= 75){
    // vehicle_heading.linear.x = 75*v;
    // vehicle_heading.angular.z = heading_error;
    // }else if(veh_dis_rel1 <= 100){
    // vehicle_heading.linear.x = 100*v;
    // vehicle_heading.angular.z = heading_error;
    // }else if(veh_dis_rel1 <= 125){
    // vehicle_heading.linear.x = 125*v;
    // vehicle_heading.angular.z = heading_error;
    // }else if(veh_dis_rel1 <= 150){
    // vehicle_heading.linear.x = 150*v;
    // vehicle_heading.angular.z = heading_error;
    // }else if(veh_dis_rel1 <= 200){
    // vehicle_heading.linear.x = 200*v;
    // vehicle_heading.angular.z = heading_error;
    // }else{
    // vehicle_heading.linear.x = 200*v;
    // vehicle_heading.angular.z = heading_error;
  
  // vehicle_heading.angular.z = heading_error;
  double brakes = 10;
  std_msgs::Float64 brake;
  brake.data = brakes;
  heading_pub.publish(vehicle_heading);
  brake_pub.publish(brake);


  

  // ROS_INFO("Theta: (%f)", theta);
  // ROS_INFO("Convergence angle: (%f)", convergence_angle);
  // ROS_INFO("Heading rad: (%f)", heading_rad);
  // ROS_INFO("Heading : (%f)", heading);
  // ROS_INFO("Heading error: (%f)", heading_error);
  // ROS_INFO("Heading Correction: (%f)", heading_correction);
  // ROS_INFO("Dist to 1: (%f)", veh_dis_rel1);
  // ROS_INFO("Current UTM: (%f, %f)", current_coords.getX(), current_coords.getY());
  // ROS_INFO("Relative Position: (%f, %f)", relative_position.x(), relative_position.y());

  

  

}

void recvBrake(const std_msgs::Float64ConstPtr& msg){
  std_msgs::Float64 brakes;
  brakes.data = msg->data + 1;
  // brake_pub.publish(brakes);
}

int main(int argc, char** argv){
  ros::init(argc,argv,"main2");
  ros::NodeHandle nh;
  ros::Timer timer = nh.createTimer(ros::Duration(.5), timerCallback);
  ros::Subscriber gps_loc_sub = nh.subscribe("/audibot/gps/fix",1,recvFix);
  ros::Subscriber gps_head_sub = nh.subscribe("/audibot/gps/heading",1,recvHead);
  ros::Subscriber brake_sub = nh.subscribe("/audibot/brake_cmd",1,recvBrake);
  
  path_pub = nh.advertise<nav_msgs::Path>("gps_path",1);
  marker_pub = nh.advertise<visualization_msgs::MarkerArray>("marker_array", 1);
  // vel_pub = nh.advertise<geometry_msgs::Twist>("/audibot/cmd_vel", 1);
  // steer_pub = nh.advertise<std_msgs::Float64>("/audibot/steering_cmd", 10);
  heading_pub = nh.advertise<geometry_msgs::Twist>("/audibot/cmd_vel",1);
  brake_pub = nh.advertise<std_msgs::Float64>("audibot/brake_cmd",1);
  // throttle_pub = nh.advertise<geometry_msgs::Twist>("/audibot/throttle_cmd",1);
  
  
  
  
  double ref_lat;
  double ref_lon;

  nh.getParam("/audibot/gps/ref_lat",ref_lat);
  nh.getParam("/audibot/gps/ref_lon",ref_lon);
  
  LatLon ref_coords_lat_lon(ref_lat, ref_lon, 0);
  ref_coords = UTMCoords(ref_coords_lat_lon);

  central_meridian = ref_coords.getCentralMeridian();

  // ROS_INFO("Central Meridian of the Reference Cooridinate: %f", central_meridian);
  // ros::Duration(5).sleep(); // sleep 
  ros::spin();
}