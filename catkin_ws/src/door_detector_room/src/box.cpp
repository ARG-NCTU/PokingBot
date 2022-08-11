#include <stdio.h>
#include <iostream>
#include <cstdlib>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <geometry_msgs/Vector3.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetLinkState.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <cmath>
#include <string>

using namespace ros;
using namespace std;
using namespace pcl;

class Box{
private:
  tf::TransformBroadcaster tf_br;
  tf::TransformListener tf_listener;
  tf::StampedTransform tf_pose;
  tf::Transform transform;
  tf::Quaternion q;
  Eigen::Matrix4f pose_matrix;

  float rotation_rad, rotate_x, rotate_y;
  int count_box = 0;
  geometry_msgs::PointStamped map_frame_laserpoints[241];

  Publisher pub_scan_label, pub_door_string, pub_room_info;
  Subscriber sub_scan;
  ServiceClient ser_client, link_client;

  gazebo_msgs::GetModelState getmodelstate, robotstate;
  gazebo_msgs::GetLinkState getlinkstate;
  sensor_msgs::LaserScan input_scan, output_scan;

public:
  Box(NodeHandle &nh){

    pub_scan_label = nh.advertise<sensor_msgs::LaserScan>("/RL/scan_label", 1);
    pub_door_string = nh.advertise<std_msgs::String>("/RL/door_string", 1);
    pub_room_info = nh.advertise<std_msgs::Int16>("/RL/room_info", 1);
    sub_scan = nh.subscribe("/RL/scan", 1, &Box::scan_cb, this);
    ser_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    link_client = nh.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
    ROS_INFO("box detection service initialized");
  }

  void scan_process();
  void scan_cb(const sensor_msgs::LaserScan msg);
  bool get_tf();
  ~Box(){};
};

bool Box::get_tf(){
  /*
  
    Transfer lidar tf from robot to box.

    Return:

      bool : true is success, false is fail.

  */
  //map frame tf broadcast
  transform.setOrigin(tf::Vector3(robotstate.response.pose.position.x,
                                  robotstate.response.pose.position.y,
                                  robotstate.response.pose.position.z));
  tf::Quaternion q(robotstate.response.pose.orientation.x,
                    robotstate.response.pose.orientation.y,
                    robotstate.response.pose.orientation.z,
                    robotstate.response.pose.orientation.w);
  transform.setRotation(q);
  tf_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // get door frame TF
  getmodelstate.request.model_name = "pull_box";
  if (ser_client.call(getmodelstate)) ;
  else return 0;
  string tmp = "::link_0";
  getlinkstate.request.link_name = "pull_box"+tmp;
  if (link_client.call(getlinkstate)) ;
  else return 0;
  rotate_x = getmodelstate.response.pose.position.x;
  rotate_y = getmodelstate.response.pose.position.y;
  // TF Broadcaster
  transform.setOrigin( tf::Vector3(rotate_x, rotate_y, 0) ); // z = 0

  tf::Quaternion q_box(getlinkstate.response.link_state.pose.orientation.x,
                    getlinkstate.response.link_state.pose.orientation.y,
                    getlinkstate.response.link_state.pose.orientation.z,
                    getlinkstate.response.link_state.pose.orientation.w);

  transform.setRotation(q_box);//tf::Transform transform;
  tf_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "model_box"));

  // prepare for transforming LaserScan from "front_laser" to "model_box"
  try{
      tf_listener.waitForTransform("model_box", "front_laser", ros::Time(0), ros::Duration(0.2));
      tf_listener.lookupTransform("model_box", "front_laser", ros::Time(0), tf_pose);
  }catch (tf::TransformException ex){
      ROS_ERROR("%s", ex.what());
      return 0;
  }
  return 1;
}

void Box::scan_cb(const sensor_msgs::LaserScan msg){
  /*
  
    Get lidar data and run preprocessing.

    Args:

      msg(const sensor_msgs::LaserScan) : laser data.

  */
  count_box = 0;
  input_scan = msg;
  output_scan.ranges.assign(msg.ranges.size(), std::numeric_limits<double>::infinity());
  output_scan.intensities.assign(msg.ranges.size(), std::numeric_limits<double>::infinity());
  for(int i=0;i<input_scan.ranges.size();i++){
    output_scan.ranges[i] = input_scan.ranges[i];
    output_scan.intensities[i] = 0;
  }

  // Which door should I detect
  getmodelstate.request.model_name = "robot";
  if (ser_client.call(getmodelstate)) ;
  else{
    ROS_ERROR("gazebo service get robot state fail");
    return ;
  }
  robotstate = getmodelstate;

  if(get_tf())
    scan_process();

  output_scan.header = msg.header;
  output_scan.angle_min = msg.angle_min;
  output_scan.angle_max = msg.angle_max;
  output_scan.angle_increment = msg.angle_increment;
  output_scan.time_increment = msg.time_increment;
  output_scan.scan_time = msg.scan_time;
  output_scan.range_min = msg.range_min;
  output_scan.range_max = msg.range_max;
  pub_scan_label.publish(output_scan);

  cout<<"total scanpoints:"<<output_scan.ranges.size()<<" box:"<<count_box<<endl;
}

void Box::scan_process(){

  /*

    Transfer raw laser to label laser on the certain range.

  */

  if(input_scan.ranges.size()>0){
    float o_t_min = input_scan.angle_min, o_t_max = input_scan.angle_max, o_t_inc = input_scan.angle_increment;
    for(int i=0;i<input_scan.ranges.size();i++){
      float theta = o_t_min+i*o_t_inc, r = input_scan.ranges[i];
      geometry_msgs::PointStamped pt;
      pt.header.frame_id = "front_laser", pt.point.x = r*cos(theta), pt.point.y = r*sin(theta);
      try{tf_listener.transformPoint("model_box", pt, pt);}
      catch (tf::TransformException ex){
          ROS_ERROR("%s", ex.what());
          return;
      }
      if((pt.point.y<0.5) && (pt.point.y>-0.5) && (pt.point.x<(0.6)) && (pt.point.x>-0.6)){
        output_scan.intensities[i] = 1, count_box++;
      }
    }
  }
}

int main(int argc, char **argv){
  init(argc, argv, "box");
  NodeHandle nh;
  Box box(nh);
  spin();
  return 0;
}
