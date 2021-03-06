/*
 * rrtstar_visualization.cpp
 *
 * ---------------------------------------------------------------------
 * Copyright (C) 2022 Matthew (matthewoots at gmail.com)
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * ---------------------------------------------------------------------
 *
 * 
 * 
 */

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

#include <string>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>

#include <geometry_msgs/Point.h>

#include "rrt_pcl_ros/point_array.h"
#include <std_msgs/Float32MultiArray.h>

#define check_size 2

using namespace std;
using namespace Eigen;

ros::Publisher rrt_marker_pub, bs_marker_pub;
ros::Publisher ellipsoid_marker_pub;
ros::Subscriber bs_message, rrt_message, solo_msg_sub, formation_msg_sub;

vector<double> formation_param, solo_param;

bool _is_using_formation;

bool message_check[check_size] = {false, false};

double _obs_threshold, _scale_z;

void rrt_callback(const rrt_pcl_ros::point_array::ConstPtr &msg)
{
  rrt_pcl_ros::point_array rrt = *msg;

  visualization_msgs::Marker rrt_points, line_strip, ellipsoid;
  rrt_points.header.frame_id = line_strip.header.frame_id = 
    ellipsoid.header.frame_id = "/map";
  rrt_points.header.stamp = line_strip.header.stamp = 
    ellipsoid.header.stamp = ros::Time::now();
  rrt_points.ns = line_strip.ns = ellipsoid.ns = "rrt_visualization_points";
  rrt_points.action = line_strip.action = ellipsoid.action = visualization_msgs::Marker::ADD;
  rrt_points.pose.orientation.w = line_strip.pose.orientation.w =
    ellipsoid.pose.orientation.w = 1.0;

  rrt_points.id = 0;
  line_strip.id = 1;

  rrt_points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  ellipsoid.type = visualization_msgs::Marker::SPHERE;

  // POINTS markers use x and y scale for width/height respectively
  rrt_points.scale.x = 0.2;
  rrt_points.scale.y = 0.2;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.1;

  // Points color
  rrt_points.color.g = 1.0f;
  rrt_points.color.b = 1.0f;
  rrt_points.color.a = 1.0f;

  // Line strip color
  line_strip.color.r = 1.0f;
  line_strip.color.b = 1.0f;
  line_strip.color.a = 1.0f;

  int rrt_size = rrt.array.size();
  // Create the vertices for the points and lines
  for (int i = 0; i < rrt_size; i++)
  {
    geometry_msgs::Point p;
    p.x = rrt.array[i].x;
    p.y = rrt.array[i].y;
    p.z = rrt.array[i].z;

    rrt_points.points.push_back(p);
    line_strip.points.push_back(p);

    ellipsoid.id = i;
    ellipsoid.scale.x = _obs_threshold;
    ellipsoid.scale.y = _obs_threshold;
    ellipsoid.scale.z = (1 /_scale_z) * _obs_threshold;
    // Ellipsoid color
    ellipsoid.color.b = 1.0f;
    ellipsoid.color.a = 0.5f;
    ellipsoid.pose.position = p;

    ellipsoid_marker_pub.publish(ellipsoid);
  }

  rrt_marker_pub.publish(rrt_points);
  rrt_marker_pub.publish(line_strip);
}


void bs_callback(const rrt_pcl_ros::point_array::ConstPtr &msg)
{
  rrt_pcl_ros::point_array rrt = *msg;

  visualization_msgs::Marker cp_points, line_strip;
  cp_points.header.frame_id = line_strip.header.frame_id = "/map";
  cp_points.header.stamp = line_strip.header.stamp = ros::Time::now();
  cp_points.ns = line_strip.ns = "bspline_visualization_points";
  cp_points.action = line_strip.action = visualization_msgs::Marker::ADD;
  cp_points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

  cp_points.id = 0;
  line_strip.id = 1;

  cp_points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;


  // POINTS markers use x and y scale for width/height respectively
  cp_points.scale.x = 0.2;
  cp_points.scale.y = 0.2;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.1;

  // Points color
  cp_points.color.r = 1.0f;
  cp_points.color.g = 1.0f;
  cp_points.color.a = 1.0f;

  // Line strip color
  line_strip.color.b = 1.0f;
  line_strip.color.g = 1.0f;
  line_strip.color.a = 1.0f;

  int rrt_size = rrt.array.size();
  // Create the vertices for the points and lines
  for (int i = 0; i < rrt_size; i++)
  {
    geometry_msgs::Point p;
    p.x = rrt.array[i].x;
    p.y = rrt.array[i].y;
    p.z = rrt.array[i].z;

    cp_points.points.push_back(p);
    line_strip.points.push_back(p);
  }

  bs_marker_pub.publish(cp_points);
  bs_marker_pub.publish(line_strip);
}

/** @brief Handles formation parameters from float64 array */
void formationParamMsgCallBack(const  std_msgs::Float32MultiArray::ConstPtr &msg)
{
    std_msgs::Float32MultiArray multi_array = *msg;
    
    formation_param.clear();
    for (int i = 0; i < multi_array.data.size(); i++)
    {
        formation_param.push_back((double)multi_array.data[i]);
    }
    message_check[0] = true;
    printf("%s[viz.cpp] RRT Formation Param Msg received! \n", KGRN);
}

/** @brief Handles solo parameters from float64 array */
void soloParamMsgCallBack(const  std_msgs::Float32MultiArray::ConstPtr &msg)
{
    std_msgs::Float32MultiArray multi_array = *msg;
    
    solo_param.clear();
    for (int i = 0; i < multi_array.data.size(); i++)
    {
        solo_param.push_back((double)multi_array.data[i]);
    }
    message_check[1] = true;
    printf("%s[viz.cpp] RRT Solo Param Msg received! \n", KGRN);
}

void update_param()
{
  if (_is_using_formation)
  {
    _obs_threshold = formation_param[1];
    _scale_z = formation_param[9];
  }
  else
  {
    _obs_threshold = solo_param[1];
    _scale_z = solo_param[9];
  }
}


int main( int argc, char** argv )
{
  double rate = 1.0;
  
  ros::init(argc, argv, "rrt_pcl_ros_visualization");
  ros::NodeHandle n("~");

  n.param<bool>("is_using_formation", _is_using_formation, false);

  bs_marker_pub = n.advertise<visualization_msgs::Marker>(
        "/bs_visualization_marker", 10);
  rrt_marker_pub = n.advertise<visualization_msgs::Marker>(
        "/rrt_visualization_marker", 10);
  ellipsoid_marker_pub = n.advertise<visualization_msgs::Marker>(
        "/ellipsoid_visualization_marker", 10);

  bs_message = n.subscribe<rrt_pcl_ros::point_array>(
        "/bs", 10, &bs_callback);      
  rrt_message = n.subscribe<rrt_pcl_ros::point_array>(
        "/rrt", 10, &rrt_callback);
  /** 
  * @brief Handles solo parameters from float64 array
  */
  solo_msg_sub = n.subscribe<std_msgs::Float32MultiArray>(
      "/param/solo_settings",10,  &soloParamMsgCallBack);
  /** 
  * @brief Handles formation parameters from float64 array
  */
  formation_msg_sub = n.subscribe<std_msgs::Float32MultiArray>(
      "/param/formation_settings", 10, &formationParamMsgCallBack);

  ros::Rate r(rate);

  while (ros::ok())
  {
    int status_pass = 0;
    for (int i = 0; i < check_size; i++)
    {
        if (message_check[i])
            status_pass++;
    }
    
    if (status_pass == check_size)
      update_param();

    ros::spinOnce();
    ros::Duration(rate).sleep();
  }
  return 0;
}


  



