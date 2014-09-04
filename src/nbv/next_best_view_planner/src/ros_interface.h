/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2014  <copyright holder> <email>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/


#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H

#include <ros/ros.h>
#include "planner.h"
#include "workcell.h"
#include "path_optimization.h"

#include <boost/foreach.hpp>

#include <rw/math.hpp>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <robot_srvs/PrePlanViewPoints.h>
#include <robot_srvs/NBV_MoveToNextPrePlannedPose.h>

#include <robot_msgs/MoveCmd.h>
#include <sensor_msgs/JointState.h>

namespace next_best_view_planner
{
  static const unsigned char MOVE_J=0, MOVE_L=1, MOVE_P2P=2, MOVE_JN=3, MOVE_LN=4;

class ros_interface
{

public:
  
  ros_interface(boost::shared_ptr<Planner> planner_ptr,boost::shared_ptr<workcell> workcell_ptr, 
		boost::shared_ptr<Path_optimization> optimization_ptr ,ros::NodeHandle n);
  virtual ~ros_interface();
  
  bool init();
  bool publish_nbv_marker(const std::vector<rw::math::Transform3D<> > &pose_list );
  bool prePlan(robot_srvs::PrePlanViewPoints::Request &req, robot_srvs::PrePlanViewPoints::Response &res);
  bool moveToNextPrePlannedPose(robot_srvs::NBV_MoveToNextPrePlannedPose::Request &req, robot_srvs::NBV_MoveToNextPrePlannedPose::Response &res);
  
  bool moveTrajectory(rw::trajectory::Path<Q> path);
  bool moveJ(rw::math::Q q);
  bool moveL(rw::math::Transform3D<> T);
  
private:
  ros::NodeHandle _n;
  
  std::string _nbv_pose_topic;
  std::string _planning_frame;
  double _vel;
  double _acc;
  
  std::vector<rw::math::Transform3D<> > _view_points;
  int _view_count;
  rw::math::Q _currentQ;
  
  ros::Subscriber _sub_nbv_pose;
  ros::Subscriber _joint_state;
  boost::shared_ptr<Planner> _planner;
  boost::shared_ptr<workcell> _workcell;
  boost::shared_ptr<Path_optimization> _optimization;
  
    // Publishes the octree in MarkerArray format so that it can be visualized in rviz
  ros::Publisher _nbv_marker_array_publisher;
  // Marker array to visualize the different view points. It displays the view point with an arrow
  visualization_msgs::MarkerArray _nbv_marker_array_msg;
  
  ros::ServiceServer _service_PrePlan;
  ros::ServiceServer _srv_move2PrePlanPose;
  ros::Publisher _movePublisher;
  ros::Publisher _movePublisher_buffer;
  ros::Publisher _movePublisher_bufferExe;
  
private:
  bool nbv_pose_CB(const geometry_msgs::PoseArray::Ptr &msg);
  void joint_stateCB(const sensor_msgs::JointState::Ptr &msg);
  

  
};

}
#endif // ROS_INTERFACE_H
