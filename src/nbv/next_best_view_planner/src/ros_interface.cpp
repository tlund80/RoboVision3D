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


#include "ros_interface.h"

namespace next_best_view_planner
{

ros_interface::ros_interface(boost::shared_ptr<Planner> planner_ptr,
			     boost::shared_ptr<workcell> workcell_ptr,
			     boost::shared_ptr<Path_optimization> optimization_ptr,
			     ros::NodeHandle n) 
: _planner(planner_ptr),_workcell(workcell_ptr),_optimization(optimization_ptr), _n(n)
{
 
}

ros_interface::~ros_interface()
{

}

bool ros_interface::init()
{
  // Load parameters from launch file
  if(!_n.getParam("nbv_pose_topic",_nbv_pose_topic)){
    ROS_ERROR("Next_best_view_planner: Could not get 'nbv_pose_topic' parameter from parameter server");
    return false;
  }
   if(!_n.getParam("planning_frame",_planning_frame)){
    ROS_ERROR("Next_best_view_planner: Could not get 'planning_frame' parameter from parameter server");
    return false;
  }
  if(!_n.getParam("velocity",_vel)){
    ROS_ERROR("Next_best_view_planner: Could not get 'velocity' parameter from parameter server");
    return false;
  }
  if(!_n.getParam("acceleration",_acc)){
    ROS_ERROR("Next_best_view_planner: Could not get 'acceleration' parameter from parameter server");
    return false;
  }
  
  if(_vel <= 0) ROS_ERROR("Next_best_view_planner: Invalid 'velocity' value = %.03f", _vel);
  if(_acc <= 0) ROS_ERROR("Next_best_view_planner: Invalid 'acceleration' value = %.03f", _acc);
  
  //Subscribe to topics
    //_sub_nbv_pose = _n.subscribe(_nbv_pose_topic,10,&ros_interface::nbv_pose_CB);
  _joint_state = _n.subscribe("/ROS_Sensor/joint_states", 10,&ros_interface::joint_stateCB, this);
    
    _nbv_marker_array_publisher = _n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100);
     ROS_INFO("Next_best_view_planner: Starting Service %s/preplan", _n.getNamespace().c_str());
    _service_PrePlan = _n.advertiseService("preplan",&ros_interface::prePlan, this);
    ROS_INFO("Next_best_view_planner: Starting Service %s/move_to_next_preplanned_pose", _n.getNamespace().c_str());
    _srv_move2PrePlanPose = _n.advertiseService("move_to_next_preplanned_pose",&ros_interface::moveToNextPrePlannedPose,this);
    
    _movePublisher           = _n.advertise<robot_msgs::MoveCmd>("/ROS_Sensor/moveCmd", 1000);
    _movePublisher_buffer    = _n.advertise<robot_msgs::MoveCmd>("/ROS_Sensor/moveCmd_buffer", 1000);
    _movePublisher_bufferExe = _n.advertise<robot_msgs::MoveCmd>("/ROS_Sensor/moveCmd_bufferExe", 1000);
  
    
    _view_count = -1;
    
    return true;
}

bool ros_interface::moveToNextPrePlannedPose(robot_srvs::NBV_MoveToNextPrePlannedPose::Request &req, robot_srvs::NBV_MoveToNextPrePlannedPose::Response &res)
{

  //Moving to the next 
  if(_view_points.size() > 0){
    
     if(_view_count < int(_view_points.size())){
      _view_count++;
      }else{
	ROS_INFO("Starting from view point 1 again!");
      _view_count = -1;
     
      }
      
    rw::math::Transform3D<> next = _view_points[_view_count]; 
    ROS_INFO("Moving to view point %d", _view_count);
    res.current_viewPoint = _view_count;
    
  rw::math::Q initQ(6, 0.0); 
  std::vector<rw::math::Q> solution;
  rw::trajectory::QPath path;
  if(_workcell->solve_invkin(next,false,initQ,solution, true, true))
  {
     ROS_INFO("Planning.....");
     std::cout << "Q_start: " << _currentQ << std::endl;
     std::cout << "Q_goal: " << solution[0] << std::endl;
    if(_planner->plan(_currentQ,solution[0],path)){
    
      _optimization->optimize(path);
      if(moveTrajectory(path)){
          res.status = true;
      }else res.status = false;
    
    }else
      ROS_ERROR("Could not query a path from the path planner!");
  }else{
    ROS_ERROR("Could not find a inverse kinematic solution!");
    res.status = false;
  }
    
  }else
    res.status = false;
  
 return true;

}

bool ros_interface::moveTrajectory(rw::trajectory::QPath path)
{
 robot_msgs::MoveCmd moveCmd;
 moveCmd.moveType = MOVE_J;
 moveCmd.vel = _vel;
 moveCmd.acc = _acc;

  std::cout << "===========PATH=============" << std::endl;
 for(unsigned int j=0; j<path.size(); j++){
   rw::math::Q q = path[j];
   std::cout << q << std::endl;
    for(unsigned int i=0; i<6; i++){
      
	moveCmd.joint.q.push_back(q[i]);
    }
 
 }
  std::cout << "===========FINISH=============" << std::endl;
 _movePublisher_buffer.publish(moveCmd);
 return true;
  
}
bool ros_interface::moveJ(rw::math::Q q)
{
  unsigned int _cmdId = 1;
  _workcell->setQ(_currentQ);
  _workcell->test_moveJ(_currentQ, q);
  
  robot_msgs::MoveCmd moveCmd;
  moveCmd.cmdId = _cmdId;
  moveCmd.moveType = MOVE_J;
  moveCmd.vel = _vel;
  moveCmd.acc = _acc;
  for(unsigned int j=0; j<q.size(); j++)
    moveCmd.joint.q.push_back(q[j]);
   
  //std::cout << "Moving to: " << q << std::endl;
 _movePublisher.publish(moveCmd);
}

bool ros_interface::moveL(rw::math::Transform3D<> T)
{
  rw::math::Q initQ(6, 0.0); 
  std::vector<rw::math::Q> solution;
  rw::math::Q q;
  if(_workcell->solve_invkin(T,false,initQ,solution, true, true))
  {			
    robot_msgs::MoveCmd moveCmd;
    moveCmd.cmdId = 1;
    moveCmd.moveType = MOVE_L;
    moveCmd.vel = _vel;
    moveCmd.acc = _acc;
    q = solution[0];
    for(unsigned int i=0; i<q.size(); i++)
      moveCmd.joint.q.push_back(q[i]);
 
    //std::cout << "Moving to: " << q << std::endl;
    _movePublisher.publish(moveCmd);
    return true;
  }else{
    ROS_ERROR("No inverse kinematic solution!");
    return false;
  }
  
}

void ros_interface::joint_stateCB(const sensor_msgs::JointState::Ptr& msg)
{
  rw::math::Q cur(msg->position.size());
  for(unsigned int j=0; j<msg->position.size(); j++)
     	cur[j]=msg->position[j];

  _currentQ = cur;

}
bool ros_interface::prePlan(robot_srvs::PrePlanViewPoints::Request &req, robot_srvs::PrePlanViewPoints::Response &res)
{  
  _view_points.clear();
 
  rw::math::Vector3D<> v(req.object_orgin.position.x,
			 req.object_orgin.position.y,
			 req.object_orgin.position.z);

  rw::math::Quaternion<> q(req.object_orgin.orientation.x, req.object_orgin.orientation.y, 
			    req.object_orgin.orientation.z, req.object_orgin.orientation.w);

  rw::math::Rotation3D<> r = q.toRotation3D();
  rw::math::Transform3D<> T(v,r);
  _planner->sample_sphere(T, _view_points);
  //_planner->computeShortestPath(_view_points);
 // _planner->sample_sphere_random(T, _view_points);
  res.num_view_points = int(_view_points.size());
  
  publish_nbv_marker(_view_points);
  
  return true;
}
bool ros_interface::nbv_pose_CB(const geometry_msgs::PoseArray::Ptr &msg)
{
 
  ROS_INFO_STREAM(msg->poses.size() << " next best view candidates received!!");
  
 
 
  
  return true;
}
bool ros_interface::publish_nbv_marker(const std::vector<rw::math::Transform3D<> > &pose_list )
{
  using namespace rw::math;
  
  static unsigned int lastsize = 0;
  unsigned int size = pose_list.size();
  
   ros::Duration d(50.0);
  
   //std::cout << "list size: " << pose_list.size() << std::endl;
   
   if (size >= lastsize) {
    _nbv_marker_array_msg.markers.resize(size);
  }
   
   for(int i = 0; i<= pose_list.size()-1; i++){

      Transform3D<> tr = pose_list.at(i);
      Vector3D<> T = tr.P();
      Rotation3D<> R = tr.R();
      
      geometry_msgs::Point pos;
      pos.x = T[0];
      pos.y = T[1];
      pos.z = T[2];
      
      Quaternion<> quat(R);
     // quat.normalize();
      geometry_msgs::Quaternion quat_msg;
      quat_msg.w = quat.getQw();
      quat_msg.x = quat.getQx();
      quat_msg.y = quat.getQy();
      quat_msg.z = quat.getQz();
      
     //_nbv_marker_array_msg.markers[i].header.pose = p;
      
     _nbv_marker_array_msg.markers[i].pose.position = pos;
     _nbv_marker_array_msg.markers[i].pose.orientation = quat_msg;
     _nbv_marker_array_msg.markers[i].header.frame_id = "RobotBase";
     _nbv_marker_array_msg.markers[i].header.stamp = ros::Time::now();
     _nbv_marker_array_msg.markers[i].ns = "nbv_views";
     _nbv_marker_array_msg.markers[i].color.r = 0.0f;
     _nbv_marker_array_msg.markers[i].color.g = 0.0f;
     _nbv_marker_array_msg.markers[i].color.b = 1.0f;
     _nbv_marker_array_msg.markers[i].color.a = 0.5f;
     _nbv_marker_array_msg.markers[i].scale.x = 0.01;
     _nbv_marker_array_msg.markers[i].scale.y = 0.01;
     _nbv_marker_array_msg.markers[i].scale.z = 0.01;
     _nbv_marker_array_msg.markers[i].id = i;
     _nbv_marker_array_msg.markers[i].lifetime = d; //ros::Duration::Duration();
     _nbv_marker_array_msg.markers[i].type = visualization_msgs::Marker::ARROW;
     
     _nbv_marker_array_msg.markers[i].action = visualization_msgs::Marker::ADD;
  
    }
    
  if (lastsize > size) {
    for (unsigned int i = size; i < lastsize; ++i) {
      _nbv_marker_array_msg.markers[i].action = visualization_msgs::Marker::DELETE;
    }
  }
  lastsize = size;
 
  _nbv_marker_array_publisher.publish(_nbv_marker_array_msg);
   //ROS_INFO("next_best_view_planner: Published marker array with view points!");
  
  return true;
}

}
