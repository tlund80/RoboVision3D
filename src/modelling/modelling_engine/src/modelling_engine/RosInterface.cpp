/*
 * RosInterface.cpp
 *
 *  Created on: Jul 26, 2013
 *      Author: thomas
 */

#include "Modeling_Engine/RosInterface.h"

namespace perception_3D {

RosInterface::RosInterface(boost::shared_ptr<SharedData> data_ptr,ros::NodeHandle n) : _sharedData(data_ptr), _n(n){
	// TODO Auto-generated constructor stub

	//_sharedData = data_ptr;

	setScanningState(4);
	
	// Load parameters from launch file
	_robotFlangeTopic ="/ur5/tcp";
	_handeye_calib_path = "/home/thomas/dti_co_worker/trunk/dti_co_worker/perception_3D/data/co-worker-mini_hand_eye.yml";
	//_n.param<std::string>("robotFlangeTopic", _robotFlangeTopic, "");
	    //_n.param<std::string>("pointCloudTopic", _pointCloudTopic, "");
	//_n.param<std::string>("handeye_calib_path", _handeye_calib_path, "");

	//Publisher
	_pubRegisteredModel = pcl_ros::Publisher<pcl::PointXYZRGB> (_n, "registeredmodel", 1);
	_pubLastPointCloud = pcl_ros::Publisher<pcl::PointXYZRGB> (_n, "lastPointCloud", 1);
	_posePub =  _n.advertise<robot_msgs::ResultPose>( "/perception/resultPose",1);

	//Subscriber
	_robotFlangSub = _n.subscribe(_robotFlangeTopic,1, &RosInterface::robotFlangeCallback, this);

	//Subscribe to Snap topic
	_controlSub = _n.subscribe("/perception/detectObject", 100, &RosInterface::ControlCallback, this);

	//Action client
	ac = new actionlib::SimpleActionClient<perception_3D::ScanAction>(_n,"/perception_3D/dti_structured_light", true);

	_clearModelService = _n.advertiseService("clearModel",&RosInterface::clearModelService,this);
	ROS_INFO("3D Modeling Engine: Service 'modeling_engine/clearModel'is started! ");

	//Load Hand eye calibration
	ROS_INFO("3D Modeling Engine: Loading Hand_eye transform from %s!", _handeye_calib_path.c_str());
	cv::FileStorage fs2(_handeye_calib_path, cv::FileStorage::READ);
	cv::Mat ha_transform;
	fs2["hand_eye_transform"] >> ha_transform;

	Eigen::Matrix4f ha;
	ha(0,0) = ha_transform.at<double>(0); ha(0,1) = ha_transform.at<double>(1); ha(0,2) = ha_transform.at<double>(2); ha(0,3) = ha_transform.at<double>(3);
	ha(1,0) = ha_transform.at<double>(4); ha(1,1) = ha_transform.at<double>(5); ha(1,2) = ha_transform.at<double>(6); ha(1,3) = ha_transform.at<double>(7);
	ha(2,0) = ha_transform.at<double>(8); ha(2,1) = ha_transform.at<double>(9); ha(2,2) = ha_transform.at<double>(10); ha(2,3) = ha_transform.at<double>(11);
	ha(3,0) = ha_transform.at<double>(12); ha(3,1) = ha_transform.at<double>(13); ha(3,2) = ha_transform.at<double>(14); ha(3,3) = ha_transform.at<double>(15);

/*	Eigen::Matrix3d ha1;
	ha1(0,0) = ha_transform.at<double>(0); ha1(0,1) = ha_transform.at<double>(1); ha1(0,2) = ha_transform.at<double>(2);
	ha1(1,0) = ha_transform.at<double>(4); ha1(1,1) = ha_transform.at<double>(5); ha1(1,2) = ha_transform.at<double>(6);
	ha1(2,0) = ha_transform.at<double>(8); ha1(2,1) = ha_transform.at<double>(9); ha1(2,2) = ha_transform.at<double>(10);


	Eigen::Vector3d rpy1;
	tiv::ti_mat2rpy(ha1,rpy1);
	std::cout << rpy1 << std::endl;
	
*/
	_sharedData->setHandEyeTransform(ha);
	//_sharedData->setHandEyeTransform(Eigen::Matrix4f::Identity ());
	 GlobalTransform = Eigen::Matrix4f::Identity ();

	count = 0;

	ROS_INFO("3D Modeling Engine: End initialization! ");
}

RosInterface::~RosInterface() {
	// TODO Auto-generated destructor stub
	ROS_INFO("3D Modeling Engine: Closing ROS Interface!");
	ac->cancelAllGoals();
	delete ac;
}

void RosInterface::ControlCallback(const robot_msgs::PerceptionMsg::ConstPtr& msg)
{
	ROS_INFO("New snap shot function called");

//	tiv::pose po = robot_poses.at(count);
//	Eigen::Matrix3d R = po.q.toMatrix();
//	Eigen::Vector3d rpy;
//	tiv::ti_mat2rpy(R,rpy);

	robot_msgs::PerceptionMsg m;
	robot_msgs::ResultPose result;

	takeScan();

	result.cmdId = msg->cmdId;
	result.pose.x = -0.100;
	result.pose.y = 0.530;
	result.pose.z = 0.415;
	result.pose.roll = -1.492;
	result.pose.pitch = -0.119;
	result.pose.yaw = -2.796;
	result.status = true;
	result.confirmed = false;
	_posePub.publish(result);

/*
	result.cmdId = msg->cmdId;
	result.pose.x = po.t.x;
	result.pose.y = po.t.y;
	result.pose.z = po.t.z;
	result.pose.roll = rpy[0];
	result.pose.pitch = rpy[1];
	result.pose.yaw = rpy[2];
	result.status = true;
	result.confirmed = false;

	if(count == (int)robot_poses.size()-1)
	{
		result.confirmed = true;
		count = 0;
	}
	_posePub.publish(result);
	count++;
*/

	//////////////////////////////////////////////////////
	// 1. Get current view point 					//////
	/////////////////////////////////////////////////////



	//////////////////////////////////////////////////////
	// 2. Take snap shot by calling Perception 3D Node	//////
	/////////////////////////////////////////////////////




	//////////////////////////////////////////////////////
	// 2. Take snap shot by calling Perception 3D Node	//////
	/////////////////////////////////////////////////////

}

void RosInterface::robotFlangeCallback(const robot_msgs::Pose::ConstPtr msg)
{
	//Convert from meters to millimeters
	tiv::pose p;

	p.t.x =  msg->x *1000;
	p.t.y =  msg->y  *1000;
	p.t.z =  msg->z *1000;

	Eigen::Vector3d rpy;
	rpy(0) = msg->roll;
	rpy(1) = msg->pitch;
	rpy(2) = msg->yaw;

	Eigen::Matrix3d R;
	tiv::ti_rpy2mat(rpy,R);
	p.q.fromMatrix(R);
	
	Eigen::Matrix4d mat = p.toMatrix();
	
	//Cast to float 
	Eigen::Matrix4f trans(mat.cast<float>());
	//ROS_INFO("robotFlangeCallback setting");
	//_sharedData->setRobotFlangePose(trans);
	//ROS_INFO("robotFlangeCallback finish");


}

void RosInterface::pubRegisteredModel(pcl::PointCloud<pcl::PointXYZRGB> pc)
{
	_pubRegisteredModel.publish(pc);

}

void RosInterface::pubLastPointCloud(pcl::PointCloud<pcl::PointXYZRGB> pc)
{
	_pubLastPointCloud.publish(pc);

}

bool RosInterface::clearModelService(perception_3D::ClearModel::Request& req, perception_3D::ClearModel::Response& res)
{
	_sharedData->clearPointcloudList();
	count = 0;

	return true;
}

void RosInterface::scanDoneCB(const actionlib::SimpleClientGoalState& state,const ScanResultConstPtr& result)
{
	ROS_INFO("Action finished: %s",state.toString().c_str());


	tf::StampedTransform transform;
	listener.waitForTransform("/RobotBase", "/sensor_frame",ros::Time(0),ros::Duration(5));
	
	try{
	    listener.lookupTransform("/RobotBase", "/sensor_frame", ros::Time(0), transform);
	}catch (tf::TransformException& ex){
	     ROS_ERROR("%s",ex.what());
	}

	tf::Quaternion qr =  transform.getRotation();
	tf::Vector3 tr =  transform.getOrigin();
	std::cout << "x: " << tr.getX() << " y: " << tr.getY() << " z: " << tr.getZ() << std::endl;
	std::cout << "w:" << qr.getW() << " x: " << qr.getX() << " y: " << qr.getY() << " z: " << qr.getZ() << std::endl;

	tiv::quat q;	q.w =  (double)qr.getW();	q.x = (double)qr.getX();	q.y = (double)qr.getY();	q.z = (double)qr.getZ();
	tiv::pt3d p;	p.x = (double)tr.getX();	p.y = (double)tr.getY();	p.z = (double)tr.getZ();

	tiv::pose pose;
	pose.q = q;
	pose.t = p;

	Eigen::Matrix4d _wp = pose.toMatrix();
	Eigen::Matrix4f _wpf(_wp.cast<float>());
	_sharedData->setViewPoint(_wpf);
	
	Eigen::Matrix4d _wp_inv = _wp.inverse();
	Eigen::Matrix4f trans(_wp_inv.cast<float>());

	Eigen::Matrix4f vl; _sharedData->getLastViewPoint(vl);
	//Create new scan data object
	boost::shared_ptr<perception_3D::ScanData> _scan_data;
	_scan_data.reset(new perception_3D::ScanData);

	//Eigen::Matrix4f v; _sharedData->getViewPoint(v);
	//Eigen::Matrix4f vl; _sharedData->getLastViewPoint(vl);
	//Eigen::Matrix4f vp_inv = v.inverse();
	
//	std::cout << "v: \n" <<  v << std::endl;
	
//	std::cout << "vp_inv: \n" <<  vp_inv << std::endl;
	
		//Compute transformation between the views
	Eigen::Matrix4f rob_transform = vl.inverse() * _wpf;
	
	if(_sharedData->getPointcloudListLength() == 2)
	{ //Second scan
		ROS_INFO("Second scan");
		_sharedData->setDifTransform(rob_transform);

	}else if(_sharedData->getPointcloudListLength() > 2)
	{
		Eigen::Matrix4f dif; _sharedData->getDifTransfrom(dif);
		Eigen::Matrix4f dif_transform =rob_transform.inverse() * dif;

		std::cout << "Transform: \n"
				  <<  dif_transform << std::endl;

		_sharedData->setTransform(dif_transform);
	}else
	{	//First scan transfrom is just the current robot position
		_sharedData->setDifTransform(rob_transform);

	}

    Eigen::Vector3d lrpy;
    Eigen::Matrix4d d_vl(vl.cast<double>());
    tiv::pose pu;	pu.fromMatrix(d_vl);
    tiv::ti_mat2rpy(pu.q.toMatrix(),lrpy);
    
	std::cout << "Last view point: \n"
	   	     << "\tx: " << pu.t.x
			 << "\ty: " << pu.t.y
			 << "\tz: " << pu.t.z
			 << "\tR: " << lrpy[0]
			 << "\tP: " << lrpy[1]
			 << "\tY: " << lrpy[2] << std::endl;

   Eigen::Vector3d rpy;
    Eigen::Matrix4d d_v(_wpf.cast<double>());
    tiv::pose pu1;	pu1.fromMatrix(d_v);
    tiv::ti_mat2rpy(pu1.q.toMatrix(),rpy);


   std::cout << "Current view point: \n"
	     	  << "\tx: " << pu1.t.x
		   	  << "\ty: " << pu1.t.y
		   	  << "\tz: " << pu1.t.z
		   	  << "\tR: " << rpy[0]
		   	  << "\tP: " << rpy[1]
		   	  << "\tY: " << rpy[2] << std::endl;
	
				  
	pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
	pcl::PointCloud<pcl::PointXYZRGB> transformed_cloud;


	if(result->success == true){

	   sensor_msgs::PointCloud2 pc = result->pc;
	   pcl::fromROSMsg(pc, pointCloud);

	   //Relate point cloud to world coordinate system vp_inv
	   pcl::transformPointCloud(pointCloud, transformed_cloud,trans);

	    _scan_data->setPointCloud(pointCloud.makeShared());
	   //_scan_data->setPointCloud(transformed_cloud.makeShared());
	   _scan_data->setReconstructionTime(result->scan_time);
	   _scan_data->setScanId(result->cloud_id);
	   _scan_data->setScanTime(result->scan_request_received);
	   _scan_data->setViewPoint(_wpf);
	   _scan_data->setTransform(rob_transform);
	   
	   stringstream str;
	   str << "/home/thomas/leap_box/p_";
	   str << count;
	   str << ".pcd";
	   string s = str.str();

	   DTI_INFO("3D Modeling Engine: Saving cloud at " << s);
	   pcl::io::savePCDFile(s,transformed_cloud);
	   count++;
	   
	  //  _pubLastPointCloud.publish(transformed_cloud);
	   
	   if(!_sharedData) ROS_ERROR("Shared pointer is not existing any more!!");
	   else _sharedData->addNewPointcloud2List(_scan_data);
  

	}else
	{
	        ROS_ERROR("3D Modeling Engine: Scanning failed!!");
	}
	


   //Store this viewpoint
   _sharedData->setLastViewPoint(_wpf);

    setScanningState(0);
	
}

void RosInterface::scanActiveCb(void)
{
	ROS_INFO("Goal just went active");


}

void RosInterface::scanfeedbackCb(const ScanFeedbackConstPtr& feedback)
{
	//ROS_INFO("3D Modeling Engine: progress %d", feedback->progress);

}

bool RosInterface::takeScan( void)
{
	ROS_INFO("3D Modeling Engine: Waiting for scan action server to start.");
	// wait for the action server to start
	ac->waitForServer(); //will wait for infinite time

	if(ac->isServerConnected()){

	  ROS_INFO("3D Modeling Engine: Scan action server started, sending goal.");

	  // send a goal to the action
	  perception_3D::ScanGoal goal;
	  perception_3D::ScanFeedback feedback;
	  perception_3D::ScanActionResult result;

	  goal.scan = "take scan";
	  //ac->sendGoal(goal);
	  ac->sendGoal(goal,
			boost::bind(&RosInterface::scanDoneCB, this,_1,_2),
			boost::bind(&RosInterface::scanActiveCb,this),
			boost::bind(&RosInterface::scanfeedbackCb, this,_1));


	  //wait for the action to return
	  bool finished_before_timeout = ac->waitForResult(ros::Duration(30.0));

	  if (!finished_before_timeout)
	  {
		ROS_INFO("3D Modeling Engine: Action client time out!!!");
		return false;
	  }
	  return true;
	}else
	  return false;
}


} /* namespace perception */
