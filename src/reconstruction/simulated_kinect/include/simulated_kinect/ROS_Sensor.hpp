/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#ifndef ROS_SENSOR_HPP
#define ROS_SENSOR_HPP

#include "SensorView.hpp"

#include <rws/RobWorkStudioPlugin.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/common/Message.hpp>
#include <rw/trajectory/Path.hpp>
#include <rw/sensor/Scanner25D.hpp>
#include <rw/sensor/Scanner2D.hpp>
#include <rw/sensor/Scanner1D.hpp>
#include <rw/sensor/Camera.hpp>

#include <rwlibs/simulation/SimulatedSensor.hpp>
#include <rwlibs/simulation/SimulatedKinnect.hpp>

#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rwlibs/simulation/GLFrameGrabber25D.hpp>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "robot_msgs/MoveCmd.h"
#include "robot_msgs/CommandFinish.h"
#include "robot_msgs/ResetCmd.h"
#include "robot_msgs/CommandFinish.h"
#include "robot_msgs/DigitalIO.h"
#include "robot_msgs/JointCurrent.h"
#include "robot_msgs/Ur5Mode.h"
#include "std_msgs/String.h"
//#include "tf/transform_broadcaster.h"

#include <rw/math/Q.hpp>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/publisher.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>

#include <QThread>

#include <omp.h>

#include "simulated_kinect/ui_SensorsPlugin.h"
#include "simulated_kinect/RosInterface.h"



namespace rws {

typedef pcl::PointXYZRGBA PointT;
  
class ROSCommonNode
{
protected:
  ROSCommonNode()
  {
    int argc;
    char** argv;
    ros::init(argc,argv,"ROS_Sensor");
  }
  
  
};

class ROS_Sensor : public RobWorkStudioPlugin, private Ui::SensorsPlugin, public ROSCommonNode
{
   Q_OBJECT
    Q_INTERFACES(  rws::RobWorkStudioPlugin )

public:
    ROS_Sensor();

    virtual ~ROS_Sensor();

    void initialize();

    void open(rw::models::WorkCell* workcell);

    void close();

    //void setupToolBar(QToolBar* toolbar);
public Q_SLOTS:
    void updateSim();

private Q_SLOTS:
    void on_btnDisplay_clicked();
  //  void on_spnUpdateTime_valueChanged(int value);
    void on_btnSave_clicked();
    void viewClosed(SensorView* view);
    
    void moveCallback(const robot_msgs::MoveCmd::ConstPtr& msg);
    void moveBufferCallback(const robot_msgs::MoveCmd::ConstPtr& msg);
    void createPointCloud(bool save_pointCloud, bool save_depthMap = false);
private:
    // This listens for changes to the state of RobWorkStudio.
    void stateChangedListener(const rw::kinematics::State& state);

private:
    QTimer *_timer;
    QComboBox *cmbSensors;
    QThread* _rosThread;
    simulated_kinect::RosInterface* _rosInterface;
    

    rw::kinematics::State _state;
    rw::kinematics::State _initstate;
    rw::models::WorkCell* _workcell;
    rw::common::Ptr<rw::models::Device> _robot;

    struct SensorSet {
    public:
        SensorSet(rwlibs::simulation::SimulatedSensor::Ptr sensor, SensorView::Ptr view, std::string frame_id):
            sensor(sensor), view(view), frame_id(frame_id) 
        {}

        rwlibs::simulation::SimulatedSensor::Ptr sensor;
        SensorView::Ptr view;
	std::string frame_id;
  };
  
    std::vector<SensorSet> _sensors;

    ros::NodeHandle _n;
    std::vector<ros::Subscriber> _subscribers;
    ros::Publisher _pubDigInput;
    ros::Publisher _pubDigOutput;
    ros::Publisher _pubJointCurrent;
    ros::Publisher _jointPublisher;
    ros::Publisher _pubRobotMode;
    ros::Publisher _pubCmdFinish;
    ros::Publisher _joint_state_publisher;
    ros::Publisher _pubDiagnostic;
    pcl_ros::Publisher<PointT> _pointCloudPub;
    
    ros::Subscriber _subMoveCmd;
    
    // need tactile switch sensor
    rwlibs::simulation::GLFrameGrabber::Ptr _framegrabber;
    rwlibs::simulation::GLFrameGrabber25D::Ptr _framegrabber25d;
    rwlibs::simulation::SimulatedScanner25D* _simscan25;
   // rwlibs::simulation::SimulatedKinnect* _simKinect;
    rw::graphics::SceneViewer::Ptr _gldrawer;

};

}

#endif /* ROS_SENSOR_HPP */
