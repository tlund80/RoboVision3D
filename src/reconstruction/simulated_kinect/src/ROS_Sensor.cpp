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

#include "simulated_kinect/ROS_Sensor.hpp"

#include <rws/RobWorkStudio.hpp>

#include <sstream>

#include <boost/foreach.hpp>

#include <rw/sensor.hpp>
#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Quaternion.hpp>

#include <rwlibs/simulation/SimulatedCamera.hpp>
#include <rwlibs/simulation/SimulatedScanner25D.hpp>
#include <rwlibs/simulation/SimulatedScanner2D.hpp>
#include <rwlibs/opengl/RenderScan.hpp>
#include <rw/geometry/PointCloud.hpp>
//#include <rwlibs/simulation/SimulatedScanner1D.hpp>


#include <boost/foreach.hpp>
#include <QMessageBox>

#include <sstream>

using namespace rw::math;
using namespace rw::common;
using namespace rw::sensor;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rwlibs::simulation;
using namespace rw::graphics;
using namespace rws;
using namespace rwlibs::opengl;



ROS_Sensor::ROS_Sensor():
    RobWorkStudioPlugin("ROS_Sensor", QIcon(":/sensors.png"))
{
    setupUi(this);

    QWidget *widget = new QWidget(this);
    QVBoxLayout *lay = new QVBoxLayout(widget);
    widget->setLayout(lay);
    this->setWidget(widget);
    
     {
       cmbSensors = new QComboBox();
       lay->addWidget(cmbSensors);
        
    }
    {
        QSpinBox* spnUpdateTime = new QSpinBox();
        spnUpdateTime->setValue(100);
	lay->addWidget(spnUpdateTime); // Own button.
	
   //     connect(spnUpdateTime, SIGNAL(valueChanged(int)), this, SLOT(on_spnUpdateTime_valueChanged()));
    }

    {
        QPushButton* button = new QPushButton("Display");
        lay->addWidget(button); // Own button.
        connect(button, SIGNAL(pressed()), this, SLOT(on_btnDisplay_clicked()));
    }
    
    {
        QPushButton* button = new QPushButton("Save Image");
        lay->addWidget(button); // Own button.
        connect(button, SIGNAL(pressed()), this, SLOT(on_btnSave_clicked()));
    }

   {
      QCheckBox* chk = new QCheckBox("Publish point cloud");
      lay->addWidget(chk);
      chk->setEnabled(true);
   }

   qRegisterMetaType<robot_msgs::MoveCmd::ConstPtr>("robot_msgs::MoveCmd::ConstPtr");
   
   //_rosThread = new QThread();
   _rosInterface = new simulated_kinect::RosInterface();
   
    connect(_rosInterface, SIGNAL(moveCmdCalled(robot_msgs::MoveCmd::ConstPtr)), this, SLOT(moveCallback(robot_msgs::MoveCmd::ConstPtr)));
    connect(_rosInterface, SIGNAL(moveCmd_buffer_Called(robot_msgs::MoveCmd::ConstPtr)), this, SLOT(moveBufferCallback(robot_msgs::MoveCmd::ConstPtr)));
    connect(_rosInterface, SIGNAL(getScan(bool)), this, SLOT(createPointCloud(bool)));
   
   _rosInterface->start();
   
   //_rosInterface->moveToThread(_rosThread);
   //_rosThread->start(QThread::HighPriority);

   _timer = new QTimer(this);
    _timer->setSingleShot(false);
    _timer->setInterval(10);//(this->spnUpdateTime->value());
    connect(_timer, SIGNAL(timeout()), this, SLOT(updateSim()));
    _timer->start();
    
}

ROS_Sensor::~ROS_Sensor()
{
    BOOST_FOREACH(SensorSet& set, _sensors) {
        set.view->close();
    }
    _sensors.clear();  
}

void ROS_Sensor::initialize()
{
    getRobWorkStudio()->stateChangedEvent().add(
    boost::bind(&ROS_Sensor::stateChangedListener,this,_1), this);
   
}

void ROS_Sensor::updateSim(){
  
  /*  BOOST_FOREACH(SensorSet& set, _sensors) {
        //getRobWorkStudio()->getView()->makeCurrent(); TODO
        Simulator::UpdateInfo info(0.001*100);//(0.001*spnUpdateTime->value());
        set.view->makeCurrent();
        set.sensor->update(info, _state);
        set.view->update();
    }
    */
  //Publish current joint configuration
   Q Joints = _robot->getQ(_state);
   _rosInterface->publishJoint(Joints); 
}

void ROS_Sensor::stateChangedListener(const State& state)
{
    _state = state;
}

void ROS_Sensor::open(WorkCell* workcell)
{
  
    if (workcell == NULL)
	return;
  
    _workcell = workcell;
    _sensors.clear();

    _state = getRobWorkStudio()->getState();
    _initstate = getRobWorkStudio()->getState();
    _robot = _workcell->findDevice("EasyBot");
    if(!_robot){
	  std::cout << "Can't find device: '" << "EasyBot" << "'" << std::endl; //
	  return;
    }
	  
    std::vector<Frame*> frames = Kinematics::findAllFrames(workcell->getWorldFrame(), workcell->getDefaultState());

    cmbSensors->clear();
    
    std::cout << "Initializing framegrabbers ....." << std::endl;
    
    _gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
    SensorView* view = NULL;
    SimulatedSensor* sensor = NULL;
    std::string _f_id;
	
    BOOST_FOREACH(Frame* frame, frames) {
        if (frame->getPropertyMap().has("Camera")) {
	   //Find all cameras in the scene
            cmbSensors->addItem(QString("%1:%2").arg("Camera").arg(frame->getName().c_str()), QVariant(frame->getName().c_str()));
         }
        if (frame->getPropertyMap().has("Scanner25D")) {
	    //Find all 2.5D scanners in the scene
            cmbSensors->addItem(QString("%1:%2").arg("Scanner25D").arg(frame->getName().c_str()),  QVariant(frame->getName().c_str()));
	    //Add Point cloud publisher!!
	    _f_id = frame->getName();
	    _rosInterface->addPointCloudPublisher(_f_id);
	    
	     Scan25DView* scanview = new Scan25DView();
	     double fovy; int width,height;
	     std::string camId("Scanner25D");
	     std::string camParam = frame->getPropertyMap().get<std::string>(camId);
	     std::istringstream iss (camParam, std::istringstream::in);
	     iss >> fovy >> width >> height;
	     
	     _framegrabber25d = ownedPtr( new GLFrameGrabber25D(width, height,fovy) );
             _framegrabber25d->init(_gldrawer);
	     _simscan25 = new SimulatedScanner25D("SimulatedScanner25D", frame, _framegrabber25d);
	      sensor = _simscan25;
	     _simscan25->open();
	  
	     getRobWorkStudio()->getWorkCellScene()->addRender("scanrender", ownedPtr(new RenderScan(_simscan25->getScanner25DSensor())),frame);
	     scanview->initialize(_simscan25->getScanner25DSensor());
	     view = scanview;
  
        }
        if (frame->getPropertyMap().has("Scanner2D")) {
            cmbSensors->addItem(QString("%1:%2").arg("Scanner2D").arg(frame->getName().c_str()),  QVariant(frame->getName().c_str()));
        }
        
           if (sensor != NULL && view != NULL) {
        _sensors.push_back(SensorSet(sensor, view,_f_id));
	connect(view, SIGNAL(viewClosed(SensorView*)), this, SLOT(viewClosed(SensorView*)));
    }
    }
    
        std::cout << "Finish ....." << std::endl;
    // Define _init state
    Q q(6);
    q[0] = 180.0 * Deg2Rad;
    q[1] = -90.0 * Deg2Rad;
    q[2] = -90.0 * Deg2Rad;
    q[3] = -90.0 * Deg2Rad;
    q[4] =  90.0 * Deg2Rad;
    q[5] =  90.0 * Deg2Rad;
    _robot->setQ(q, _initstate);
    getRobWorkStudio()->setState(_initstate);
    stateChangedListener(getRobWorkStudio()->getState());
}

void ROS_Sensor::close()
{}

void ROS_Sensor::on_btnDisplay_clicked() {
    std::string frameName = cmbSensors->itemData(cmbSensors->currentIndex()).toString().toStdString();
    QStringList strings = cmbSensors->currentText().split(":");
    if (strings.size()<=1) {
        QMessageBox::critical(this, tr("Sensors"), tr("Unable to split string"));
    }
    std::string sensorName = strings[0].toStdString();
   // std::cout<<"sensorName "<< sensorName <<std::endl;
    Frame* frame = _workcell->findFrame(frameName);
    if (frame == NULL) {
        QMessageBox::critical(this, tr("Sensors"), tr("Unable to find frame %1").arg(frameName.c_str()));
        return;
    }

    SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
    SimulatedSensor* sensor = NULL;
    SensorView* view = NULL;

    if (sensorName == "Camera" && frame->getPropertyMap().has("Camera")) {
        double fovy;
        int width,height;
        std::string camId("Camera");
        std::string camParam = frame->getPropertyMap().get<std::string>(camId);
        std::istringstream iss (camParam, std::istringstream::in);
        iss >> fovy >> width >> height;
        //getRobWorkStudio()->getView()->makeCurrentContext(); TODO
        GLFrameGrabber::Ptr framegrabber = ownedPtr( new GLFrameGrabber(width,height,fovy) );
        framegrabber->init(gldrawer);
        SimulatedCamera *simcam = new SimulatedCamera("SimulatedCamera", frame, framegrabber);
        sensor = simcam;
        simcam->initialize();
        simcam->start();
        view = new CameraView(simcam->getCameraSensor(), NULL);


     }
    else if (sensorName == "Scanner25D" && frame->getPropertyMap().has("Scanner25D")) {
        double fovy;
        int width,height;
        std::string camId("Scanner25D");
        std::string camParam = frame->getPropertyMap().get<std::string>(camId);
        std::istringstream iss (camParam, std::istringstream::in);
        iss >> fovy >> width >> height;

        Scan25DView* scanview = new Scan25DView();
        //scanview->makeCurrent();
        _framegrabber25d = ownedPtr( new GLFrameGrabber25D(width, height,fovy) );
        _framegrabber25d->init(gldrawer);
        SimulatedScanner25D* simscan25 = new SimulatedScanner25D("SimulatedScanner25D", frame, _framegrabber25d);
        sensor = simscan25;

        simscan25->open();

        getRobWorkStudio()->getWorkCellScene()->addRender("scanrender", ownedPtr(new RenderScan(simscan25->getScanner25DSensor())),frame);
        scanview->initialize(simscan25->getScanner25DSensor());
        view = scanview;
        view->resize(512,512);
    }


    else if (sensorName == "Scanner2D" && frame->getPropertyMap().has("Scanner2D")) {
        double fovy;
        int cnt;
        std::string camId("Scanner2D");
        std::string camParam = frame->getPropertyMap().get<std::string>(camId);
        std::istringstream iss (camParam, std::istringstream::in);
        iss >> fovy >> cnt;

        Scan2DView* scanview = new Scan2DView();
        scanview->makeCurrent();
        GLFrameGrabber25D::Ptr framegrabber25d = ownedPtr( new GLFrameGrabber25D(1, cnt,fovy) );
        framegrabber25d->init(gldrawer);
        SimulatedScanner2D* simscan2D = new SimulatedScanner2D("SimulatedScanner2D", frame, framegrabber25d);
        sensor = simscan2D;
        simscan2D->open();

        RenderScan::Ptr scanRender = ownedPtr( new RenderScan() );
        DrawableNode::Ptr node = getRobWorkStudio()->getWorkCellScene()->addRender("Scan25DView", scanRender, frame);

        scanview->initialize(simscan2D);
        view = scanview;
        view->resize(512,512);
    }

    if (sensor != NULL && view != NULL) {
       // _sensors.push_back(SensorSet(sensor, view));
       // connect(view, SIGNAL(viewClosed(SensorView*)), this, SLOT(viewClosed(SensorView*)));
      //  view->show();
    } else {
        QMessageBox::critical(this, tr("Sensors"), tr("Failed to create sensor and view"));
        if (sensor != NULL)
            delete sensor;
        if (view != NULL)
            delete view;
    }

}


void ROS_Sensor::on_btnSave_clicked(){
 
  createPointCloud(false);
}

void ROS_Sensor::createPointCloud(bool save_pointCloud, bool save_depthMap)
{
  _rosInterface->setActionServerProgress(0);
  _rosInterface->setScanState(simulated_kinect::STARTED);
  //Update each sensor view
    BOOST_FOREACH(SensorSet& set, _sensors) {
        Simulator::UpdateInfo info(0.001*100);
        set.view->makeCurrent();
        set.sensor->update(info, _state);
        set.view->update();
    }
    
   const Image25D& img25D = _framegrabber25d->getImage();
   const Image::Ptr img2D = img25D.asImage();
   if(save_depthMap)img2D->saveAsPGM("RW_DEPTH");
   
   rw::geometry::PointCloud pcloud(img25D.getWidth(),img25D.getHeight());
   pcloud.getData() = img25D.getData();
    
   //delete the infinity plane
    for(int i=0;i<(int)pcloud.getData().size();i++){
        if(MetricUtil::norm2(pcloud.getData()[i])> 2){ //6
            pcloud.getData()[i] = Vector3D<float>(0,0,0);
        }
    }
    
    //from PointCloud to pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZRGBA> cloud(640, 480);
    float ff = img2D->getPixelValue(10,10, 2);
    int i = 0;
    #pragma omp parallel for
    for(size_t r = 0; r < cloud.height; ++r)
    {
        for(size_t c = 0; c < cloud.width; ++c)
        {
            cloud(c,r).x = pcloud.getData()[i](0);
            cloud(c,r).y = pcloud.getData()[i](1);
            cloud(c,r).z = pcloud.getData()[i++](2);
            cloud(c,r).r = img2D->getPixelValuei(c,r, 0);
            cloud(c,r).g = img2D->getPixelValuei(c,r, 1);
            cloud(c,r).b = img2D->getPixelValuei(c,r, 2);

            if (img2D->getPixelValue(c,r, 2) != ff && img2D->getPixelValue(c,r, 2) != 0) {
              //  std::cout << "val2: " <<  img2D->getPixelValue(c,r, 2)<< std::endl;
            }

        }
    }
    Eigen::Matrix4f transform;
    transform << 1,0,0,0,0,-1,0,0,0,0,-1,0,0,0,0,1;
    
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr _cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::transformPointCloud(cloud,*_cloud,transform);
    //this should be the more right point cloud, but contains extra dot..
    if(save_pointCloud) pcl::io::savePCDFileASCII("cloud.pcd",*_cloud); //save point cloud
    //Publish the point cloud
    _rosInterface->publishPointCloud(_cloud, "sensor_frame"); 
    
    //Send result to the Action server
    _rosInterface->setActionServer_PointCloud(_cloud);
    _rosInterface->setActionServer_ScanTime(100);
    _rosInterface->setActionServerProgress(0);
    _rosInterface->setScanState(simulated_kinect::FINISHED);
}


/*void ROS_Sensor::on_spnUpdateTime_valueChanged(int value) {
    _timer->setInterval(spnUpdateTime->value());
}
*/
void ROS_Sensor::viewClosed(SensorView* view) {    
    for (std::vector<SensorSet>::iterator it = _sensors.begin(); it != _sensors.end(); ++it) {
        if ((*it).view == view) {
            _sensors.erase(it); //The smart pointers makes sure we delete the view and the sensor
            return;
        }
    }
}


void ROS_Sensor::moveCallback(const robot_msgs::MoveCmd::ConstPtr& msg)
{
  unsigned int n = msg->joint.q.size();
  rw::math::Q q(n);
  for(unsigned int i=0; i<n; i++)
   q[i] = msg->joint.q[i];
  std::cout << "Q goal: " << q << std::endl;
   if(_robot) _robot->setQ(q,_state);
   else   std::cout << "Could not find device!!" << std::endl;
   getRobWorkStudio()->setState(_state);
  
  _rosInterface->pubCmdFinish(msg->cmdId);
}

void ROS_Sensor::moveBufferCallback(const robot_msgs::MoveCmd::ConstPtr& msg)
{
	unsigned int dof = _robot->getDOF();
	rw::trajectory::Path<rw::math::Q> path;
        rw::trajectory::TimedStatePath Tpath;
	
	
	unsigned int n = msg->joint.q.size();
	rw::math::Q q(dof);
	
	for(unsigned int i=0; i<n; i= i +dof){
	  for(unsigned int j=0; j<dof; j++){
	  q[j] = msg->joint.q[j+i];
	  }
	  path.push_back(q);
	   std::cout << "Q goal: " << q << std::endl;
	}
	
	for(unsigned int k=1; k<path.size()+1;k++)
	{
	 if(_robot) _robot->setQ(path[k-1],_state);
	  else   std::cout << "Could not find device!!" << std::endl;
	  double time = (1/msg->acc) * k;
	  rw::trajectory::TimedState s(time,_state);
	 
	  Tpath.push_back(s);
	 
	} 
	  getRobWorkStudio()->setTimedStatePath(Tpath);
	 
	  
	   
	   // path.push_back(q);
	//_moveCmdBuffer.push_back(msg);
}
//----------------------------------------------------------------------
Q_EXPORT_PLUGIN2(ROS_Sensor, ROS_Sensor)

