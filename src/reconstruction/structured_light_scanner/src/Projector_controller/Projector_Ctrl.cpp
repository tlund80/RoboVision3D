/*
 * main.cpp
 *
 *  Created on: Mar 18, 2013
 *      Author: thomas
 */
#include <ros/ros.h>

//Qt includes
#include <QApplication>
#include <QtGui>

#include <cmath>
#include <csignal>
#include "ProjectorUI.h"
#include <boost/thread/thread.hpp>

boost::thread run_thread;

int CountFiles(QString path)
{
	int sum = 0;
	QDir dir(path);

	dir.setFilter(QDir::AllDirs | QDir::NoDotAndDotDot);

	QFileInfoList lst = dir.entryInfoList();
	for(int i = 0; i < lst.size(); i++)
	{
		sum = sum + CountFiles(lst.at(i).canonicalPath());
	}
	dir.setFilter(QDir::Files);
	return dir.entryInfoList().size() + sum;
}

void delay()
{
    QTime dieTime= QTime::currentTime().addMSecs(30);// .addSecs(1);
    while( QTime::currentTime() < dieTime )
    QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}

void start_thread(int argc, char **argv)
{
	std::cout << "--> Projector thread is running!" << std::endl;
	QString path = "/home/thomas/dti_co_worker/trunk/dti_co_worker/perception_pipeline/src/3D_processing/Projector_controller/Patterns/";
	QApplication a(argc, argv);

	perception::ProjectorUI *ui = new perception::ProjectorUI();

	int width, height;
	ui->select_screen(a,width, height);
	ui->createGrayPatternSerie_Negative(12);
	ui->createGrayPatternSerie_Positive(12);

	while(ros::ok()){
		for(int k = 0; k<= 10; k++){
			ui->showPattern(ui->getNextPattern_Negative());
			delay();
		}
	}

	std::cout << "--> Closing projector thread!" << std::endl;
	a.exec();
	a.quit();

}

int main(int argc, char **argv)
{
	ROS_INFO("****************************************************");
	ROS_INFO("This node controlling the DELL M110 mini projector  ");
	ROS_INFO("Projects random dot patterns and gray coded patterns");
	ROS_INFO("****************************************************");

	ros::init(argc, argv, "DTI_PROJECTOR_CTRL");
	ros::NodeHandle nh("~");

	std::cout << "--> Starting projector thread!" << std::endl;
	run_thread = boost::thread(&start_thread,argc,argv);

	ROS_INFO("DTI_PROJECTOR_CTRL node is running");


	while(ros::ok())
	{

		ros::spinOnce();
	}

	std::cout << "--> Closing!" << std::endl;
	//run_thread.join();
	return 0;
//	return a.exec();
}
