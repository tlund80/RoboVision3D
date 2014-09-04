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


#include "workcell.h"
#include <string>

#include <rw/geometry/Geometry.hpp>
#include <rw/invkin/IKMetaSolver.hpp>
#include <rw/invkin/JacobianIKSolver.hpp>
#include <rw/invkin/JacobianIKSolverM.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/TreeDevice.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/trajectory.hpp>

namespace next_best_view_planner
{
  
workcell::workcell()
{

}

workcell::~workcell()
{

}

void workcell::init(std::string &workcellFilename, std::string &deviceName, std::string &planning_frame)
{
  using namespace rw::proximity;
  using namespace rwlibs::proximitystrategies;

  if(workcellFilename.length() > 0)
  _workcell = rw::loaders::WorkCellFactory::load(workcellFilename);
  _robot = _workcell->findDevice(deviceName);

  if(!_robot.get())
    RW_THROW("No such device: " << deviceName);
	
  std::cout << "DOF=" << _robot->getDOF() << std::endl;
  
  _tcpFrame = _workcell->findFrame(planning_frame);
  if(!_tcpFrame) _tcpFrame = _robot->getEnd();
  
  _collisionDetector = CollisionDetector::Ptr(new CollisionDetector(_workcell, ProximityStrategyFactory::makeCollisionStrategy("PQP")));
  _state = _workcell->getDefaultState();

}

bool workcell::inCollision(const Q& q, std::string& colFrameStr)
{
	using namespace rw::proximity;
	
	CollisionDetector::QueryResult queryResult;
	_robot->setQ(q, _state);
	if(_collisionDetector->inCollision(_state, &queryResult))
	{
		std::stringstream sStr;
		rw::kinematics::FramePairSet::iterator it;
		for(it=queryResult.collidingFrames.begin(); it!=queryResult.collidingFrames.end(); it++) {
			//sStr << "\t" << it->first->getName() << " <--> " << it->second->getName() << "\n";
			sStr << "[" << it->first->getName() << ", " << it->second->getName() << "]";
		}
		colFrameStr = sStr.str();
		return true;
	}

	return false;
}

bool workcell::solve_invkin(const Transform3D<> &trans, bool stopAtFirst, const Q &initQ, std::vector<Q> &solutions, bool doCollisionDetection, bool enable_joint6swop)
{
  
	using namespace rw::proximity;
	using namespace rw::invkin;
   
	CollisionDetector::Ptr collisionDetector;
	
	if (doCollisionDetection) {
		collisionDetector = _collisionDetector;
	}
	else {
		collisionDetector = NULL;
	}
	
	if(stopAtFirst)
		_robot->setQ(initQ, _state);
	
	JacobianIKSolver solver(_robot, _tcpFrame, _state);
	IKMetaSolver metaSolver(&solver, _robot, collisionDetector);
	metaSolver.setCheckJointLimits(true);
	
	solutions = metaSolver.solve(trans, _state, 100, stopAtFirst);
	
	if(solutions.empty() && enable_joint6swop)
	{
	
	  Rotation3D<> rot(1,0,0,0,-1,0,0,0,-1);
	  Rotation3D<> R = trans.R() * rot;
	  Vector3D<> T = trans.P();
	  Transform3D<> _new_trans(T,R);
	  
	  solutions = metaSolver.solve(_new_trans, _state, 100, stopAtFirst);
	  
	}
	
	return !solutions.empty();
	
}

bool workcell::solve_invkin(std::string tcpFrameName, const Transform3D<> &trans, bool stopAtFirst, const Q &initQ, std::vector<Q> &solutions, bool enable_joint6swop)
{
  	using namespace rw::invkin;
	using namespace rw::kinematics;
	if(stopAtFirst)
		_robot->setQ(initQ, _state);
	Frame *tcpFrame = _workcell->findFrame(tcpFrameName);
	if(!tcpFrame)
		RW_THROW("The frame '" << tcpFrameName << "' doesn't exist");
	JacobianIKSolver solver(_robot, tcpFrame, _state);
	IKMetaSolver metaSolver(&solver, _robot);//, _collisionDetector);
	metaSolver.setCheckJointLimits(true);
	solutions = metaSolver.solve(trans, _state, 100, stopAtFirst);
	
	if(solutions.empty() && enable_joint6swop)
	{
	
	  Rotation3D<> rot(1,0,0,0,-1,0,0,0,-1);
	  Rotation3D<> R = trans.R() * rot;
	  Vector3D<> T = trans.P();
	  Transform3D<> _new_trans(T,R);
	  
	  solutions = metaSolver.solve(_new_trans, _state, 100, stopAtFirst);
	  
	}
	
	return !solutions.empty();
	
}

bool workcell::solve_invkin_multi(const std::vector<Transform3D<> > &trans, Q &q)
{
	using namespace rw::invkin;
  	using namespace rw::kinematics;
	using namespace rw::models;
	
        State state = _workcell->getDefaultState();
	TreeDevice *treeDev = static_cast<TreeDevice*>(_robot.get());
	JacobianIKSolverM solver(treeDev, state);
	std::vector<Q> solutions = solver.solve(trans, state);
	if(solutions.empty())
		return false;
	q = solutions[0];
	return true;
}

void workcell::test_moveJ(const Q& startQ, const Q& endQ)
{
  	using namespace rw::proximity;
	using namespace rw::trajectory;
	
	double t_total=10.0;
	double dt=0.01;
	CollisionDetector::QueryResult queryResult;
	QLinearInterpolator qLinearInterpolator(startQ, endQ, t_total);
	for(double t=0.0; t<=t_total; t+=dt)
	{
		Q q = qLinearInterpolator.x(t);
		_robot->setQ(q, _state);
		if(_collisionDetector->inCollision(_state, &queryResult))
		{
			RW_THROW("test_moveJ: Collision detected! (" << q << ")");
		}
	}
}

void workcell::test_moveL(const Transform3D<> &startTrans, const Transform3D<> &endTrans)
{
  	using namespace rw::proximity;
	using namespace rw::trajectory;
	
	double x_total = (endTrans.P() - startTrans.P()).norm2();
	double dx=0.01;	// dx = 1 cm
	CollisionDetector::QueryResult queryResult;
	CartesianLinearInterpolator cartLinearInterpolator(startTrans, endTrans, x_total);
	for(double x=0.0; x<=x_total; x+=dx)
	{
		const bool stopAtFirst = true;
		Q initQ(6, 0.0); // starting guess for inverse kinmatics calculation		
		std::vector<rw::math::Q> solutions;
				
		Transform3D<> trans = cartLinearInterpolator.x(x);
		if (!solve_invkin(trans, stopAtFirst, initQ, solutions, false, false))
			RW_THROW("test_moveL: No inv.kin. solutions (x=" << x << ")");

		_robot->setQ(solutions[0], _state);
	}
}

Transform3D< double > workcell::getTcp(const Q& q)
{
	_robot->setQ(q, _state);
	return rw::kinematics::Kinematics::frameTframe(getRobot()->getBase(), _tcpFrame, _state);
}

}