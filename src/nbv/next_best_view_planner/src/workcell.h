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


#ifndef WORKCELL_H
#define WORKCELL_H

#include <rw/rw.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/proximity/CollisionDetector.hpp>

namespace next_best_view_planner
{

class workcell
{

  public:

	workcell();
	virtual ~workcell();
  
	void init(std::string &workcellFilename, std::string &deviceName, std::string &planning_frame);
	bool inCollision(const Q& q, std::string& colFrameStr);
	bool solve_invkin(const Transform3D<> &trans, bool stopAtFirst, const Q &initQ, std::vector<Q> &solutions, bool doCollisionDetection, bool enable_joint6swop);
	bool solve_invkin(std::string tcpFrameName, const Transform3D<> &trans, bool stopAtFirst, const Q &initQ, std::vector<Q> &solutions, bool enable_joint6swop);
	bool solve_invkin_multi(const std::vector<Transform3D<> > &trans, Q &q);
	void test_moveJ(const Q& startQ, const Q& endQ);
	void test_moveL(const Transform3D<> &startTrans, const Transform3D<> &endTrans);
	Transform3D< double > getTcp(const Q& q);
	void setQ(const rw::math::Q &q){
		_robot->setQ(q, _state);
	}
	
	rw::models::WorkCell::Ptr getWorkcell() const { return _workcell; }
	rw::kinematics::State getState() const { return _state; }
	rw::models::Device::Ptr getRobot() const { return _robot; }
	rw::proximity::CollisionDetector::Ptr getCollisionDetector() const { return _collisionDetector; }
  
  private:
	rw::kinematics::Frame* _tcpFrame;
	rw::models::WorkCell::Ptr _workcell;
	rw::models::Device::Ptr _robot;
	rw::proximity::CollisionDetector::Ptr _collisionDetector;
	rw::kinematics::State _state;
};

} /* namespace next_best_view_planner */
#endif // WORKCELL_H
