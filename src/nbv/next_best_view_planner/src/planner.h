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

#include <rw/pathplanning/QSampler.hpp>
#include <rwlibs/pathplanners/prm/PRMPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>

#include <boost/random.hpp>
#include <boost/random/uniform_on_sphere.hpp>
#include <boost/shared_ptr.hpp>

#include "workcell.h"
#include "MGPGraph.h"

#ifndef PLANNER_H
#define PLANNER_H

namespace next_best_view_planner
{
class PlannerConfiguration;
class PRMConfig;
class RRTConfig;
  
class Planner
{
  
  public:
	Planner(boost::shared_ptr<workcell> workcell, boost::shared_ptr<PlannerConfiguration> Config);
	virtual ~Planner();
	
	bool plan(const rw::math::Q &qstart, const rw::math::Q &qgoal, rw::trajectory::Path<Q> &path);
	void sample_sphere(const rw::math::Transform3D<> &orgin, std::vector<rw::math::Transform3D<> > &pose_list);
	void sample_sphere_random(const rw::math::Transform3D<> &orgin, std::vector<rw::math::Transform3D<> > &pose_list);
	void computeShortestPath(const std::vector<rw::math::Transform3D<> > &pose_list);
	
  private:
	enum PlannerType { PRM, RRT };
	enum SamplerType { GAUSS, UNIFORM };
	
	class GaussSampler : public rw::pathplanning::QSampler
	{
	public:
		GaussSampler(const rw::math::Q &qstart, const rw::math::Q &qgoal, double sigma)
		: _sigma(sigma),
		  _normal_distribution()
		{
			_qMean = (qstart + qgoal) /2.0;

			//uint32_t seed = rw::common::TimerUtil::currentTimeMs();
			//uint32_t seed = 4294967295;
			uint32_t seed = 982893126;
			std::cout << "GaussSampler sampler using seed=" << seed << std::endl;
		    _generator.seed(static_cast<boost::mt19937::result_type>(seed));
			_normal_distribution = new boost::variate_generator<boost::mt19937, boost::normal_distribution<double> >(_generator, boost::normal_distribution<double>(0, 1));
		}

		virtual ~GaussSampler()
		{
			if(_normal_distribution)
				delete _normal_distribution;
		}

  protected:
        rw::math::Q doSample()
        {
        	rw::math::Q q(_qMean.size());
        	for(unsigned int j=0; j<_qMean.size(); j++)
        		q[j] = ranNormalDist(_qMean[j], _sigma);
        	return q;
        }

    	double ranNormalDist(double mean, double sigma)
    	{
    	    return mean + sigma * (*_normal_distribution)();
    	}

    	double _sigma;
        rw::math::Q _qMean;

    	boost::mt19937 _generator;
        boost::variate_generator<boost::mt19937, boost::normal_distribution<double> > *_normal_distribution;
     };
  
  
	class IKSampler : public GaussSampler
	{
	public:
		IKSampler(rw::models::TreeDevice *treeDev, const rw::math::Q &qstart, const rw::math::Q &qgoal, double sigma, rw::kinematics::State state)
		: GaussSampler(qstart, qgoal, sigma),
		  _treeDev(treeDev),
		  _state(state)
		{
		}

	private:
		rw::math::Q doSample()
		{
			rw::math::Q q(_treeDev->getDOF());
			for(unsigned int j=0; j<q.size(); j++)
				q[j] = (*_normal_distribution)();
			_treeDev->setQ(q, _state);
			std::vector<rw::kinematics::Frame*> endFrames = _treeDev->getEnds();
			return q;
		}

		rw::models::TreeDevice *_treeDev;
		rw::kinematics::State _state;
	};
	
  private:
    
       class MotionPlanning_PRM
	{
	public:
		MotionPlanning_PRM()
		: _collisionCheckingStrategy(rwlibs::pathplanners::PRMPlanner::LAZY),
		  _neighborSearchStrategy(rwlibs::pathplanners::PRMPlanner::BRUTE_FORCE),
		  _shortestPathSearchStrategy(rwlibs::pathplanners::PRMPlanner::A_STAR),
		  _roadmapNodecount(1000),
		  _maxtime(20.0),
		  _resolution(0.1)
		{}

		void init( Planner *motionPlanning, PRMConfig* config);
		bool plan(const rw::math::Q &qstart, const rw::math::Q &qgoal, rw::trajectory::Path<rw::math::Q> &path);

	private:
		Planner *_motionPlanning;
		rwlibs::pathplanners::PRMPlanner::CollisionCheckingStrategy _collisionCheckingStrategy;
		rwlibs::pathplanners::PRMPlanner::NeighborSearchStrategy _neighborSearchStrategy;
		rwlibs::pathplanners::PRMPlanner::ShortestPathSearchStrategy _shortestPathSearchStrategy;
		int _roadmapNodecount;
		double _maxtime;
		double _resolution;
	};

	class MotionPlanning_RRT
	{
	
	public:
		MotionPlanning_RRT(){}

		void init(Planner *motionPlanning, RRTConfig* _config);
		bool plan(const rw::math::Q &qstart, const rw::math::Q &qgoal, rw::trajectory::Path<rw::math::Q> &path);

	private:
		Planner *_motionPlanning;
		
		double _resolution;
		double _extend;
	};


	
	rw::pathplanning::QSampler::Ptr getSampler(const rw::math::Q &qstart, const rw::math::Q &qgoal) const;
	
	boost::shared_ptr<PlannerConfiguration> _config;
	boost::shared_ptr<workcell> _workcell;
	PlannerType _plannerType;
	SamplerType _samplerType;
	MotionPlanning_PRM _motionPlanning_PRM;
	MotionPlanning_RRT _motionPlanning_RRT;
     
};

class PlannerConfiguration
{
	  
  public:
	  PlannerConfiguration(){}
	  virtual ~PlannerConfiguration(){}
	  
	  enum CollisionCheckingStrategy { LAZY, NODECHECK, FULL };
	  enum NeighborSearchStrategy {BRUTE_FORCE, PARTIAL_INDEX_TABLE };
	  enum ShortestPathSearchStrategy {A_STAR, DIJKSTRA};
	  
	  enum PlannerType { PRM, RRT };
	  enum SamplerType { GAUSS, UNIFORM };
	  
	  virtual int getPlannertype() = 0;
};

class PRMConfig : public PlannerConfiguration
{
  
  public:
	  PRMConfig(){}
	  virtual ~PRMConfig(){}
	  
	  inline int getCollisionCheckingStrategy(){return _collisionCheckingStrategy; };
	  inline int getNeighborSearchStrategy(){return _neighborSearchStrategy; };
	  inline int getShortestPathSearchStrategy(){return _shortestPathSearchStrategy; };
	  inline int getRoadmapNodecount(){return _roadmapNodecount; };
	  inline double getMaxTime(){return _maxtime; };
	  inline double getResolution(){return _resolution; };
	  int getPlannertype(){return PRM;};
	  
	  inline void setCollisionCheckingStrategy(int Strategy){_collisionCheckingStrategy = Strategy; };
	  inline void setNeighborSearchStrategy(int Strategy){_neighborSearchStrategy = Strategy; };
	  inline void setShortestPathSearchStrategy(int Strategy){_shortestPathSearchStrategy = Strategy; };
	  inline void setRoadmapNodecount(int Count){_roadmapNodecount = Count; };
	  inline void setMaxTime(double Maxtime){_maxtime = Maxtime; };
	  inline void setResolution(double Resolution){_resolution = Resolution; };
  
  private:
	  int _collisionCheckingStrategy;
	  int _neighborSearchStrategy;
	  int _shortestPathSearchStrategy;
	  int _roadmapNodecount;
	  double _maxtime;
	  double _resolution;
};

class RRTConfig : public PlannerConfiguration
{
  
  public:
	  RRTConfig(){}
	  virtual ~RRTConfig(){}
    
         inline double getResolution(){return _resolution; };
	 inline double getExtend(){return _extend; };
         int getPlannertype(){return RRT;};
	 
	 inline void setResolution(double Resolution){_resolution = Resolution; };
	 inline void setEsxtend(double Extend){_extend = Extend; };
 private:
	 
	 double _extend;
	 double _resolution;
};
	
} /* namespace next_best_view_planner */
#endif // PLANNER_H
