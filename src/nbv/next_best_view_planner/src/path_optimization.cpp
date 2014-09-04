/********************************************************************************************************************
 *
 * Copyright (c) 2013
 *
 * Danish Technological Institute (DTI)
 *
 *-------------------------------------------------------------------------------------------------------------------
 *
 * @file       Optimizing.cpp
 * @Author     Martin Mølbach Olsen (mmo@dti.dk)
 * @brief
 * @details
 * @addtogroup
 *
 * $Revision: 4075 $
 * $Date: 2013-11-12 12:17:47 +0100 (Tue, 12 Nov 2013) $
 * $Author: arf $
 * $Id: Optimizing.cpp 4075 2013-11-12 11:17:47Z arf $
 *
 *
 *-------------------------------------------------------------------------------------------------------------------
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the Danish Technolocial Institute (DTI) nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * This program is copyrighted software: you can redistribute it and/or modify
 * it under the terms and protection of the Consortium Agreements and/or NDAs
 * assosiated to collaborative projects with DTI. Redistribution to and use by
 * parties outside of such an agreement is prohibited.
 *
 * This program is distributed in the hope that modifications of this software
 * and results achieved thereof will be reported back to DTI and the collaborative
 * projects. When modifying and evaluating this software, ensure that the author
 * have been notified.
 *
 *
 ********************************************************************************************************************/

#include "path_optimization.h"
#include "workcell.h"

#include <rw/math/Q.hpp>
#include <rw/math/MetricFactory.hpp>
#include <rwlibs/pathoptimization/pathlength/PathLengthOptimizer.hpp>
#include <rwlibs/pathoptimization/clearance/ClearanceOptimizer.hpp>
#include <rwlibs/pathoptimization/clearance/MinimumClearanceCalculator.hpp>



using namespace rw::math;
using namespace rw::pathplanning;
using namespace rwlibs::pathoptimization;


namespace next_best_view_planner {

Path_optimization::Path_optimization(boost::shared_ptr<workcell> workcell, boost::shared_ptr<OptimizationConfiguration> Config) : _config(Config), _workcell(workcell)
{
   OptimizationConfiguration *configuration = _config.get();
   
  if(_config->getOptimizationtype() == OptimizationConfiguration::Clearance)
  {
   
    ClearanceConfig* c = (ClearanceConfig*)configuration;
    _optimizingType = Path_optimization::Clearance;
    
    std::cout << "Optimization type is Clearance!!" << std::endl;
    _optimizing_Clearance.init(c, _workcell.get());
  }else if (_config->getOptimizationtype() == OptimizationConfiguration::PathLength)
  {
     std::cout << "Optimization type is PathLength!!" << std::endl;
     PathLengthConfig* c = (PathLengthConfig*)configuration;
     _optimizingType = Path_optimization::PathLength;
     _optimizing_PathLength.init(c,_workcell.get());
    
  }
}

Path_optimization::~Path_optimization()
{
}

void Path_optimization::optimize(rw::trajectory::Path<rw::math::Q> &path)
{
	if(_optimize)
	{
		switch(_optimizingType)
		{
		case Path_optimization::Clearance:
			_optimizing_Clearance.optimize(path);
			break;
		case Path_optimization::PathLength:
			_optimizing_PathLength.optimize(path);
			break;
		}
	}
}



void Path_optimization::Optimizing_Clearance::init(ClearanceConfig* config, workcell *workcell)
{
	_workcell = workcell;
	_max_time = config->getMaxTime();
	_max_count = config->getMaxCount();
	_stepsize = config->getStepsize();
}

void Path_optimization::Optimizing_Clearance::optimize(rw::trajectory::Path<rw::math::Q> &path)
{
	EuclideanMetric<Q>::Ptr metricPtr = MetricFactory::makeEuclidean<Q>();
	rw::common::Ptr<MinimumClearanceCalculator> mccPtr = rw::common::Ptr<MinimumClearanceCalculator>(new MinimumClearanceCalculator(_workcell->getWorkcell(), _workcell->getState()));
	ClearanceOptimizer co(_workcell->getWorkcell(), _workcell->getRobot(), _workcell->getState(), metricPtr, mccPtr);
	path = co.optimize(path, _stepsize, _max_count, _max_time);
}



void Path_optimization::Optimizing_PathLength::init(PathLengthConfig* config, workcell *workcell)
{
	_workcell = workcell;
	_resolution = config->getResolution();
}

void Path_optimization::Optimizing_PathLength::optimize(rw::trajectory::Path<rw::math::Q> &path)
{
	//EuclideanMetric<Q>::Ptr metricPtr = MetricFactory::makeEuclidean<Q>();
	InfinityMetric<Q>::Ptr metricPtr = MetricFactory::makeInfinity<Q>();
	QConstraint::Ptr qConstraintPtr = QConstraint::make(_workcell->getCollisionDetector(), _workcell->getRobot(), _workcell->getState());
	//QEdgeConstraint::Ptr qEdgeConstraintPtr = QEdgeConstraint::makeDefault(qConstraintPtr, _workcell->getRobot());
	QEdgeConstraint::Ptr qEdgeConstraintPtr = QEdgeConstraint::make(qConstraintPtr, metricPtr, _resolution);

	PlannerConstraint plannerConstraint(qConstraintPtr, qEdgeConstraintPtr);

	PathLengthOptimizer plo(plannerConstraint, metricPtr);
	path = plo.pathPruning(path);
	//path = plo.shortCut(path);
	//path = plo.partialShortCut(path);
}

} /* namespace MP */
