#ifndef PATH_OPTIMIZATION_H_
#define PATH_OPTIMIZATION_H_

#include <rw/math/Q.hpp>
#include <rw/trajectory/Path.hpp>


namespace next_best_view_planner {

class workcell;
class OptimizationConfiguration;
class ClearanceConfig;
class PathLengthConfig;

class Path_optimization
{
private:
	enum OptimizingType { Clearance, PathLength };

public:
	Path_optimization(boost::shared_ptr<workcell> workcell, boost::shared_ptr<OptimizationConfiguration> Config);
	virtual ~Path_optimization();

	//void init(XmlRpc::XmlRpcValue &xmlRpcValue, workcell *workcell);
	void optimize(rw::trajectory::Path<rw::math::Q> &path);

private:
	class Optimizing_Clearance
	{
	public:
		Optimizing_Clearance()
		{}

		void init(ClearanceConfig* config, workcell *workcell);
		void optimize(rw::trajectory::Path<rw::math::Q> &path);

	private:
		workcell *_workcell;
		double _stepsize;
		int _max_count;
		double _max_time;
	};

	class Optimizing_PathLength
	{
	public:
		Optimizing_PathLength()
		{}

		void init(PathLengthConfig* config, workcell *workcell);
		void optimize(rw::trajectory::Path<rw::math::Q> &path);

	private:
		workcell *_workcell;
		double _resolution;
	};

	bool _optimize;
	boost::shared_ptr<OptimizationConfiguration> _config;
	boost::shared_ptr<workcell> _workcell;
	OptimizingType _optimizingType;
	Optimizing_Clearance _optimizing_Clearance;
	Optimizing_PathLength _optimizing_PathLength;
};

class OptimizationConfiguration
{
	  
  public:
	  OptimizationConfiguration(){}
	  virtual ~OptimizationConfiguration(){}
	  
	  enum OptimizingType { Clearance, PathLength, All };
	  
	  virtual int getOptimizationtype() = 0;
};

class ClearanceConfig : public OptimizationConfiguration
{
  
  public:
	  ClearanceConfig(){}
	  virtual ~ClearanceConfig(){}
    
         inline double getStepsize(){return _stepsize; };
	 inline int getMaxCount(){return _max_count; };
	 inline double getMaxTime(){return _max_time; };
         int getOptimizationtype(){return Clearance;};
	 
	 inline void setStepsize(double Stepsize){_stepsize = Stepsize; };
	 inline void setMaxCount(int MaxCount){_max_count = MaxCount; };
	 inline void setMaxTime(double MaxTime){_max_time = MaxTime; };
 private:
	 
	 double _stepsize;
	 int _max_count;
	 double _max_time;
};

class PathLengthConfig : public OptimizationConfiguration
{
  
  public:
	  PathLengthConfig(){}
	  virtual ~PathLengthConfig(){}
    
       
         int getOptimizationtype(){return PathLength;};
	 
	 inline double getResolution(){return _resolution; };
	 inline void setResolution(double Resolution){_resolution = Resolution; };
	 
 private:
	 double _resolution;
};

} /* namespace MP */
#endif /* OPTIMIZING_H_ */
