/*
 * SurfaceModelCreator.h
 *
 *  Created on: Apr 22, 2013
 *      Author: thomas
 */

#ifndef SURFACEMODELCREATOR_H_
#define SURFACEMODELCREATOR_H_

#include <ros/ros.h>
#include <halconcpp/HalconCpp.h>

#include "ModelCreationParameters.hpp"
#include "ModelCreators.hpp"

using namespace HalconCpp;

namespace perception {

class SurfaceModelCreator  : public ModelCreators{
public:
	SurfaceModelCreator();
	virtual ~SurfaceModelCreator();

	virtual void createModel();
	virtual void saveModel(const std::string& savePath);
	void setParameters(const SurfaceModelCreationParameters& param);
	void showModels();
	void getCenterOfSurfaceModel(HTuple& center_out);
	void getBounding_boxOfSurfaceModel(HTuple& Bounding_box_out);
	void getDiameterSurfaceModel(HTuple& Diameter_out);

	HTuple getSurfaceModel(){return mModelID;};

protected:
	/// Save model parameters in database
	virtual void saveModelDB() { };

private:
	HTuple mModelID;				/**< Model ID */
	SurfaceModelCreationParameters mParam;	/**< Model paramters */
};

} /* namespace perception */
#endif /* SURFACEMODELCREATOR_H_ */
