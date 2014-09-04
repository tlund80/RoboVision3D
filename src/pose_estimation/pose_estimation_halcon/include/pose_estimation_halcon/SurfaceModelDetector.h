/*
 * SurfaceModelDetector.h
 *
 *  Created on: Apr 23, 2013
 *      Author: thomas
 */

#ifndef SURFACEMODELDETECTOR_H_
#define SURFACEMODELDETECTOR_H_

#include <ros/ros.h>
#include <halconcpp/HalconCpp.h>
#include "ModelDetectors.hpp"
#include "ModelDetectionParameters.hpp"

using namespace HalconCpp;

namespace perception {

class SurfaceModelDetector : public ModelDetectors {
public:
	SurfaceModelDetector();
	virtual ~SurfaceModelDetector();

	void loadModel( const std::string& loadPath );
	int detectModel(HTuple& search_data);
	void setParameters(const SurfaceModelDetectionParameters& param);
	void setSurfaceModel(const HTuple& surfaceModel)
	{
		mModelID.Clear();
		mModelID = surfaceModel;
	}

	/*Successors */
	void getKeyPoints(int index, HTuple& keypoints);
	void getSampledScene(HTuple& sampledSceneObjectModel3D_out);
	void getScoreBeforeRefining(int index, HTuple& score_out);
	void getScoreAfterRefining(int index, HTuple& score_out);
	void getPose(int index, HTuple& pose_out, HTuple& score_out);
	void getCenterOfSurfaceModel(HTuple& center_out);
	void getBounding_boxOfSurfaceModel(HTuple& Bounding_box_out);
	void getDiameterSurfaceModel(HTuple& Diameter_out);
	int getBestMatch(HTuple& pose, HTuple& score);
	HTuple getObjectModel(void){return mModelID;};

private:
	SurfaceModelDetectionParameters mParam;
	HTuple mPose, mScore, mSurfaceMatchingResultID;
	bool mVerbose;
};

} /* namespace perception */
#endif /* SURFACEMODELDETECTOR_H_ */
