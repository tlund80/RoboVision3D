/*
 * ObjectModel3DSegmentation.cpp
 *
 *  Created on: Jun 5, 2013
 *      Author: thomas
 */

#include <pose_estimation_halcon/ObjectModel3DSegmentation.h>

namespace perception {

ObjectModel3DSegmentation::ObjectModel3DSegmentation() {
	// TODO Auto-generated constructor stub

}

ObjectModel3DSegmentation::~ObjectModel3DSegmentation() {
	// TODO Auto-generated destructor stub
}
/*First step*/
void ObjectModel3DSegmentation::prepareObjectModelForSegmentation(int max_area_holes){

	try
	{

		PrepareObjectModel3d(mModel,HTuple("segmentation"),HTuple("true"),HTuple("max_area_holes"), HTuple(max_area_holes));

	}
	catch (HException& except)
	{
		ROS_ERROR("%s in ObjectModel3DSegmentation::prepareObjectModelForSegmentation()", except.ErrorText().Text());
	}
}
/*Second step*/
void ObjectModel3DSegmentation::SegmentObjectModel3D(ObjectModelSegmentationParams params, HTuple& ObjectModelArray_out){

	HTuple ParamName;
	ParamName.Append("max_orientation_diff");
	ParamName.Append("max_curvature_diff");
	ParamName.Append("min_area");
	ParamName.Append("fitting");
	ParamName.Append("output_xyz_mapping");

	HTuple ParamValues;
	ParamValues.Append(params.max_orientation_diff);
	ParamValues.Append(params.max_curvature_diff);
	ParamValues.Append(params.min_area);
	ParamValues.Append(params.fitting);
	ParamValues.Append(params.output_xyz_mapping);

	try
	{
		SegmentObjectModel3d(mModel,ParamName, ParamValues,&ObjectModelArray_out);


	}
	catch (HException& except)
	{
		ROS_ERROR("%s in ObjectModel3DSegmentation::SegmentObjectModel3D()", except.ErrorText().Text());
	}
}

void ObjectModel3DSegmentation::Fit3DPrimitivesToObjectModel(HTuple Primitive3d_type, HTuple fitting_algorithm, Primitive3DFittingParams params,  HTuple& ObjectModelArray_out){

	HTuple ParamName;
	ParamName.Append("primitive_type");
	ParamName.Append("fitting_algorithm");
	ParamName.Append("min_radius");
	ParamName.Append("max_radius");
	ParamName.Append("output_point_coord");
	ParamName.Append("output_xyz_mapping");

	HTuple ParamValues;
	ParamValues.Append(Primitive3d_type);
	ParamValues.Append(fitting_algorithm);
	ParamValues.Append(params.min_radius);
	ParamValues.Append(params.max_radius);
	ParamValues.Append(params.output_point_coord);
	ParamValues.Append(params.output_xyz_mapping);

	try
	{
		FitPrimitivesObjectModel3d(mModel,ParamName,ParamValues, &ObjectModelArray_out);

	}
	catch (HException& except)
	{
		ROS_ERROR("%s in ObjectModel3DSegmentation::Fit3DPrimitivesToObjectModel()", except.ErrorText().Text());
	}
}



} /* namespace perception */
