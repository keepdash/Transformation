/**************************************************************************************/
/*                                                                                    */
/*  Transformation Library                                                            */
/*  https://github.com/keepdash/Transformation                                        */
/*                                                                                    */
/*  Copyright (c) 2017-2017, Wei Ye                                                   */
/*  All rights reserved.                                                              */
/*                                                                                    */
/**************************************************************************************/

#ifndef __TRANSFORMATION__ESTIMATION__
#define __TRANSFORMATION__ESTIMATION__

#include <vector>
#include <Eigen/Dense>

/************************************************************************/
/* 1. Rigid Transformation Estimation									*/
/************************************************************************/

/**
* Estimation rigid transformation of 2 point sets
* @param src_pts: the source point set
* @param dst_pts: the destiny point set
* @param: weight: the weight array of the point set, size should be equal to point set.
* @param: tfMat: the output 4*4 transformation matrix.
* @param: err: the output error of the estimation.
* @remark: if all points have same weight, set all values in [weight] to 1.0f.
*/
extern int RigidTransformation(
	const std::vector<Eigen::Vector3f> &src_pts,
	const std::vector<Eigen::Vector3f> &dst_pts,
	const std::vector<float> &weight,
	Eigen::Matrix4f &tfMat,
	float& err);

/**
* Estimation rigid transformation of 2 point sets
* @param src_pts: the source point set
* @param dst_pts: the destiny point set
* @param: weight: the weight array of the point set, size should be equal to point set.
* @param: rotation: the output 3*3 rotation matrix.
* @param: translation: the output 3*1 translation vector.
* @param: err: the output error of the estimation.
* @remark: if all points have same weight, set all values in [weight] to 1.0f.
*/
extern int RigidTransformation(
	const std::vector<Eigen::Vector3f> &src_pts,
	const std::vector<Eigen::Vector3f> &dst_pts,
	const std::vector<float> &weight,
	Eigen::Matrix3f &rotation,
	Eigen::Vector3f &translation,
	float& err);

/************************************************************************/
/* 2. Similarity Transformation Estimation                              */
/************************************************************************/

/**
* Estimation rigid transformation of 2 point sets
* @param src_pts: the source point set
* @param dst_pts: the destiny point set
* @param: weight: the weight array of the point set, size should be equal to point set.
* @param: tfMat: the output 4*4 transformation matrix.
* @param: err: the output error of the estimation.
* @remark: if all points have same weight, set all values in [weight] to 1.0f.
*/
extern int SimilarityTransformation(
	const std::vector<Eigen::Vector3f> &src_pts,
	const std::vector<Eigen::Vector3f> &dst_pts,
	const std::vector<float> &weight,
	Eigen::Matrix4f &tfMat,
	float& err);

/**
* Estimation rigid transformation of 2 point sets
* @param src_pts: the source point set
* @param dst_pts: the destiny point set
* @param: weight: the weight array of the point set, size should be equal to point set.
* @param: rotation: the output 3*3 rotation matrix.
* @param: translation: the output 3*1 translation vector.
* @param: scale: the output 1*1 scale scalar.
* @param: err: the output error of the estimation.
* @remark: if all points have same weight, set all values in [weight] to 1.0f.
*/
extern int SimilarityTransformation(
	const std::vector<Eigen::Vector3f> &src_pts,
	const std::vector<Eigen::Vector3f> &dst_pts,
	const std::vector<float> &weight,
	Eigen::Matrix3f &rotation,
	Eigen::Vector3f &translation,
	float& scale,
	float& err);

#endif