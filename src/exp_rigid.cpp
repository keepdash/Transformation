/**************************************************************************************/
/*                                                                                    */
/*  Transformation Library                                                            */
/*  https://github.com/keepdash/Transformation                                        */
/*                                                                                    */
/*  Copyright (c) 2017-2017, Wei Ye                                                   */
/*  All rights reserved.                                                              */
/*                                                                                    */
/**************************************************************************************/

#include <iostream>
#include <cstdlib>
#include <ctime>
#include "Transformation.h"

const int point_count = 10;

int main(void){
	std::srand(std::time(0));

	std::vector<Eigen::Vector3f> src_array;
	std::vector<Eigen::Vector3f> dst_array;
	std::vector<float> weight;

	float a = std::rand() % 180;
	a = 3.1415926535f * a / 180.0f;
	Eigen::Vector3f axis(std::rand(), std::rand(), std::rand());
	axis.normalize();
	Eigen::Matrix3f rot = Eigen::AngleAxisf(a, axis) * Eigen::Scaling(1.0f);
	Eigen::Vector3f trans(std::rand() / 1000.0f, std::rand() / 1000.0f, std::rand() / 1000.0f);

	std::cout << "Rotation matrix:\n" << rot << "\n";
	std::cout << "Translation vector:\n" << trans << "\n\n";

	for (int i = 0; i < point_count; i++){
		int r = std::rand() % 3;
		float x_r = r - 1;
		r = std::rand() % 3;
		float y_r = r - 1;
		r = std::rand() % 3;
		float z_r = r - 1;

		float x = std::rand() / (1000.0f + x_r);
		float y = std::rand() / (1000.0f + y_r);
		float z = std::rand() / (1000.0f + z_r);

		Eigen::Vector3f src;
		src << x, y, z;

		Eigen::Vector3f dst = rot * src + trans;

		src_array.push_back(src);
		dst_array.push_back(dst);
		weight.push_back(1.0f);
	}

	Eigen::Matrix3f rot_e;
	Eigen::Vector3f trans_e;
	float err;
	RigidTransformation(src_array, dst_array, weight, rot_e, trans_e, err);

	std::cout << "Estimated rotation matrix:\n" << rot_e << "\n";
	std::cout << "Estimated translation vector:\n" << trans_e << "\n\n";

	return 1;
}