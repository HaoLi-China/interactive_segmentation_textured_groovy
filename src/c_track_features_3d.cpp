/**
 * Copyright (c) 2011, Robert Bosch LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Robert Bosch LLC nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \author Christian Bersch
 */

#define NO_ROS 0

#include "cv.h"
#include "highgui.h"
#include <stdio.h>
#include <ctype.h>
#include <Eigen/Eigen>
#include <assert.h>
#include <iostream>
#include <set>
#include <opencv2/core/eigen.hpp>
#include <cmath>
#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "cv_bridge/cv_bridge.h"
#include "camera_self_filter/mask.h"
#include "Timer.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <math.h>
//#include <boost/math/distributions/beta.hpp>

using std::cout;
using std::endl;

/***
 * Retrives rotation and translation minimzing the error of the rigid transformation between two sets of points.
 * Modified opencv version.
 */
static void icvGetRTMatrix(const CvPoint2D32f* a, const CvPoint2D32f* b,
		int count, CvMat* M) {

  /** Copied from https://code.ros.org/trac/opencv/browser/trunk/opencv/src/cv/cvlkpyramid.cpp?rev=2170  For license information please see the package LICENSE file                                                                          
                                                                                
  **/

	double sa[16], sb[4], m[4], *om = M->data.db;
	CvMat A = cvMat(4, 4, CV_64F, sa), B = cvMat(4, 1, CV_64F, sb);
	CvMat MM = cvMat(4, 1, CV_64F, m);

	int i;

	memset(sa, 0, sizeof(sa));
	memset(sb, 0, sizeof(sb));

	for (i = 0; i < count; i++) {
		sa[0] += a[i].x * a[i].x + a[i].y * a[i].y;
		sa[1] += 0;
		sa[2] += a[i].x;
		sa[3] += a[i].y;

		sa[4] += 0;
		sa[5] += a[i].x * a[i].x + a[i].y * a[i].y;
		sa[6] += -a[i].y;
		sa[7] += a[i].x;

		sa[8] += a[i].x;
		sa[9] += -a[i].y;
		sa[10] += 1;
		sa[11] += 0;

		sa[12] += a[i].y;
		sa[13] += a[i].x;
		sa[14] += 0;
		sa[15] += 1;

		sb[0] += a[i].x * b[i].x + a[i].y * b[i].y;
		sb[1] += a[i].x * b[i].y - a[i].y * b[i].x;
		sb[2] += b[i].x;
		sb[3] += b[i].y;
	}

	cvSolve(&A, &B, &MM, CV_SVD);

	om[0] = om[4] = m[0];
	om[1] = -m[1];
	om[3] = m[1];
	om[2] = m[2];
	om[5] = m[3];

}

/***
 * Retrives rotation and translation minimzing the error of the rigid transformation between two sets of points.
 *
 */
cv::Mat cvEstimateRigidTransformFrom2(const cv::Mat& A, const cv::Mat& B) {
	cv::Mat M = cv::Mat(2, 3, CV_64F);

	CvPoint2D32f* a = (CvPoint2D32f*) A.data;
	CvPoint2D32f* b = (CvPoint2D32f*) B.data;
	CvMat matM = M;
	icvGetRTMatrix(a, b, 2, &matM);
	return M;
}

pcl::PointXYZ NanCheckContours(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int i,
		int j, int iterations) {
	int k = 0;
	pcl::PointXYZ pointRet;
	iterations = iterations * 2;
	for (; k <= iterations; k++) {
		if (!isnan(cloud->points[i + 640 * (j + 1 * k)].x)) {
			pointRet = cloud->points[i + 640 * (j + 1 * k)];
			break;
		} else if (!isnan(cloud->points[i + 1 * k + 640 * j].x)) {
			pointRet = cloud->points[i + 1 * k + 640 * j];
			break;
		} else if (!isnan(
				cloud->points[i + iterations - 1 * k + 640 * (j + iterations)].x)) {
			pointRet = cloud->points[i + iterations - 1 * k
					+ 640 * (j + iterations)];
			break;

		} else if (!isnan(
				cloud->points[i + iterations + 640 * (j + iterations - 1 * k)].x)) {
			pointRet = cloud->points[i + iterations
					+ 640 * (j + iterations - 1 * k)];
			break;
		}

	}
	return pointRet;
}

bool dealingWithNanPointsOpt(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int i,
		int j, int iterations) {

	int k = 1;
	if ((isnan(cloud->points[i + 640 * j].x))) {

		for (; k <= iterations; k++) {

			if (((i - 1 * k) < 0) && ((j - 1 * k < 0))
					&& ((i + 1 * k) > cloud->height)
					&& ((j + 1 * k) > cloud->width))
				break;
			if (NanCheckContours(cloud, i - 1 * k, j - 1 * k, k).x != 0)
				cloud->points[i + 640 * j] = NanCheckContours(cloud, i - 1 * k,
						j - 1 * k, k);

			if (!isnan(cloud->points[i + 640 * j].x)) {
				break;

			}
		}
		if ((isnan(cloud->points[i + 640 * j].x))) {
			printf(
					"it couldn't find not nan point that is near to cloud->at(i,j) in ",
					k, " iterations");
			return false;
		}

	}

	return true;
}

//function to deal with Nan points from the kinect camera. It is looking for the neighbor
//that is not a Nan and assigns it to the given point.
//

bool getPclPoint(int x, int y)
{



}


bool dealingWithNanPoints(
		pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int i, int j,
		int iterations) {
	int k = 1;
	if ((isnan(cloud->at(i, j).x))) {

		for (; k <= iterations; k++) {

			if (((i - 1 * k) < 0) && ((j - 1 * k < 0))
					&& ((i + 1 * k) > cloud->height)
					&& ((j + 1 * k) > cloud->width))
				break;

			if ((!isnan(cloud->points[i - 1 * k + 640*j].x))
					) {
				cloud->points[i+640* j] = cloud->points[i - 1 * k + 640*j];
			} else if ((!isnan(cloud->points[i + 1 * k+ 640* j].x))
					) {
				cloud->points[i+640* j] = cloud->points[i + 1 * k+ 640* j];
			} else if ((!isnan(cloud->points[i+640*( j + 1 * k)].x))
					) {
				cloud->points[i+640* j] = cloud->points[i+640*( j + 1 * k)];
			} else if ((!isnan(cloud->points[i+640*( j - 1 * k)].x))
					) {
				cloud->points[i+640* j] = cloud->points[i+640*( j - 1 * k)];
			} else if ((!isnan(cloud->points[i + 1 * k+640*( j + 1 * k)].x))
					) {
				cloud->points[i+640* j] = cloud->points[i + 1 * k+640*( j + 1 * k)];
			} else if ((!isnan(cloud->points[i - 1 * k+640*( j + 1 * k)].x))
					) {
				cloud->points[i+640* j] = cloud->points[i - 1 * k+640*( j + 1 * k)];
			} else if ((!isnan(cloud->points[i + 1 * k+640*( j - 1 * k)].x))
					) {
				cloud->points[i+640* j] = cloud->points[i + 1 * k+640*( j - 1 * k)];
			} else if ((!isnan(cloud->points[i - 1 * k+640*( j - 1 * k)].x))
					) {
				cloud->points[i+640* j] = cloud->points[i - 1 * k+640*( j - 1 * k)];
			}

		}

		if ((isnan(cloud->at(i, j).x)) )
			{printf(
					"it couldn't find not nan point that is near to cloud->at(i,j) in ",
					k, " iterations");

			return false;
			}
	} else {
		//do nothing
	}

	return true;
}

/***
 * Extracts corner features from an image  and tracks them in subsequent images.
 * The features are clustered such that each feature cluster consists of features with similar trajectories, i.e.
 * which can be explained by the same sequence of rigid transforms.
 */

class FeatureTracker {
public:

	//Parameters for feature detection
	static const int MAX_CORNERS = 500;
	static const double quality = 0.01;
	static const double min_distance = 10;
	static const int WIN_SIZE = 10;

	//number of points used to calculate RT
	const static int num_points_to_select = 3;

	cv::Mat vis_image;
	cv::Mat vis_image_orig;
	cv::Mat last_image;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

	std::vector<std::vector<cv::Point2f> > corner_trajectories;
	std::vector<std::vector<cv::Point2f> > corner_trajectories_filtered_and_transposed;
	std::vector<std::vector<pcl::PointXYZ> > corner_trajectories_filtered_and_transposed_3d;

	Eigen::VectorXi good_corners;

	//executed tracking steps
	int num_tracking_steps;

	// colormap for disparities, RGB order
	static unsigned char colormap[];

	static const float residual_error_comp = 0.1; // 0.1; //deduct half a pixel from the residuals at each timestep

	FeatureTracker() :
			num_tracking_steps(0) {
	}

	/***
	 * Find corners using OpnecCV's goodFeaturesToTrack.
	 */
	std::vector<cv::Point2f> findCorners(cv::Mat& first_image,
			cv::Mat* mask = 0);

	/***
	 * Execute one tracking step:
	 * 	Calculate sparse optical flow from all corners in old image
	 */
	bool tracking_step(cv::Mat& new_image);

	/***
	 * Caluculates residuals as squared reprojection error between  rigid transform trajectory predicted by hypothesis and actual trajectory.
	 */
	bool calcResiduals(const std::vector<Eigen::Matrix3f>& RTs,
			const std::vector<std::vector<cv::Point2f> >& corner_trajectories_t,
			Eigen::VectorXf& residuals);

	bool filterTrajectories3d();

	/**
	 * Draw random bucket according probability distribution.
	 */
	int draw(Eigen::VectorXf& prob_dist);

	bool selectRandomFeatures(std::vector<int>& features_idx,
			const Eigen::VectorXf& mask);

	/***
	 * generates a rotation translation trajectory hypothesis for all features marked in mask in a RANSAC like fashion.
	 */
	bool generateHypothesis(const Eigen::VectorXf& mask,
			std::vector<Eigen::Matrix3f>& RTs);

	/*** Top-level function:
	 *  Samples a number of hypothesis sets, i.e. samples sets of possible feature clusters.
	 *  Hypothesis sets are scored and best one is returned/visualized.
	 */
	bool evluateTrajectories();

	/***
	 * Sample recursively possible feature-to-cluster associations (hypotheses) by generating a hypothesis
	 * on the input features set, calculate outliers and generate another hypothesis, ... and so on until a
	 * hypothesis exists for each feature.
	 */
	float sampleHypothesisSet(std::vector<int>& feature_assocoation);

};

/***
 * Find corners using OpnecCV's goodFeaturesToTrack.
 */
std::vector<cv::Point2f> FeatureTracker::findCorners(cv::Mat& first_image,
		cv::Mat* mask) {
	num_tracking_steps = 0;
	cv::Mat first_image_grey;
	cv::cvtColor(first_image, first_image_grey, CV_BGR2GRAY);
	std::vector<cv::Point2f> corners;
	cv::goodFeaturesToTrack(first_image_grey, corners, MAX_CORNERS, quality,
			min_distance);

//	vector to store the good corners
	good_corners = Eigen::VectorXi::Ones(corners.size());
	cv::cornerSubPix(first_image_grey, corners, cv::Size(WIN_SIZE, WIN_SIZE),
			cv::Size(-1, -1),
			cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));

//	vector of vectors of corners; put the very first ones into the structure
	corner_trajectories.push_back(corners);
	last_image = first_image_grey;

	//visulization
	vis_image_orig = first_image;
	vis_image = vis_image_orig.clone();
	for (unsigned int i = 0; i < corners.size(); i++) {
		int color_idx = ((num_tracking_steps * 11) % 256) * 3;
		cv::circle(
				vis_image,
				corners[i],
				3,
				cv::Scalar(colormap[color_idx], colormap[color_idx + 1],
						colormap[color_idx + 2], 0), -1);
	}

	return corners;
}

/***
 * Execute one tracking step:
 * 	Calculate sparse optical flow from all corners in old image
 */
bool FeatureTracker::tracking_step(cv::Mat& new_image) {
	num_tracking_steps++;
	cv::Mat new_image_grey;
	cv::cvtColor(new_image, new_image_grey, CV_BGR2GRAY);
	std::vector<cv::Point2f> new_corners;
	std::vector<unsigned char> status;
	std::vector<float> error;

//	calculate optical flow between the last image and the current one
	cv::calcOpticalFlowPyrLK(last_image, new_image_grey,
			corner_trajectories.back(), new_corners, status, error,
			cv::Size(WIN_SIZE, WIN_SIZE), 3,
			cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));

	last_image = new_image_grey;

//	add corners which we recognized to the structure we have
	corner_trajectories.push_back(new_corners);
	for (unsigned int i = 0; i < good_corners.rows(); i++) {
//		put 1 in these elements of the vector where we found corresponding corner, 0 otherwise
		good_corners[i] = good_corners[i] & status[i];
	}
	printf("num good corners %d  %d new _corner_size %d \n", good_corners.sum(),
			good_corners.rows(), corner_trajectories.back().size());

	//visulization
	vis_image_orig = new_image;
	vis_image = vis_image_orig.clone();
	std::vector<cv::Point2f>& old_corners =
			corner_trajectories[num_tracking_steps - 1];
	for (unsigned int i = 0; i < new_corners.size(); i++) {
		if (!good_corners[i])
			continue;
		int color_idx = ((num_tracking_steps * 11) % 256) * 3;
//			cv::circle(vis_image, new_corners[i], 3, cv::Scalar(colormap[color_idx], colormap[color_idx +1], colormap[color_idx +2],0), -1);
//			cv::line(vis_image,new_corners[i], old_corners[i], cv::Scalar(0,0,255,0), 1);
//			cv::line(vis_image,new_corners[i], old_corners[i], cv::Scalar(colormap[color_idx], colormap[color_idx +1], colormap[color_idx +2]), 1);

	}

	return true;

}

/***
 * Caluculates residuals as squared reprojection error between  rigid transform trajectory predicted by hypothesis and actual trajectory.
 */
bool FeatureTracker::calcResiduals(const std::vector<Eigen::Matrix3f>& RTs,
		const std::vector<std::vector<cv::Point2f> >& corner_trajectories_t,
		Eigen::VectorXf& residuals) {

//	the same size as the number of the corners we have
	residuals = Eigen::VectorXf(corner_trajectories_t.size());
//	for each feature
	for (unsigned int t = 0; t < corner_trajectories_t.size(); t++) {
		const std::vector<cv::Point2f>& current_trajectory =
				corner_trajectories_t[t];
//			printf("RTs.size() %d current_trajectory.size() %d\n",RTs.size(),current_trajectory.size());

//		it should be as big as number of steps -1
		assert(RTs.size() == current_trajectory.size() - 1);
		float residual = 0;

//		for one feature calculate the difference for every two time steps and calculate the error with the hypothesis
		for (unsigned int c = 0; c < current_trajectory.size() - 1; c++) {
			Eigen::Vector3f old_corner(current_trajectory[c].x,
					current_trajectory[c].y, 1.0);
			Eigen::Vector3f new_corner(current_trajectory[c + 1].x,
					current_trajectory[c + 1].y, 1.0);
			Eigen::Vector3f res = new_corner - (RTs[c] * old_corner);
//				cout<<"res for traj "<<t<<": vec\n"<<res<<endl;
// sum up all the errors for one feature
			residual += std::max(0.0f, res.squaredNorm() - residual_error_comp);

		}
//		residuals for all features sum up for all time steps
		residuals[t] = residual;
	}
	return true;
}

// function to filter and transpose corner_trajectories vector
bool FeatureTracker::filterTrajectories3d() {

	std::vector<std::vector<cv::Point2f> > corner_trajectories_filtered;
	//std::vector<std::vector<pcl::PointXYZ> > corner_trajectories_filtered3d;

	printf(
			"corner_trajectories size %d corner_trajectories[0].size() %d good _corners %d\n",
			corner_trajectories.size(), corner_trajectories[0].size(),
			good_corners.rows());
//	how many good features do we have
	int num_good_features = good_corners.sum();

//filter out bad features
	for (unsigned int j = 0; j < corner_trajectories.size(); j++) {
		std::vector<cv::Point2f> features_filtered;
		//std::vector<pcl::PointXYZ> features_filtered3d;

		features_filtered.reserve(num_good_features);
		for (unsigned int i = 0; i < good_corners.rows(); i++) {
			if (good_corners[i]) {
				features_filtered.push_back(corner_trajectories[j][i]);
				//features_filtered3d.push_back(cloud->at(corner_trajectories[j][i].x,corner_trajectories[j][i].y));
			}
		}
		corner_trajectories_filtered.push_back(features_filtered);
		//corner_trajectories_filtered3d.push_back(features_filtered3d);

	}

//transpose, so that we have vector of vectors with good corners in time
	corner_trajectories_filtered_and_transposed.clear();
	corner_trajectories_filtered_and_transposed.reserve(good_corners.rows());
	for (unsigned int i = 0; i < good_corners.rows(); i++) {
		std::vector<cv::Point2f> features_trans;

		std::vector<pcl::PointXYZ> features_trans_3d;
		features_trans.reserve(corner_trajectories_filtered.size());
		for (unsigned int j = 0; j < corner_trajectories_filtered.size(); j++) {
			features_trans.push_back(corner_trajectories_filtered[j][i]);
//			printf("0000000 normal %d \n",corner_trajectories_filtered[j][i].x);

//			features_trans_3d.push_back(cloud->at(corner_trajectories_filtered3d[j][i].x,corner_trajectories_filtered3d[j][i].y));

			//features_trans_3d.push_back(cloud->at(corner_trajectories_filtered[j][i].x,corner_trajectories_filtered[j][i].y));
//			printf("0000000 3d %d \n",corner_trajectories_filtered3d[j][i].x);

			//for each corner calculate relevant 3d pcl point

//			printf("features trans %d \n",features_trans.size());
//			printf("features trans 3d %d \n",features_trans_3d.size());


		}
		corner_trajectories_filtered_and_transposed.push_back(features_trans);
		//corner_trajectories_filtered_and_transposed_3d.push_back(features_trans_3d);

	}
//how many good corners we have
	printf("corner_trajectories_filtered size %d \n",
			corner_trajectories_filtered.size());

	//printf("corner_trajectories_filtered3d size %d \n",
				//corner_trajectories_filtered3d.size());
//	printf ("normal******************** %d \n",corner_trajectories_filtered_and_transposed.at(1).back().y);
//			printf ("3d******************** %d \n",corner_trajectories_filtered_and_transposed_3d.at(1).back().y);
//			printf ("3d******************** %d \n",corner_trajectories_filtered_and_transposed_3d.at(1).back().z);


	return true;

}

/**
 * Draw random bucket according probability distribution.
 */
int FeatureTracker::draw(Eigen::VectorXf& prob_dist) {
	float pos = (float) rand() / RAND_MAX;
	int idx = 0;
	float accumulator = 0.0f;
	for (;;) {
		accumulator += prob_dist[idx];
		if (accumulator > pos)
			break;
		idx++;
	}
	return idx;

}

//it gets two numbers of random features from the outliers
//TODO: This needs to be done more efficiently...
bool FeatureTracker::selectRandomFeatures(std::vector<int>& features_idx,
		const Eigen::VectorXf& mask) {

	float variance = 200.0f * 200.0f; //pixel size of objects
	//select random but depending on gaussian distance
	//first create map
//	how many elements we should consider - outliers from previous hypothesis
	int num_non_masked_elements = mask.sum();
	if (num_non_masked_elements < 3) {
		printf("num non masked elements %d\n", num_non_masked_elements);
		return false;
	}
	int* map_array = new int[num_non_masked_elements];
	int counter = 0;
//	we store all the outliers numbers in the map_array array
	for (int i = 0; i < mask.rows(); i++) {
		if (mask[i] > 0.0) {
			map_array[counter] = i;
			counter++;
		}

	}

	//we assume 3 points are selected...
//	we get random corner number
	int idx1 = rand() % num_non_masked_elements;
	Eigen::VectorXf prob_dist(num_non_masked_elements);
//	we get last position for the picked corner
	cv::Point2f current_point =
			corner_trajectories_filtered_and_transposed[map_array[idx1]].back();
//	for all outliers
	for (int i = 0; i < num_non_masked_elements; i++) {

//		if we have bad luck and pick the same outlier- continue
		if (i == idx1) {
			prob_dist[i] = 0.0;
			continue;
		}
//		if not we take every outlier and calculate the distance to the random taken other one
		cv::Point2f other =
				corner_trajectories_filtered_and_transposed[map_array[i]].back();
		float dist_sqr = pow((other.x - current_point.x), 2)
				+ pow((other.y - current_point.y), 2);
		printf("dist sqr %d %f\n", i, dist_sqr);
		float dist_sqr_n = dist_sqr / variance;

//		not sure - we sum up the distances from all outlier to our randomly picked point
		prob_dist[i] = exp(-0.5 * dist_sqr_n);

	}

//	summed distances from every outlier to our one
	cout << "prob_dist\n" << prob_dist << endl;
	float normalizer = prob_dist.sum();

	//take care of the case where numbers are really low
	if (normalizer < 0.0000000001) {
		return false;
	}

//	we have normalized prob_dist vector which consists of distance from every outlier to our one
	prob_dist = prob_dist / normalizer;
	cout << "prob_dist_n\n" << prob_dist << endl;
//	now we have to draw randomly another outlier from the outliers that are good- not so close to our one
	int idx2 = draw(prob_dist);

	assert(map_array[idx1] < mask.rows());
	assert(map_array[idx2] < mask.rows());
	features_idx.clear();
	features_idx.push_back(map_array[idx1]);
	features_idx.push_back(map_array[idx2]);

	delete[] map_array;
	return true;
}

/***
 * generates a rotation translation trajectory hypothesis for all features marked in mask in a RANSAC like fashion.
 */
bool FeatureTracker::generateHypothesis(const Eigen::VectorXf& mask,
		std::vector<Eigen::Matrix3f>& RTs) {

//	how many outliers we have
	int num_elements = mask.sum();
	printf("in generateHypothesis num_elements %d\n", num_elements);
	if (num_elements < 3)
		return false;

	std::vector<int> features_idx;
	if (!selectRandomFeatures(features_idx, mask))
		return false;

//	reserve place for every time step
	RTs.reserve(num_tracking_steps);
	printf("num_tracking_steps %d\n", num_tracking_steps);
//	for every time step do
	for (int i = 1; i < num_tracking_steps + 1; i++) {
		printf("step gen %d \n", i);
		cv::Mat new_points(features_idx.size(), 1, CV_32FC2);
		cv::Mat old_points(features_idx.size(), 1, CV_32FC2);
//		for our (2) randomly picked features store them in two matrices
//		in every matrix there are both cordinates of i.e. 2 randomly picked features
		for (int j = 0; j < features_idx.size(); j++) {
			cv::Point2f& new_point =
					corner_trajectories_filtered_and_transposed[features_idx[j]][i];
			new_points.ptr<float>()[j * 2] = new_point.x;
			new_points.ptr<float>()[j * 2 + 1] = new_point.y;
			cv::Point2f& old_point =
					corner_trajectories_filtered_and_transposed[features_idx[j]][i
							- 1];
			old_points.ptr<float>()[j * 2] = old_point.x;
			old_points.ptr<float>()[j * 2 + 1] = old_point.y;
		}
//			cv::Mat RT_cv = cv::estimateRigidTransform(old_points, new_points, false);
//		estimate rigid transform between two sets of points during one time step
		cv::Mat RT_cv = cvEstimateRigidTransformFrom2(old_points, new_points);
		Eigen::Matrix3f RT = Eigen::Matrix3f::Identity();
		Eigen::Matrix<float, 2, 3> rt_temp;
		cv::cv2eigen(RT_cv, rt_temp);
		RT.block<2, 3>(0, 0) = rt_temp;
//		write the transform in the vector
		RTs.push_back(RT);
	}
	cout << "last hyp tf \n" << RTs.back() << endl;
	return true;

}

/*** Top-level function:
 *  Samples a number of hypothesis sets, i.e. samples sets of possible feature clusters.
 *  Hypothesis sets are scored and best one is returned/visualized.
 */
bool FeatureTracker::evluateTrajectories() {

	int num_samples = 15;
	int num_hypotheses = 10;
	filterTrajectories3d();
	Eigen::VectorXf scores = Eigen::VectorXf::Zero(num_samples);
	std::vector<std::vector<int> > feature_assocoations;

	//create hypothesis set samples
	for (int i = 0; i < num_samples; i++) {
		std::vector<int> feature_assocoation;
//		try 15 sets of hypotheses and look fo the best one by looking up the scores vector
		//	feature_assocoation - vector which describes for which features(place in the vector) there is the best hypothesis (value)

		scores[i] = sampleHypothesisSet(feature_assocoation);

//		store all the vectors of feature_assocoation for every sample
		feature_assocoations.push_back(feature_assocoation);
//			if(feature_assocoation.size() != 1){
//				cv::imshow("tracker_temp", vis_image);
//				cvWaitKey(1500);
//			}
	}


	//find best hypothesis...
//	actually find the best set of hypotheses
	int best_hypthesis_set_idx;
	scores.minCoeff(&best_hypthesis_set_idx);

	//..which becomes final feature-to-cluster association.
	std::vector<int>& final_feature_association =
			feature_assocoations[best_hypthesis_set_idx];

	vis_image = vis_image_orig.clone();
	for (int i = 0; i < corner_trajectories_filtered_and_transposed.size();
			i++) {
//		idx being the number of hypothesis
//		if that many features have the same hypothesis then they will have the same color
		int idx = final_feature_association[i];
		if (idx >= 0) {
			cv::circle(
					vis_image,
					corner_trajectories_filtered_and_transposed[i].back(),
					9,
					cv::Scalar(colormap[3 * idx], colormap[3 * idx + 1],
							colormap[3 * idx + 2], 0), 5);
//			}else{
//				cv::circle(vis_image, corner_trajectories_filtered_and_transposed[i].back(), 9, cv::Scalar(  0 , 0, 0, 0), 5, 1);
		}
	}

	return true;

}

/***
 * Sample recursively possible feature-to-cluster associations (hypotheses) by generating a hypothesis
 * on the input features set, calculate outliers and generate another hypothesis, ... and so on until a
 * hypothesis exists for each feature.
 */
float FeatureTracker::sampleHypothesisSet(
		std::vector<int>& feature_assocoation) {
	int num_hypotheses = 10;
	float mask_threshold = 80;

	Eigen::MatrixXf residualMatrix(
			corner_trajectories_filtered_and_transposed.size(), num_hypotheses);

//	each hypothesis consists of the corners transformations
	std::vector<std::vector<Eigen::Matrix3f> > hypothesis;

	//create identity Matrix as first hypothesos
//	we build hypothesis for one feature in num_tracking_steps time steps
	std::vector<Eigen::Matrix3f> RTs_I;
	RTs_I.resize(num_tracking_steps, Eigen::Matrix3f::Identity());
	printf("num_tracking_steps %d RTs_I.size() %d \n", num_tracking_steps,
			RTs_I.size());
	hypothesis.push_back(RTs_I);

//	vector with size of all the good corners
	Eigen::VectorXf mask = Eigen::VectorXf::Ones(
			corner_trajectories_filtered_and_transposed.size());
	bool points_left;
	int i = 1;

//	for all hypotheses - max 10
	for (; i < num_hypotheses; i++) {
		Eigen::VectorXf residuals;
		std::vector<Eigen::Matrix3f>& last_hypothesis = hypothesis[i - 1];
		printf("last_hypothesis size %d\n", last_hypothesis.size());

		//calculate the residuals incurred by current hypothesis (i.e. error between features' actual RT and the hypothetical RT).
		calcResiduals(last_hypothesis,
				corner_trajectories_filtered_and_transposed, residuals);

		//store residuals for one hypothesis
		residualMatrix.col(i - 1) = residuals;

//			cout<<"residuals\n"<<residuals<<endl;
//			cout<<"residuals_mat \n"<<residualMatrix<<endl;

		//find inliers and mark them in binary mask - 0 for an inlier
		Eigen::VectorXf mask_bin =
				(residuals.array() - mask_threshold).matrix();
		for (int i = 0; i < mask_bin.rows(); i++) {
			mask_bin[i] = (mask_bin[i] > 0.0f) ? 1.0f : 0.0f;
		}

		Eigen::ArrayXf mask_array = mask.array();
		Eigen::ArrayXf mask_array_bin = mask_bin.array();

//		in mask we store for every ith corner whether it is an inlier- 0 or outlier - 1
		mask = mask_array.min(mask_array_bin);

//		mask = mask.array().min(mask_bin.array());

		std::vector<Eigen::Matrix3f> new_hypothesis;

		//for the outliers, generate a new hypothesis
//		it generates hypothesis just for one set of points
		points_left = generateHypothesis(mask, new_hypothesis);
		if (!points_left)
			break;
		printf("generated new hyp of size %d\n", new_hypothesis.size());
		hypothesis.push_back(new_hypothesis);

	}
	//calc last residuals if all hypothesis had to be used
	int& actual_num_hypothesis = i;
	printf("actual_num_hypothesis %d num_hypotheses %d\n",
			actual_num_hypothesis, num_hypotheses);
	if (actual_num_hypothesis == num_hypotheses) {
		Eigen::VectorXf residuals;
		printf("hypothesis.back() size %d\n", hypothesis.back().size());
		calcResiduals(hypothesis.back(),
				corner_trajectories_filtered_and_transposed, residuals);
//			cout<<"last residuals\n"<<residuals;
		residualMatrix.col(residualMatrix.cols() - 1) = residuals;
	} else {
//		get just these residuals that were calculated
//		to remind - residualMatrix consists of hypothesis and residuals for each of them
//			cout<<"residuals_mat \n"<<residualMatrix<<endl;
		Eigen::MatrixXf residualMatrix2 = residualMatrix.block(0, 0,
				residualMatrix.rows(), actual_num_hypothesis);
		residualMatrix = residualMatrix2;
//			cout<<"residuals_mat \n"<<residualMatrix<<endl;
	}

	cout << "residuals_mat \n" << residualMatrix << endl;

// sum of all residuals for every hypothesis
	Eigen::VectorXf summed_residuals = residualMatrix.colwise().sum();

	cout << "summed_residuals \n" << summed_residuals << endl;

	Eigen::VectorXf summed_residuals_per_h = Eigen::VectorXf::Zero(
			residualMatrix.cols());

//	until here we have many hypotheses and every one of them describe just a group o features
	//Finally run the process again and assign each feature to its best possible RT hypothesis (or mark it as complete outlier).
	std::vector<int> best_feature_h_idx;
	best_feature_h_idx.resize(
			corner_trajectories_filtered_and_transposed.size(), 0);
	Eigen::VectorXf feature_per_hyp_count = Eigen::VectorXf::Ones(
			residualMatrix.cols());
	feature_per_hyp_count *= 0.0000001f;
	int num_outliers = 0;
	for (int i = 0; i < corner_trajectories_filtered_and_transposed.size();
			i++) {
		int idx;
		residualMatrix.row(i).minCoeff(&idx);
		float residual_value = residualMatrix(i, idx);

		//check if feature was an outlier
		if (mask[i] > 0.0f) {
			num_outliers++;
			best_feature_h_idx[i] = -1;
			continue;
		}

//		idx - number of hypothesis, i - number of feature
//		for ith feature the best deal hypothesis number idx
		best_feature_h_idx[i] = idx;
//		how many features deals this hypothesis with
		feature_per_hyp_count[idx]++;

//		general result of clustering, summed up all of the features with respecting hypothesis for them
		summed_residuals_per_h[idx] += residualMatrix(i, idx);
//			if (idx >= 0 && feature_per_hyp_count[idx] > 2.1){
//				cv::circle(vis_image, corner_trajectories_filtered_and_transposed[i].back(), 9, cv::Scalar( colormap[3*idx], colormap[3*idx+1], colormap[3*idx+2],0), 5);
//			}else{
//				cv::circle(vis_image, corner_trajectories_filtered_and_transposed[i].back(), 9, cv::Scalar(  0 , 0, 0, 0), 5, 1);
//			}
//
//
////			cv::circle(vis_image, corner_trajectories_filtered_and_transposed[i].back(), 10, cv::Scalar( colormap[3*idx], colormap[3*idx+1], colormap[3*idx+2],0), 3);
////			cv::circle(vis_image, corner_trajectories_filtered_and_transposed[i].back(), 9, cv::Scalar( idx * 256 / actual_num_hypothesis, ( (idx + num_hypotheses) / 3) % num_hypotheses * 256 / num_hypotheses , ( (idx + 2 * num_hypotheses) /3) % num_hypotheses * 256 /num_hypotheses ,0), 5);
////			cv::circle(vis_image, corner_trajectories_filtered_and_transposed[i].back(), 10, cv::Scalar( idx * 256 / num_hypotheses, (idx + 1) % num_hypotheses * 256 / num_hypotheses , (idx + 2) % num_hypotheses * 256 /num_hypotheses ,0), 3);
////			printf ("feature %d is of type %d\n", i,idx);
	}

	vis_image = vis_image_orig.clone();
	for (int i = 0; i < corner_trajectories_filtered_and_transposed.size();
			i++) {
		int idx = best_feature_h_idx[i];
//		if the number of hypothesis is correct and hypothesis deals with more than 5 features then
		if (idx >= 0 && feature_per_hyp_count[idx] > 5.1) {
			cv::circle(
					vis_image,
					corner_trajectories_filtered_and_transposed[i].back(),
					9,
					cv::Scalar(colormap[3 * idx], colormap[3 * idx + 1],
							colormap[3 * idx + 2], 0), 5);
		} else { //stupid feature
			best_feature_h_idx[i] = -1;
		}
	}

	cout << "num feature_per_hyp_count  \n" << feature_per_hyp_count << endl;
	summed_residuals.array() /= feature_per_hyp_count.array();

	cout << "summed_residuals \n" << summed_residuals << endl;

	cout << "summed_residuals_per_h \n" << summed_residuals_per_h << endl;

	summed_residuals_per_h.array() /= feature_per_hyp_count.array();
	cout << "summed_residuals_per_h \n" << summed_residuals_per_h << endl;

	cout << "total average residual error " << summed_residuals_per_h.sum()
			<< endl;

//	vector which describes for which features there is the best hypothesis
	feature_assocoation = best_feature_h_idx;
	float outlier_penalty = num_outliers * 100.0;
	printf("outlier num %d penalty %f\n", num_outliers, outlier_penalty);
	return summed_residuals_per_h.sum();

}

unsigned char FeatureTracker::colormap[36] = { 255, 0, 0, 0, 255, 0, 0, 0, 255,
		255, 255, 0, 255, 0, 255, 0, 255, 255, 127, 0, 0, 0, 127, 0, 0, 0, 127,
		127, 127, 0, 127, 0, 127, 0, 127, 127, };

//unsigned char FeatureTracker::colormap[768] =
//  { 150, 150, 150,
//    107, 0, 12,
//    106, 0, 18,
//    105, 0, 24,
//    103, 0, 30,
//    102, 0, 36,
//    101, 0, 42,
//    99, 0, 48,
//    98, 0, 54,
//    97, 0, 60,
//    96, 0, 66,
//    94, 0, 72,
//    93, 0, 78,
//    92, 0, 84,
//    91, 0, 90,
//    89, 0, 96,
//    88, 0, 102,
//    87, 0, 108,
//    85, 0, 114,
//    84, 0, 120,
//    83, 0, 126,
//    82, 0, 131,
//    80, 0, 137,
//    79, 0, 143,
//    78, 0, 149,
//    77, 0, 155,
//    75, 0, 161,
//    74, 0, 167,
//    73, 0, 173,
//    71, 0, 179,
//    70, 0, 185,
//    69, 0, 191,
//    68, 0, 197,
//    66, 0, 203,
//    65, 0, 209,
//    64, 0, 215,
//    62, 0, 221,
//    61, 0, 227,
//    60, 0, 233,
//    59, 0, 239,
//    57, 0, 245,
//    56, 0, 251,
//    55, 0, 255,
//    54, 0, 255,
//    52, 0, 255,
//    51, 0, 255,
//    50, 0, 255,
//    48, 0, 255,
//    47, 0, 255,
//    46, 0, 255,
//    45, 0, 255,
//    43, 0, 255,
//    42, 0, 255,
//    41, 0, 255,
//    40, 0, 255,
//    38, 0, 255,
//    37, 0, 255,
//    36, 0, 255,
//    34, 0, 255,
//    33, 0, 255,
//    32, 0, 255,
//    31, 0, 255,
//    29, 0, 255,
//    28, 0, 255,
//    27, 0, 255,
//    26, 0, 255,
//    24, 0, 255,
//    23, 0, 255,
//    22, 0, 255,
//    20, 0, 255,
//    19, 0, 255,
//    18, 0, 255,
//    17, 0, 255,
//    15, 0, 255,
//    14, 0, 255,
//    13, 0, 255,
//    11, 0, 255,
//    10, 0, 255,
//    9, 0, 255,
//    8, 0, 255,
//    6, 0, 255,
//    5, 0, 255,
//    4, 0, 255,
//    3, 0, 255,
//    1, 0, 255,
//    0, 4, 255,
//    0, 10, 255,
//    0, 16, 255,
//    0, 22, 255,
//    0, 28, 255,
//    0, 34, 255,
//    0, 40, 255,
//    0, 46, 255,
//    0, 52, 255,
//    0, 58, 255,
//    0, 64, 255,
//    0, 70, 255,
//    0, 76, 255,
//    0, 82, 255,
//    0, 88, 255,
//    0, 94, 255,
//    0, 100, 255,
//    0, 106, 255,
//    0, 112, 255,
//    0, 118, 255,
//    0, 124, 255,
//    0, 129, 255,
//    0, 135, 255,
//    0, 141, 255,
//    0, 147, 255,
//    0, 153, 255,
//    0, 159, 255,
//    0, 165, 255,
//    0, 171, 255,
//    0, 177, 255,
//    0, 183, 255,
//    0, 189, 255,
//    0, 195, 255,
//    0, 201, 255,
//    0, 207, 255,
//    0, 213, 255,
//    0, 219, 255,
//    0, 225, 255,
//    0, 231, 255,
//    0, 237, 255,
//    0, 243, 255,
//    0, 249, 255,
//    0, 255, 255,
//    0, 255, 249,
//    0, 255, 243,
//    0, 255, 237,
//    0, 255, 231,
//    0, 255, 225,
//    0, 255, 219,
//    0, 255, 213,
//    0, 255, 207,
//    0, 255, 201,
//    0, 255, 195,
//    0, 255, 189,
//    0, 255, 183,
//    0, 255, 177,
//    0, 255, 171,
//    0, 255, 165,
//    0, 255, 159,
//    0, 255, 153,
//    0, 255, 147,
//    0, 255, 141,
//    0, 255, 135,
//    0, 255, 129,
//    0, 255, 124,
//    0, 255, 118,
//    0, 255, 112,
//    0, 255, 106,
//    0, 255, 100,
//    0, 255, 94,
//    0, 255, 88,
//    0, 255, 82,
//    0, 255, 76,
//    0, 255, 70,
//    0, 255, 64,
//    0, 255, 58,
//    0, 255, 52,
//    0, 255, 46,
//    0, 255, 40,
//    0, 255, 34,
//    0, 255, 28,
//    0, 255, 22,
//    0, 255, 16,
//    0, 255, 10,
//    0, 255, 4,
//    2, 255, 0,
//    8, 255, 0,
//    14, 255, 0,
//    20, 255, 0,
//    26, 255, 0,
//    32, 255, 0,
//    38, 255, 0,
//    44, 255, 0,
//    50, 255, 0,
//    56, 255, 0,
//    62, 255, 0,
//    68, 255, 0,
//    74, 255, 0,
//    80, 255, 0,
//    86, 255, 0,
//    92, 255, 0,
//    98, 255, 0,
//    104, 255, 0,
//    110, 255, 0,
//    116, 255, 0,
//    122, 255, 0,
//    128, 255, 0,
//    133, 255, 0,
//    139, 255, 0,
//    145, 255, 0,
//    151, 255, 0,
//    157, 255, 0,
//    163, 255, 0,
//    169, 255, 0,
//    175, 255, 0,
//    181, 255, 0,
//    187, 255, 0,
//    193, 255, 0,
//    199, 255, 0,
//    205, 255, 0,
//    211, 255, 0,
//    217, 255, 0,
//    223, 255, 0,
//    229, 255, 0,
//    235, 255, 0,
//    241, 255, 0,
//    247, 255, 0,
//    253, 255, 0,
//    255, 251, 0,
//    255, 245, 0,
//    255, 239, 0,
//    255, 233, 0,
//    255, 227, 0,
//    255, 221, 0,
//    255, 215, 0,
//    255, 209, 0,
//    255, 203, 0,
//    255, 197, 0,
//    255, 191, 0,
//    255, 185, 0,
//    255, 179, 0,
//    255, 173, 0,
//    255, 167, 0,
//    255, 161, 0,
//    255, 155, 0,
//    255, 149, 0,
//    255, 143, 0,
//    255, 137, 0,
//    255, 131, 0,
//    255, 126, 0,
//    255, 120, 0,
//    255, 114, 0,
//    255, 108, 0,
//    255, 102, 0,
//    255, 96, 0,
//    255, 90, 0,
//    255, 84, 0,
//    255, 78, 0,
//    255, 72, 0,
//    255, 66, 0,
//    255, 60, 0,
//    255, 54, 0,
//    255, 48, 0,
//    255, 42, 0,
//    255, 36, 0,
//    255, 30, 0,
//    255, 24, 0,
//    255, 18, 0,
//    255, 12, 0,
//    255,  6, 0,
//    255,  0, 0,
//  };

//
//	IplImage *image = 0, *grey = 0, *prev_grey = 0, *pyramid = 0, *prev_pyramid = 0, *swap_temp;
//
//	int win_size = 10;
//	const int MAX_COUNT = 500;
//	CvPoint2D32f* points[2] = {0,0}, *swap_points;
//	char* status = 0;
//	int count1 = 0;
//	int need_to_init = 0;
//	int night_mode = 0;
//	int flags = 0;
//	int add_remove_pt = 0;
//	CvPoint pt;
//

#if NO_ROS
int main( int argc, char** argv )		std::vector<cv::Point2f> features_trans;

{
	if (argc == 1)
	{
		std::cerr << "Usage: " << argv[0] << " <images>" << std::endl;
		exit(0);
	}
	cv::namedWindow("tracker", 0);
	cv::namedWindow("tracker_temp", 0);

	int num_images = argc - 1;
	int num_images_loaded = 0;
	IplImage** all_images = new IplImage*[num_images];
//		for (int i = 0 ; i < num_images; i++){
//			all_images[i] = cvLoadImage(argv[i+1]);
//		}

	FeatureTracker ft;
	all_images[0] = cvLoadImage(argv[1]);
	num_images_loaded++;
	cv::Mat img(all_images[0]);
	ft.findCorners(img);

	cv::imshow("tracker", ft.vis_image);
	cv::waitKey();

	int c;
	for (int i = 1; i < num_images; i++) {
		all_images[i] = cvLoadImage(argv[i+1]);
		num_images_loaded++;
		printf("loading image %s\n", argv[i+1]);
		cv::Mat img(all_images[i]);
		ft.tracking_step(img);

		if( (i-1) % 5 == 0) {
			Timer t;
			ft.evluateTrajectories();
			printf("took %f secs\n", t.stop());
			cv::imshow("tracker", ft.vis_image);
			c = cvWaitKey();
			if( (char)c == 27 )
			break;
		}

	}

	for (int i = 0; i < num_images_loaded; i++) {
		cvReleaseImage(&all_images[i]);
	}
	delete [] all_images;

}

#else

/***
 * Main method to be run with PR2 robot
 */

int main(int argc, char** argv) {

	ros::init(argc, argv, "image_tracker");
	ros::NodeHandle nh;
	//sensor_msgs::CvBridge _bridge;
	cv::namedWindow("tracker", 0);
	cv::namedWindow("tracker_temp", 0);

	std::string input_image_topic =
			argc == 2 ? argv[1] : "/camera/rgb/image_color";

	FeatureTracker ft;

	//mycode
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
			new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
			new pcl::PointCloud<pcl::PointXYZ>);

	sensor_msgs::PointCloud2ConstPtr pclmasg_ptr;

	pclmasg_ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
			"/camera/rgb/points");

	pcl::fromROSMsg(*pclmasg_ptr, *cloud);

	ft.cloud=cloud;


//		pcl::VoxelGrid<pcl::PointXYZ> sor;
//		  sor.setInputCloud (cloud);
//		  sor.setLeafSize (0.01f, 0.01f, 0.01f);
//		  sor.filter (*cloud_filtered);

	std::cout << " height of the cloud: " << cloud->height << std::endl;
	std::cout << " width of the cloud: " << cloud->width << std::endl;
	std::cout << "size " << cloud->points.size() << std::endl;

//		for (size_t i = 0; i < cloud->points.size (); ++i)
//		    std::cout << "    " << cloud->points[i].x
//		              << " "    << cloud->points[i].y
//		              << " "    << cloud->points[i].z << std::endl;



	for (int i = 26; i < 31; i++) {

		for (int j = 26; j < 31; j++) {
			std::cout << " " << cloud->at(i, j) << " ";

		}
		std::cout << std::endl;

	}

	std::cout << std::endl;
	std::cout << std::endl;

	std::cout << "before: " << cloud->at(26, 26) << " " << std::endl;
	std::cout << "Filtering" << std::endl;

	bool done;

	done=dealingWithNanPointsOpt(cloud, 26, 26, 4);
	std::cout << std::endl;

	std::cout <<"Is it done? " << done <<std::endl;
	std::cout << std::endl;



	std::cout << std::endl;
	std::cout << "after: " << cloud->at(26, 26)<< " " << std::endl;

	std::cout << std::endl;
	std::cout << std::endl;

	for (int i = 26; i < 31; i++) {

		for (int j = 26; j < 31; j++) {
			std::cout << " " << cloud->at(i, j) << " ";

		}
		std::cout << std::endl;

	}

	std::cout << std::endl;
	std::cout << std::endl;

	//end

	sensor_msgs::ImageConstPtr imgmasg_ptr = ros::topic::waitForMessage<
			sensor_msgs::Image>(input_image_topic);
	// IplImage* ipl_img = _bridge.imgMsgToCv(imgmasg_ptr, "passthrough");
	cv_bridge::CvImagePtr bridge;
	bridge = cv_bridge::toCvCopy(imgmasg_ptr);

	cv::Mat img(bridge->image);

//		ros::ServiceClient svc = nh.serviceClient<camera_self_filter::mask>("self_mask");
//		camera_self_filter::mask servicecall;
//		servicecall.request.header.frame_id = "high_def_optical_frame";
//		servicecall.request.header.stamp = ros::Time::now();
//		svc.call(servicecall);
//		sensor_msgs::ImageConstPtr maskptr = boost::make_shared<sensor_msgs::Image>(boost::ref(servicecall.response.mask_image));
//
//		IplImage* ipl_mask = _bridge.imgMsgToCv(maskptr, "passthrough");
//		cvDilate(ipl_mask, ipl_mask, 0, 3);
//		cvNot(ipl_mask,ipl_mask);
//		cv::Mat mask(ipl_mask);
//
//		cv::Mat ROI_mat = cv::Mat::zeros(img.rows, img.cols, CV_8U);
//		cv::rectangle(ROI_mat, cv::Point(0,0), cv::Point(img.cols-1, 1400),cv::Scalar(255,255,255,0), -1);
//
//		cv::resize(mask, mask, cv::Size(img.cols, img.rows), 0, 0, CV_INTER_NN);
//
//		cv::bitwise_and(ROI_mat,mask, mask);
//
//
//		cv::imshow("tracker", mask);
//		cv::waitKey();

	ft.findCorners(img);
//		ft.findCorners(img, &mask);

	cv::imshow("tracker", ft.vis_image);
	cv::waitKey();

	int c;
	for (;;) {

		//mycode

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
				new pcl::PointCloud<pcl::PointXYZ>);



		//		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

		sensor_msgs::PointCloud2ConstPtr pclmasg_ptr;

		pclmasg_ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
				"/camera/rgb/points");

		pcl::fromROSMsg(*pclmasg_ptr, *cloud);

		ft.cloud=cloud;


		//end

		sensor_msgs::ImageConstPtr imgmasg_ptr = ros::topic::waitForMessage<
				sensor_msgs::Image>(input_image_topic);
		//IplImage* ipl_img = _bridge.imgMsgToCv(imgmasg_ptr, "passthrough");
		cv_bridge::CvImagePtr bridge;
		bridge = cv_bridge::toCvCopy(imgmasg_ptr);
		cv::Mat img(bridge->image);
		//cv::Mat img(ipl_img);
		Timer t;
		ft.tracking_step(img);
		ft.evluateTrajectories();
		printf("took %f secs\n", t.stop());
		std::cout << " height of the cloud: " << cloud->height << std::endl;
		std::cout << " width of the cloud: " << cloud->width << std::endl;
		std::cout << "size " << cloud->points.size() << std::endl;
		std::cout << "point " << cloud->at(0, 0) << std::endl;
		cv::imshow("tracker", ft.vis_image);
		c = cvWaitKey();
		if ((char) c == 27)
			break;

	}

	return 0;

}

#endif

