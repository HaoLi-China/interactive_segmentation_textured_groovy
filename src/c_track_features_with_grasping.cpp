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
#define PR2 0
#define GRASPING 0

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
#include "cv_bridge/cv_bridge.h"
#include "camera_self_filter/mask.h"
#include "Timer.h"
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/feature.h>
#include "pcl/segmentation/extract_clusters.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <math.h>
#include "pcl/common/common.h"
#include <simple_robot_control/robot_control.h>
#include <tf/transform_listener.h>
#include <pcl/common/angles.h>
#include "pcl/io/pcd_io.h"
#include "pcl_ros/transforms.h"
typedef pcl::PointXYZRGB PointT;
typedef pcl::KdTree<PointT>::Ptr KdTreePtr;


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

	tf::TransformListener tf_listen;

	cv::Mat vis_image;
	cv::Mat vis_image_orig;
	cv::Mat last_image;
	std::vector<std::vector<cv::Point2f> > corner_trajectories;
	std::vector<std::vector<cv::Point2f> > corner_trajectories_filtered_and_transposed;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segment;
#if !NO_ROS
	ros::NodeHandle nh;
#endif
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
	 * Find the corresponding point cloud calculate pose and grasp the object
	*/
	bool grasp();
	/***
	 * Find corners using OpnecCV's goodFeaturesToTrack.
	 */
	std::vector<cv::Point2f> findCorners(cv::Mat& first_image,
			std::vector<std::vector<cv::Point> > contours=std::vector<std::vector<cv::Point> > (), cv::Mat* mask = 0);

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

	bool filterTrajectories();

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

	bool getPclPoint(int x, int y, pcl::PointXYZ& point);

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
	float sampleHypothesisSet(std::vector<int>& feature_assocoation,
			Eigen::VectorXf& feature_per_hyp_count);

};

bool FeatureTracker::grasp() {
#if !NO_ROS
	
	
	double clusterTolerance=0.08;
	int spacialLocator=0;
	int minClusterSize=4;
	int maxClusterSize=200;
	double clusterEpsilon=1;
	double preGraspAbove=0.15;
	double underTheObject=0.02;
	
	ros::Publisher cloud_msg = nh.advertise<sensor_msgs::PointCloud2>(
			"points_segmented", 1);

	ros::Publisher pre_pose = nh.advertise<geometry_msgs::PoseStamped>(
			"our_pose", 1000);

	ros::Publisher pre_grasp = nh.advertise<geometry_msgs::PoseStamped>(
			"pre_grasp", 1000);

	simple_robot_control::Robot robot;
	//DEBUG
	//std::cout << " height of the cloud: " << cloud_segment->height << std::endl;
	//std::cout << " width of the cloud: " << cloud_segment->width << std::endl;
	//std::cout << "size " << cloud_segment->points.size() << std::endl;

	//erase not valid points
	for (size_t i = 0; i < cloud_segment->points.size(); ++i) {
		if (cloud_segment->points[i].x == 0.0) {
			cloud_segment->erase(cloud_segment->begin() + i);
			cloud_segment->width--;
			cloud_segment->points.resize(cloud_segment->width);
		}
	}
	//DEBUG
	//std::cerr << "Cloud after segmentation: " << std::endl;
	//for (size_t i = 0; i < cloud_segment->points.size(); ++i)
		//std::cerr << "    " << cloud_segment->points[i].x << " "
			//	<< cloud_segment->points[i].y << " "
			//	<< cloud_segment->points[i].z << std::endl;

	//computing centroid of the point cloud
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*cloud_segment, centroid);
	//DEBUG
	//std::cerr << "centroid: " << centroid << std::endl;

	if (centroid(1) != 0.0) {
		EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
		EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
		Eigen::Matrix3f cov;
		Eigen::Vector3f eigen_vector1;
		Eigen::Vector3f eigen_vector2;
		Eigen::Vector3f vector3rd;
		Eigen::Matrix4f pose;
		pcl::search::KdTree<PointT>::Ptr clusters_tree_;
		clusters_tree_ = boost::make_shared<pcl::search::KdTree<PointT> >();

		int dotprod;
		pcl::EuclideanClusterExtraction<PointT> cluster_;
		cluster_.setClusterTolerance(clusterTolerance);
		cluster_.setMinClusterSize(minClusterSize);
		cluster_.setMaxClusterSize(maxClusterSize);
		clusters_tree_ = boost::make_shared<pcl::search::KdTree<PointT> >();
		clusters_tree_->setEpsilon(clusterEpsilon);
		cluster_.setSearchMethod(clusters_tree_);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_(
				new pcl::PointCloud<pcl::PointXYZRGB>());

		pcl::PointIndices::Ptr object_inliers(new pcl::PointIndices());
		std::vector<pcl::PointIndices> cluster_indices;
		//some color- doesnt matter in this case
		uint8_t col = 5;

		for (size_t i = 0; i < cloud_segment->points.size(); ++i) {
			pcl::PointXYZRGB point_color;
			point_color.x = cloud_segment->points[i].x;
			point_color.y = cloud_segment->points[i].y;
			point_color.z = cloud_segment->points[i].z;
			point_color.r = col;
			point_color.g = col;
			point_color.b = col;
			pcl_cloud_->points.push_back(point_color);

		}
		//DEBUG
		//std::cerr << "Point cloud : " << std::endl;
		//for (size_t i = 0; i < pcl_cloud_->points.size(); ++i)
			//std::cerr << "    " << pcl_cloud_->points[i].x << " "
					//<< pcl_cloud_->points[i].y << " " << pcl_cloud_->points[i].z
					//<< std::endl;

		cluster_.setInputCloud(pcl_cloud_);
		cluster_.extract(cluster_indices);

		//DEBUG
		//std::cerr << "clusters size: " << int(cluster_indices.size())
			//	<< std::endl;
		
		//compute the real centroid after clustering
		if (int(cluster_indices.size()) > 0) {
			//DEBUG
			//std::cerr << "cluster indices: " << cluster_indices[0] << std::endl;
			pcl::copyPointCloud(*pcl_cloud_, cluster_indices[0],
					*cloud_segment);
			pcl::compute3DCentroid(*cloud_segment, centroid);
		}
		
		//orientation of the gripper based on PCA 

		pcl::computeCovarianceMatrixNormalized(*cloud_segment, centroid, cov);
		pcl::eigen33(cov, eigen_vectors, eigen_values);
		
		//DEBUG
//		std::cerr << "eigen vectors: " << eigen_vectors << std::endl;
	//	std::cerr << "eigen values: " << eigen_values << std::endl;
		eigen_vector1(0) = eigen_vectors(0, 2);
		eigen_vector1(1) = eigen_vectors(1, 2);
		eigen_vector1(2) = eigen_vectors(2, 2);
		eigen_vector2(0) = eigen_vectors(0, 1);
		eigen_vector2(1) = eigen_vectors(1, 1);
		eigen_vector2(2) = eigen_vectors(2, 1);
		dotprod = eigen_vector1.dot(eigen_vector2);
		
		//DEBUG
		//std::cerr << "dot product: " << dotprod << std::endl;
		//std::cerr << "eigen vector1: " << eigen_vector1 << std::endl;
		//std::cerr << "eigen vector2: " << eigen_vector2 << std::endl;

		vector3rd = eigen_vector1.cross(eigen_vector2);
		//DEBUG
		//std::cerr << "cross-vector3: " << vector3rd << std::endl;
		for (int i = 0; i <= 2; i++) {
			pose(i, 0) = eigen_vector1(i);
			pose(i, 1) = eigen_vector2(i);
			pose(i, 2) = vector3rd(i);
			pose(i, 3) = centroid(i);
		}
		pose(3, 0) = 0.0;
		pose(3, 1) = 0.0;
		pose(3, 2) = 0.0;
		pose(3, 3) = 1.0;
		std::cerr << "pose: " << pose << std::endl;

		//transform pose matrix to eigen::affine3d
		Eigen::Affine3d e2;
		e2.matrix() = pose.cast<double>();
		geometry_msgs::Pose p_msgs;

		tf::poseEigenToMsg(e2, p_msgs);

		//publishing the pose 
		geometry_msgs::PoseStamped msg;
		msg.pose = p_msgs;
		msg.header.frame_id = "/openni_rgb_optical_frame";
		msg.header.stamp = ros::Time::now();
		msg.header.seq = 0;
		pre_pose.publish(msg);

		ros::Time time;
		time = ros::Time::now();
		bool found_transform = tf_listen.waitForTransform(
				"/openni_rgb_optical_frame", "/base_link",
				cloud_segment->header.stamp, ros::Duration(1.0));
		//DEBUG
		//std::cerr << "found transform? : " << found_transform << std::endl;

		Eigen::Affine3d e3;
		e3.matrix() = pose.cast<double>();
		geometry_msgs::Pose p_msgs2;

		tf::poseEigenToMsg(e3, p_msgs2);
		geometry_msgs::PoseStamped msg2;

		msg2.pose = p_msgs2;
		msg2.header.frame_id = "/openni_rgb_optical_frame";
		msg2.header.stamp = ros::Time::now();
		msg2.header.seq = 0;

		//transforming the pose to the base link
		//msg_base_link pre position for the grasping
		if (found_transform) {
			geometry_msgs::PoseStamped msg_base_link;
			tf_listen.transformPose("/base_link", cloud_segment->header.stamp,
					msg2, msg2.header.frame_id, msg_base_link);
			msg_base_link.pose.position.z = msg_base_link.pose.position.z
					+ preGraspAbove;
			
			//computing the orientation of the pre pose
			Eigen::Vector3f orth(0, 0, -1);

			Eigen::Quaternionf quaternion;
			quaternion.x() = msg_base_link.pose.orientation.x;
			quaternion.y() = msg_base_link.pose.orientation.y;
			quaternion.z() = msg_base_link.pose.orientation.z;
			quaternion.w() = msg_base_link.pose.orientation.w;
			Eigen::Matrix3f mat;
			Eigen::Matrix3f mat2;
			Eigen::Matrix3f mat_last_new;
			Eigen::Matrix3f mat_last;
			mat_last = quaternion.toRotationMatrix();
			//DEBUG
			//std::cerr << "matrix last BEFORE: " << mat_last << std::endl;
			mat_last_new = mat_last;
			mat_last_new(0, 2) = -mat_last(0, 0);
			mat_last_new(1, 2) = -mat_last(1, 0);
			mat_last_new(2, 2) = -mat_last(2, 0);

			mat_last_new(0, 0) = mat_last(0, 2);
			mat_last_new(1, 0) = mat_last(1, 2);
			mat_last_new(2, 0) = mat_last(2, 2);
			Eigen::Vector3f xaxis;
			Eigen::Vector3f yaxis;
			Eigen::Vector3f zaxis;
			Eigen::Vector3f xaxis_new;
			Eigen::Vector3f yaxis_new;
			Eigen::Vector3f zaxis_new;
			xaxis(0) = mat_last_new(0, 0);
			xaxis(1) = mat_last_new(1, 0);
			xaxis(2) = mat_last_new(2, 0);

			yaxis(0) = mat_last_new(0, 1);
			yaxis(1) = mat_last_new(1, 1);
			yaxis(2) = mat_last_new(2, 1);

			zaxis(0) = mat_last_new(0, 2);
			zaxis(1) = mat_last_new(1, 2);
			zaxis(2) = mat_last_new(2, 2);

			yaxis_new = zaxis.cross(orth);
			zaxis_new = orth.cross(yaxis_new);
			xaxis_new = yaxis_new.cross(zaxis_new);

			mat_last_new.block<3, 1>(0, 0) = xaxis_new;
			mat_last_new.block<3, 1>(0, 1) = yaxis_new;
			mat_last_new.block<3, 1>(0, 2) = zaxis_new;

			//DEBUG
			//std::cerr << "matrix last AFTER: " << mat_last_new << std::endl;

			Eigen::Quaternionf quaternion_last(mat_last_new);

			msg_base_link.pose.orientation.x = quaternion_last.x();
			msg_base_link.pose.orientation.y = quaternion_last.y();
			msg_base_link.pose.orientation.z = quaternion_last.z();
			msg_base_link.pose.orientation.w = quaternion_last.w();

			//publishing the point cloud in rviz
			sensor_msgs::PointCloud2 cloudMessage;

			pcl::toROSMsg(*cloud_segment, cloudMessage);

			cloudMessage.header.stamp = ros::Time::now();
			cloudMessage.header.frame_id = "/openni_rgb_optical_frame";
			cloud_msg.publish(cloudMessage);

			//publishing the final pre pose
			pre_grasp.publish(msg_base_link);

			tf::Stamped<tf::Transform> point_stamped;
			tf::poseStampedMsgToTF(msg_base_link, point_stamped);

			tf::StampedTransform tf_l2(point_stamped, ros::Time::now(),
					"base_link", "doesnt_matter");

			//PARKING POSITION BEFORE STARTING GRASPING PROCESS
			geometry_msgs::PoseStamped msg_base_link_pre;
			
			msg_base_link_pre.header.frame_id = "/openni_rgb_optical_frame";
			msg_base_link_pre.header.stamp = msg_base_link.header.stamp;
			msg_base_link_pre.header.seq = 0;

			double parkingXl=0.437;
			double parkingYl=0.356;
			double parkingZl=0.762;
			double parkingOrientationXl=0.048;
			double parkingOrientationYl=0.133;
			double parkingOrientationZl= -0.352;
			double parkingOrientationWl=0.925;

			
			
			msg_base_link_pre.pose.position.x = parkingXl;
			msg_base_link_pre.pose.position.y = parkingYl;

			msg_base_link_pre.pose.position.z = parkingZl;
			msg_base_link_pre.pose.orientation.x = parkingOrientationXl;
			msg_base_link_pre.pose.orientation.y = parkingOrientationYl;
			msg_base_link_pre.pose.orientation.z = parkingOrientationZl;
			msg_base_link_pre.pose.orientation.w = parkingOrientationWl;

			tf::Stamped<tf::Transform> point_stamped_pre;
			tf::poseStampedMsgToTF(msg_base_link_pre, point_stamped_pre);

			tf::StampedTransform tf_l2_pre(point_stamped_pre, ros::Time::now(),
					"base_link", "doesnt_matter");
			//go to the park position with the left hand
			robot.left_arm.moveGrippertoPose(tf_l2_pre);

			//do the same with the right hand

			geometry_msgs::PoseStamped msg_base_link_pre_r;
			msg_base_link_pre_r.header.frame_id = "/openni_rgb_optical_frame";
			msg_base_link_pre_r.header.stamp = msg_base_link.header.stamp;
			msg_base_link_pre_r.header.seq = 0;
			
			double parkingXr=0.546;
			double parkingYr= -0.271;
			double parkingZr= 0.750;
			double parkingOrientationXr=-0.012;
			double parkingOrientationYr=0.198;
			double parkingOrientationZr= 0.336;
			double parkingOrientationWr=0.921;

			msg_base_link_pre_r.pose.position.x = parkingXr;
			msg_base_link_pre_r.pose.position.y = parkingYr;

			msg_base_link_pre_r.pose.position.z = parkingZr;
			msg_base_link_pre_r.pose.orientation.x = parkingOrientationXr;
			msg_base_link_pre_r.pose.orientation.y = parkingOrientationYr;
			msg_base_link_pre_r.pose.orientation.z = parkingOrientationZr;
			msg_base_link_pre_r.pose.orientation.w = parkingOrientationWr;

			tf::Stamped<tf::Transform> point_stamped_pre_r;
			tf::poseStampedMsgToTF(msg_base_link_pre_r, point_stamped_pre_r);

			tf::StampedTransform tf_l2_pre_r(point_stamped_pre_r,
					ros::Time::now(), "base_link", "doesnt_matter");
			robot.right_arm.moveGrippertoPose(tf_l2_pre_r);

			//move the left gripper to before calculated pre pose
			robot.left_arm.moveGrippertoPose(tf_l2);
			robot.left_gripper.open();

			//left gripper moves to the object position
			//underTheObject- how much under the object should the gripper go
			msg_base_link.pose.position.z = msg_base_link.pose.position.z - preGraspAbove
					- underTheObject;
			tf::Stamped<tf::Transform> point_stamped_aim;
			tf::poseStampedMsgToTF(msg_base_link, point_stamped_aim);
			tf::StampedTransform tf_l_aim(point_stamped_aim, ros::Time::now(),
					"base_link", "doesnt_matter");
			
			
			robot.left_arm.moveGrippertoPose(tf_l_aim);
			
			//close the gripper and go to the pre pose
			robot.left_gripper.close(50);
			robot.left_arm.moveGrippertoPose(tf_l2);

			//DEBUG
			//std::cerr << "position : " << msg_base_link.pose.position
					//<< std::endl;
			
			//move to the pose where we want robot to throw the object
			double throwXr= 0.550;
			double throwYr= 0.711;
			double throwZr=0.804;
			
			
			msg_base_link.pose.position.x = throwXr;
			msg_base_link.pose.position.y = throwYr;
			msg_base_link.pose.position.z = throwZr;
			
			tf::Stamped<tf::Transform> point_stamped_throw;
			tf::poseStampedMsgToTF(msg_base_link, point_stamped_throw);
			tf::StampedTransform tf_l_throw(point_stamped_throw,
					ros::Time::now(), "base_link", "doesnt_matter");
			robot.left_arm.moveGrippertoPose(tf_l_throw);
			robot.left_gripper.open();
		}

	}
#endif
	return true;
}

/***
 * Find corners using OpnecCV's goodFeaturesToTrack.
 */
std::vector<cv::Point2f> FeatureTracker::findCorners(cv::Mat& first_image,
		std::vector<std::vector<cv::Point> > contours, cv::Mat* mask) {
	num_tracking_steps = 0;
	cv::Mat first_image_grey;
	cv::cvtColor(first_image, first_image_grey, CV_BGR2GRAY);
	std::vector<cv::Point2f> corners;
	cv::goodFeaturesToTrack(first_image_grey, corners, MAX_CORNERS, quality,
			min_distance);
	
	cv::cornerSubPix(first_image_grey, corners, cv::Size(WIN_SIZE, WIN_SIZE),
			cv::Size(-1, -1),
			cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));
	float diff;
	for (std::vector<cv::Point2f>::size_type k = 0; k != corners.size(); k++) {
		for (std::vector<std::vector<cv::Point> >::size_type i = 0;
				i != contours.size(); i++) {

			std::vector<cv::Point>& current_contour = contours[i];
			for (std::vector<cv::Point>::size_type j = 0;
					j != contours[i].size(); j++) {

				diff =
						sqrt(
								(corners[k].x - current_contour[j].x)
										* (corners[k].x - current_contour[j].x)
										+ ((corners[k].y - current_contour[j].y)
												* (corners[k].y
														- current_contour[j].y)));
				if (diff < 1.5) {
					
					corners[k].x = 1.0;
					corners[k].y = 1.0;
				}

			}

		}

	}

//DEBUG
//std::cerr<<"corners SIZE BEFORE: "<<corners.size()<<std::endl;
	std::vector<cv::Point2f>::iterator iter = corners.begin();
	while (iter != corners.end()) {
		if ((iter->x - 1.0 < 0.0000001) && (iter->y - 1.0 < 0.0000001)) {
			// erase returns the new iterator to next position
			iter = corners.erase(iter);
		} else {
			// if erase is not called, then we manually increment
			// the iterator to next position
			++iter;
		}
	}
	good_corners = Eigen::VectorXi::Ones(corners.size());
//DEBUG
//std::cerr<<"corners SIZE AFTER: "<<corners.size()<<std::endl;

	//std::cerr<<"corners AFTER: "<<corners<<std::endl;

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
	cv::calcOpticalFlowPyrLK(last_image, new_image_grey,
			corner_trajectories.back(), new_corners, status, error,
			cv::Size(WIN_SIZE, WIN_SIZE), 3,
			cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));

	last_image = new_image_grey;

	corner_trajectories.push_back(new_corners);
	for (unsigned int i = 0; i < good_corners.rows(); i++) {
		good_corners[i] = good_corners[i] & status[i];
	}
	//DEBUG
	//printf("num good corners %d  %d new _corner_size %d \n", good_corners.sum(),
			//good_corners.rows(), corner_trajectories.back().size());

	//visulization
	vis_image_orig = new_image;
	vis_image = vis_image_orig.clone();
	std::vector<cv::Point2f>& old_corners =
			corner_trajectories[num_tracking_steps - 1];
	for (unsigned int i = 0; i < new_corners.size(); i++) {
		if (!good_corners[i])
			continue;
		int color_idx = ((num_tracking_steps * 11) % 256) * 3;
//DEBUG
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

	residuals = Eigen::VectorXf(corner_trajectories_t.size());
	for (unsigned int t = 0; t < corner_trajectories_t.size(); t++) {
		const std::vector<cv::Point2f>& current_trajectory =
				corner_trajectories_t[t];
//DEBUG
//			printf("RTs.size() %d current_trajectory.size() %d\n",RTs.size(),current_trajectory.size());
		assert(RTs.size() == current_trajectory.size() - 1);
		float residual = 0;
		for (unsigned int c = 0; c < current_trajectory.size() - 1; c++) {
			Eigen::Vector3f old_corner(current_trajectory[c].x,
					current_trajectory[c].y, 1.0);
			Eigen::Vector3f new_corner(current_trajectory[c + 1].x,
					current_trajectory[c + 1].y, 1.0);
			Eigen::Vector3f res = new_corner - (RTs[c] * old_corner);
			//DEBUG
//				cout<<"res for traj "<<t<<": vec\n"<<res<<endl;

			residual += std::max(0.0f, res.squaredNorm() - residual_error_comp);

		}
		residuals[t] = residual;
	}
	return true;
}

bool FeatureTracker::filterTrajectories() {

	std::vector<std::vector<cv::Point2f> > corner_trajectories_filtered;
	//DEBUG
	//printf("corner_trajectories size %d corner_trajectories[0].size() %d good _corners %d\n", corner_trajectories.size(), corner_trajectories[0].size(), good_corners.rows());
	int num_good_features = good_corners.sum();

//filter out bad features
	for (unsigned int j = 0; j < corner_trajectories.size(); j++) {
		std::vector<cv::Point2f> features_filtered;
		features_filtered.reserve(num_good_features);
		for (unsigned int i = 0; i < good_corners.rows(); i++) {
			if (good_corners[i]) {
				features_filtered.push_back(corner_trajectories[j][i]);
			}
		}
		corner_trajectories_filtered.push_back(features_filtered);
	}

//transpose
	corner_trajectories_filtered_and_transposed.clear();
	corner_trajectories_filtered_and_transposed.reserve(good_corners.rows());
	for (unsigned int i = 0; i < good_corners.rows(); i++) {
		std::vector<cv::Point2f> features_trans;
		features_trans.reserve(corner_trajectories_filtered.size());
		for (unsigned int j = 0; j < corner_trajectories_filtered.size(); j++) {
			features_trans.push_back(corner_trajectories_filtered[j][i]);
		}
		corner_trajectories_filtered_and_transposed.push_back(features_trans);
	}
//DEBUG
	//printf("corner_trajectories_filtered size %d \n", corner_trajectories_filtered.size());

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

//TODO: This needs to be done more efficiently...
bool FeatureTracker::selectRandomFeatures(std::vector<int>& features_idx,
		const Eigen::VectorXf& mask) {

	float variance = 200.0f * 200.0f; //pixel size of objects
	//select random but depending on gaussian distance
	//first create map
	int num_non_masked_elements = mask.sum();
	if (num_non_masked_elements < 3) {
		//DEBUG
		//printf("num non masked elements %d\n", num_non_masked_elements);
		return false;
	}
	int* map_array = new int[num_non_masked_elements];
	int counter = 0;
	for (int i = 0; i < mask.rows(); i++) {
		if (mask[i] > 0.0) {
			map_array[counter] = i;
			counter++;
		}

	}

	//we assume 3 points are selected...
	int idx1 = rand() % num_non_masked_elements;
	Eigen::VectorXf prob_dist(num_non_masked_elements);
	cv::Point2f current_point =
			corner_trajectories_filtered_and_transposed[map_array[idx1]].back();
	for (int i = 0; i < num_non_masked_elements; i++) {
		if (i == idx1) {
			prob_dist[i] = 0.0;
			continue;
		}
		cv::Point2f other =
				corner_trajectories_filtered_and_transposed[map_array[i]].back();
		float dist_sqr = pow((other.x - current_point.x), 2)
				+ pow((other.y - current_point.y), 2);

		float dist_sqr_n = dist_sqr / variance;
		prob_dist[i] = exp(-0.5 * dist_sqr_n);

	}

	float normalizer = prob_dist.sum();

	//take care of the case where numbers are really low
	if (normalizer < 0.0000000001) {
		return false;
	}

	prob_dist = prob_dist / normalizer;
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

	int num_elements = mask.sum();
	//DEBUG
	//printf("in generateHypothesis num_elements %d\n",num_elements);
	if (num_elements < 3)
		return false;

	std::vector<int> features_idx;
	if (!selectRandomFeatures(features_idx, mask))
		return false;

	RTs.reserve(num_tracking_steps);
	//DEBUG
	//printf("num_tracking_steps %d\n",num_tracking_steps);
	for (int i = 1; i < num_tracking_steps + 1; i++) {
		//DEBUG
		//printf("step gen %d \n", i);
		cv::Mat new_points(features_idx.size(), 1, CV_32FC2);
		cv::Mat old_points(features_idx.size(), 1, CV_32FC2);
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
		//another option
//			cv::Mat RT_cv = cv::estimateRigidTransform(old_points, new_points, false);
		cv::Mat RT_cv = cvEstimateRigidTransformFrom2(old_points, new_points);
		Eigen::Matrix3f RT = Eigen::Matrix3f::Identity();
		Eigen::Matrix<float, 2, 3> rt_temp;
		cv::cv2eigen(RT_cv, rt_temp);
		RT.block<2, 3>(0, 0) = rt_temp;
		RTs.push_back(RT);
	}
	//DEBUG
	//cout<<"last hyp tf \n"<<RTs.back()<<endl;
	return true;

}

bool FeatureTracker::getPclPoint(int x, int y, pcl::PointXYZ& point) {
	//DEBUG
	//std::cerr<<"X : "<<x<<std::endl<<"Y : "<<y<<std::endl;
	if (isnan(cloud->points[x + 640 * y].x))
		return false;
	else {
		point = cloud->points[x + 640 * y];

		return true;
	}
}

/*** Top-level function:
 *  Samples a number of hypothesis sets, i.e. samples sets of possible feature clusters.
 *  Hypothesis sets are scored and best one is returned/visualized.
 */
bool FeatureTracker::evluateTrajectories() {

	int num_samples = 15;
	int num_hypotheses = 10;
	filterTrajectories();
	Eigen::VectorXf scores = Eigen::VectorXf::Zero(num_samples);
	std::vector<std::vector<int> > feature_assocoations;
	std::vector<Eigen::VectorXf> features_per_hyp;

	//create hypothesis set samples
	for (int i = 0; i < num_samples; i++) {
		std::vector<int> feature_assocoation;
		Eigen::VectorXf feature_per_hyp_count;
		scores[i] = sampleHypothesisSet(feature_assocoation,
				feature_per_hyp_count);
		feature_assocoations.push_back(feature_assocoation);
		features_per_hyp.push_back(feature_per_hyp_count);
		//DEBUG
//			if(feature_assocoation.size() != 1){
//				cv::imshow("tracker_temp", vis_image);
//				cvWaitKey(1500);
//			}
	}

	//find best hypothesis...
	int best_hypthesis_set_idx;
	scores.minCoeff(&best_hypthesis_set_idx);

	//..which becomes final feature-to-cluster association.
	std::vector<int>& final_feature_association =
			feature_assocoations[best_hypthesis_set_idx];
	Eigen::VectorXf& final_feature_per_hyp_count =
			features_per_hyp[best_hypthesis_set_idx];

	//DEBUG
	//cout<<"feature association size "<<feature_assocoations.size()<<endl;
	//cout<<"feature per hyp size "<<features_per_hyp.size()<<endl;

	final_feature_per_hyp_count[0] = 0;
	//DEBUG
	//cout<<"final feature_per_hyp\n: "<<final_feature_per_hyp_count<<endl;
	int best_single_hyp;
	final_feature_per_hyp_count.maxCoeff(&best_single_hyp);

	int idx_max_features;
	idx_max_features = best_single_hyp;
	//DEBUG
	//cout<<"idx max features "<<idx_max_features<<endl;
	int features_best_hyp;
	features_best_hyp = final_feature_per_hyp_count[best_single_hyp];
	//DEBUG
	//cout<<"features best hyp "<<features_best_hyp<<endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudnew(
			new pcl::PointCloud<pcl::PointXYZ>);
	cloud_segment = cloudnew;

	vis_image = vis_image_orig.clone();
	size_t index = 0;

	for (int i = 0; i < corner_trajectories_filtered_and_transposed.size();
			i++) {
		int idx = final_feature_association[i];
		if (idx >= 0)

			cv::circle(
					vis_image,
					corner_trajectories_filtered_and_transposed[i].back(),
					3,
					cv::Scalar(colormap[3 * idx], colormap[3 * idx + 1],
							colormap[3 * idx + 2], 0), 5);
#if !NO_ROS
		if ((idx >= 1) && (idx == idx_max_features)) {
			pcl::PointXYZ point;
			if (getPclPoint(
					corner_trajectories_filtered_and_transposed[i].back().x,
					corner_trajectories_filtered_and_transposed[i].back().y,
					point))
					//new point cloud
					{
				//DEBUG
				//cout << "points: " << point << endl;
				cloud_segment->points.push_back(point);
			}
//DEBUG
//			}else{
//				cv::circle(vis_image, corner_trajectories_filtered_and_transposed[i].back(), 9, cv::Scalar(  0 , 0, 0, 0), 5, 1);

		}
#endif

	}

	return true;

}

/***
 * Sample recursively possible feature-to-cluster associations (hypotheses) by generating a hypothesis
 * on the input features set, calculate outliers and generate another hypothesis, ... and so on until a
 * hypothesis exists for each feature.
 */
float FeatureTracker::sampleHypothesisSet(std::vector<int>& feature_assocoation,
		Eigen::VectorXf& feature_per_hyp_count) {
	int num_hypotheses = 10;
	float mask_threshold = 80;

	Eigen::MatrixXf residualMatrix(
			corner_trajectories_filtered_and_transposed.size(), num_hypotheses);

	std::vector<std::vector<Eigen::Matrix3f> > hypothesis;

	//create identity Matrix as first hypothesos
	std::vector<Eigen::Matrix3f> RTs_I;
	RTs_I.resize(num_tracking_steps, Eigen::Matrix3f::Identity());
	//DEBUG
	//printf ("num_tracking_steps %d RTs_I.size() %d \n", num_tracking_steps, RTs_I.size());
	hypothesis.push_back(RTs_I);

	Eigen::VectorXf mask = Eigen::VectorXf::Ones(
			corner_trajectories_filtered_and_transposed.size());
	bool points_left;
	int i = 1;
	for (; i < num_hypotheses; i++) {
		Eigen::VectorXf residuals;
		std::vector<Eigen::Matrix3f>& last_hypothesis = hypothesis[i - 1];
		//DEBUG
		//printf("last_hypothesis size %d\n", last_hypothesis.size());

		//calculate the residuals incurred by current hypothesis (i.e. error between features' actual RT and the hypothetical RT).
		calcResiduals(last_hypothesis,
				corner_trajectories_filtered_and_transposed, residuals);

		//store residuals
		residualMatrix.col(i - 1) = residuals;
//DEBUG
//			cout<<"residuals\n"<<residuals<<endl;
//			cout<<"residuals_mat \n"<<residualMatrix<<endl;

		//find inliers and mark them in binary mask
		Eigen::VectorXf mask_bin =
				(residuals.array() - mask_threshold).matrix();
		for (int i = 0; i < mask_bin.rows(); i++) {
			mask_bin[i] = (mask_bin[i] > 0.0f) ? 1.0f : 0.0f;
		}

		Eigen::ArrayXf mask_array = mask.array();
		Eigen::ArrayXf mask_array_bin = mask_bin.array();

		mask = mask_array.min(mask_array_bin);

		std::vector<Eigen::Matrix3f> new_hypothesis;

		//for the outliers, generate a new hypothesis
		points_left = generateHypothesis(mask, new_hypothesis);
		if (!points_left)
			break;
		//DEBUG
		//printf("generated new hyp of size %d\n", new_hypothesis.size());
		hypothesis.push_back(new_hypothesis);

	}
	//calc last residuals if all hypothesis had to be used
	int& actual_num_hypothesis = i;
	//DEBUG
	//printf("actual_num_hypothesis %d num_hypotheses %d\n", actual_num_hypothesis,num_hypotheses);
	if (actual_num_hypothesis == num_hypotheses) {
		Eigen::VectorXf residuals;
		//DEBUG
		//printf("hypothesis.back() size %d\n", hypothesis.back().size());
		calcResiduals(hypothesis.back(),
				corner_trajectories_filtered_and_transposed, residuals);
		//DEBUG
//			cout<<"last residuals\n"<<residuals;
		residualMatrix.col(residualMatrix.cols() - 1) = residuals;
	} else {
		Eigen::MatrixXf residualMatrix2 = residualMatrix.block(0, 0,
				residualMatrix.rows(), actual_num_hypothesis);
		residualMatrix = residualMatrix2;
	}
//DEBUG
	//cout<<"residuals_mat \n"<<residualMatrix<<endl;

	Eigen::VectorXf summed_residuals = residualMatrix.colwise().sum();
//DEBUG
	//cout<<"summed_residuals \n"<<summed_residuals<<endl;

	Eigen::VectorXf summed_residuals_per_h = Eigen::VectorXf::Zero(
			residualMatrix.cols());

	//Finally run the process again and assign each feature to its best possible RT hypothesis (or mark it as complete outlier).
	std::vector<int> best_feature_h_idx;
	best_feature_h_idx.resize(
			corner_trajectories_filtered_and_transposed.size(), 0);
	feature_per_hyp_count = Eigen::VectorXf::Ones(residualMatrix.cols());
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

		best_feature_h_idx[i] = idx;
		feature_per_hyp_count[idx]++;
		summed_residuals_per_h[idx] += residualMatrix(i, idx);
		//DEBUG
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
		if (idx >= 0 && feature_per_hyp_count[idx] > 5.1) {
			//DEBUG
			//cv::circle(vis_image, corner_trajectories_filtered_and_transposed[i].back(), 9, cv::Scalar( colormap[3*idx], colormap[3*idx+1], colormap[3*idx+2],0), 5);
		} else { //stupid feature
			best_feature_h_idx[i] = -1;
		}
	}
//DEBUG
	//cout<<"num feature_per_hyp_count  \n"<<feature_per_hyp_count<<endl;
	summed_residuals.array() /= feature_per_hyp_count.array();

	//cout<<"summed_residuals \n"<<summed_residuals<<endl;

	//cout<<"summed_residuals_per_h \n"<<summed_residuals_per_h<<endl;

	summed_residuals_per_h.array() /= feature_per_hyp_count.array();
	//cout<<"summed_residuals_per_h \n"<<summed_residuals_per_h<<endl;

	//cout<<"total average residual error "<<summed_residuals_per_h.sum()<<endl;

	feature_assocoation = best_feature_h_idx;
	float outlier_penalty = num_outliers * 100.0;
	//DEBUG
	//printf("outlier num %d penalty %f\n", num_outliers, outlier_penalty);
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

#if NO_ROS
int main( int argc, char** argv )
{
	ros::init(argc, argv, "image_tracker");

	if (argc == 1)
	{
		std::cerr << "Usage: " << argv[0] << " <images>" << std::endl;
		exit(0);
	}
	cv::namedWindow("tracker", 0);
	//DEBUG
	//cv::namedWindow("tracker_temp", 0);

	int num_images = argc - 1;
	int num_images_loaded = 0;
	IplImage** all_images = new IplImage*[num_images];


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
	//sensor_msgs::CvBridge _bridge;
	cv::namedWindow("tracker", 0);
	//DEBUG
	//cv::namedWindow("tracker_temp", 0);
#if PR2
	std::string input_image_topic =
			argc == 2 ? argv[1] : "/kinect/rgb/image_color";
	#if !GRASPING
	input_image_topic =
					argc == 2 ? argv[1] : "/prosilica/image_rect_color";
	#endif


#else
	std::string input_image_topic =
				argc == 2 ? argv[1] : "/camera/rgb/image_color";
#endif

	FeatureTracker ft;

	//convert pcl::PointCloud<pcl::PointXYZRGB> to sensor messages point cloud 2
	sensor_msgs::PointCloud2 cloudMessage;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(
			new pcl::PointCloud<pcl::PointXYZ>);

	sensor_msgs::PointCloud2ConstPtr pclmasg_ptr;
#if PR2
	pclmasg_ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
			"/kinect/depth/points");
#else
	pclmasg_ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
				"/camera/depth/points");

#endif
	pcl::fromROSMsg(*pclmasg_ptr, *cloud1);

	ft.cloud = cloud1;

	sensor_msgs::ImageConstPtr imgmasg_ptr = ros::topic::waitForMessage<
			sensor_msgs::Image>(input_image_topic, ros::Duration(5.0));
	//IplImage* ipl_img = _bridge.imgMsgToCv(imgmasg_ptr, "passthrough");
	//cv::Mat img(ipl_img);
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(imgmasg_ptr);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
  		return 0;
	}
	
	cv::Mat img(cv_ptr->image);
#if PR2 && GRASPING

	ros::ServiceClient svc = ft.nh.serviceClient<camera_self_filter::mask>(
			"self_mask");
	camera_self_filter::mask servicecall;
	servicecall.request.header.frame_id = "openni_rgb_optical_frame";
	servicecall.request.header.stamp = ros::Time::now();
	svc.call(servicecall);
	sensor_msgs::ImageConstPtr maskptr = boost::make_shared<sensor_msgs::Image>(
			boost::ref(servicecall.response.mask_image));
	
	//IplImage* ipl_mask = _bridge.imgMsgToCv(maskptr, "passthrough");
	//cv_bridge::CvImagePtr cv_ptr;
	//cvDilate(ipl_mask, ipl_mask, 0, 3);
	//cvNot(ipl_mask, ipl_mask);
	//cv::Mat mask(ipl_mask);
	cv_ptr = cv_bridge::toCvCopy(maskptr);	
	IplImage ipl_mask = cv_ptr->image;
	cvDilate(&ipl_mask, &ipl_mask, 0, 3);
	cvNot(&ipl_mask, &ipl_mask);
	cv::Mat mask(cv::cvarrToMat(&ipl_mask));


	cv::Mat ROI_mat = cv::Mat::zeros(img.rows, img.cols, CV_8U);

	cv::cvtColor(img, ROI_mat, CV_RGB2GRAY);
	cv::Mat img_color = cv::Mat::zeros(mask.rows, mask.cols, CV_8UC3);
	cv::Mat img_color2 = cv::Mat::zeros(mask.rows, mask.cols, CV_8UC3);
	cv::cvtColor(mask, img_color, CV_GRAY2RGB);
	cv::bitwise_and(img, img_color, img);

	std::vector<std::vector<cv::Point> > contours;
	cv::threshold(mask, mask, 100, 255, CV_THRESH_BINARY);
	cv::Canny(mask, mask, 10, 50, 3);
	cv::findContours(mask, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
	//DEBUG
	//std::cerr<<"contours size: "<<contours.size()<<std::endl;
	//cv::imshow("image", img_color);
	//cv::waitKey();

	//DEBUG
	//cv::imshow("tracker2", mask);
	//cv::waitKey();

	ft.findCorners(img, contours, &mask);
#else
	ft.findCorners(img);
#endif

	cv::imshow("tracker", ft.vis_image);
	cv::waitKey();

	char key;
	int c;
	for (;;) {


		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(
				new pcl::PointCloud<pcl::PointXYZ>);

		sensor_msgs::PointCloud2ConstPtr pclmasg_ptr;
#if PR2
		pclmasg_ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
				"/kinect/depth/points");
#else
		pclmasg_ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
						"/camera/depth/points");

#endif
		pcl::fromROSMsg(*pclmasg_ptr, *cloud1);

		ft.cloud = cloud1;

		sensor_msgs::ImageConstPtr imgmasg_ptr = ros::topic::waitForMessage<
				sensor_msgs::Image>(input_image_topic, ros::Duration(5.0));
		//IplImage* ipl_img = _bridge.imgMsgToCv(imgmasg_ptr, "passthrough");
		//cv::Mat img(ipl_img);
		cv_ptr = cv_bridge::toCvCopy(imgmasg_ptr);
		cv::Mat img(cv_ptr->image);
#if PR2 && GRASPING
		ros::ServiceClient svc = ft.nh.serviceClient<camera_self_filter::mask>(
				"self_mask");
		camera_self_filter::mask servicecall;
		servicecall.request.header.frame_id = "openni_rgb_optical_frame";
		servicecall.request.header.stamp = ros::Time::now();
		svc.call(servicecall);
		sensor_msgs::ImageConstPtr maskptr = boost::make_shared<
				sensor_msgs::Image>(
				boost::ref(servicecall.response.mask_image));

		//IplImage* ipl_mask = _bridge.imgMsgToCv(maskptr, "passthrough");
		//cvDilate(ipl_mask, ipl_mask, 0, 3);
		//cvNot(ipl_mask, ipl_mask);
		//cv::Mat mask(ipl_mask);
		cv_ptr = cv_bridge::toCvCopy(maskptr);	
		cvDilate(cv_ptr->image, cv_ptr->image, 0, 3);
		cvNot(cv_ptr->image, cv_ptr->image);
		cv::Mat mask(cv_ptr->image);		

		cv::Mat ROI_mat = cv::Mat::zeros(img.rows, img.cols, CV_8U);

		cv::cvtColor(img, ROI_mat, CV_RGB2GRAY);
		cv::Mat img_color = cv::Mat::zeros(mask.rows, mask.cols, CV_8UC3);
		cv::cvtColor(mask, img_color, CV_GRAY2RGB);
		cv::bitwise_and(img, img_color, img);
#endif

		Timer t;
		ft.tracking_step(img);
		ft.evluateTrajectories();
		cv::imshow("tracker", ft.vis_image);
		c = cvWaitKey();
		if ((char) c == 27)
			break;
#if PR2 && GRASPING		//grasping mode
		else if ((char) c == 'g') {

			ft.grasp();

		}
#endif
		printf("took %f secs\n", t.stop());

	}

	return 0;

}

#endif



