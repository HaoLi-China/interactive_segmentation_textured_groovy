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

#define RANDOM_PUSHING 0

#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/transforms.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>

#include <pcl/io/pcd_io.h>

#include "roi_pcl_filter.h"

#include "cv_bridge/cv_bridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include <image_geometry/pinhole_camera_model.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <interactive_segmentation_textured/cornerPokePoseFind.h>
#include <interactive_segmentation_textured/cornerFind.h>
#include <boost/foreach.hpp>


//#include <camera_self_filter/mask.h>


#include <pcl/visualization/cloud_viewer.h>

  #include <sensor_msgs/image_encodings.h>



#define X_OUTPUT 1

  using std::cout;
  using std::endl;
  using std::ostream;
  using namespace std;

//typedef pcl::PointXYZ Point;
//typedef pcl::PointCloud<Point> PointCloud;
//typedef PointCloud::Ptr PointCloudPtr;
//typedef PointCloud::ConstPtr PointCloudConstPtr;
//typedef pcl::KdTree<Point>::Ptr KdTreePtr;

  void showPointClound (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,std::string name)
  {
  	pcl::visualization::CloudViewer viewer (name);

  	viewer.showCloud (cloud);
  	while (!viewer.wasStopped ())
  	{

  	}
  }


  struct randomPoint{
  	cv::Point push_point;
  	Eigen::Vector2f direction;
  };


  ostream& operator<< (ostream& out,  const tf::Transform  tf){
  	Eigen::Affine3d tf_eigen;
  	tf::transformTFToEigen(tf,tf_eigen);
  	out << tf_eigen.matrix();
  	return out;

  }


  class PokePointFinder{

  	typedef pcl::PointXYZRGB Point;

  	ros::NodeHandle _nh;
  	ros::NodeHandle _nh_private;
  	ros::ServiceServer _getMaskServer;
  	ros::ServiceClient _corner_finder;
  	sensor_msgs::CameraInfoConstPtr _cam_info;
  	image_geometry::PinholeCameraModel _model;

  	std::string base_frame;

  	double _tolerance_x;
  	double _tolerance_y;
  	double _max_height_z;
  	double virtual_cam_z;
  	double virtual_cam_x;
  	double push_distance;
  	int random_pushing;
  	int morphology_param;
  	double table_height;	

  	float dot_threshold;

  	tf::TransformListener listener_;
	//sensor_msgs::CvBridge _bridge;
	//cv_bridge::CvImagePtr cv_ptr;
  	ROI_Filter<Point> roi_filter;

  	tf::Transform tf_virtual_cam;
  	tf::Transform tf_virtual_cam_transl;

  	pcl::ModelCoefficients::Ptr coefficients;
  	std::string point_cloud;

  	ros::Publisher pub;

  public:
  	PokePointFinder():_nh_private("~"){
  		_nh_private.param("tolerance_x", _tolerance_x, 0.10);
  		_nh_private.param("tolerance_y", _tolerance_y, 0.10);
  		_nh_private.param("max_height_z", _max_height_z, 0.7);


		//_nh_private.param("virtual_cam_z", virtual_cam_z, 1.5);
		//_nh_private.param("virtual_cam_x",virtual_cam_x, .70);

  		_nh_private.param("virtual_cam_z", virtual_cam_z, .0);
  		_nh_private.param("virtual_cam_x",virtual_cam_x, .0);

  		_nh_private.param("random_pushing",random_pushing,0);
  		_nh_private.param("push_distance",push_distance,20.0);
  		_nh_private.param("morphology_param",morphology_param,8);
  		_nh_private.param("table_height",table_height,0.75);
  		table_height=table_height-0.1;

  		std::string camera_name;
  		_nh_private.param<std::string>("camera_name",camera_name, "/camera/rgb");
  		_nh_private.param<std::string>("base_frame",base_frame, "/base_link");


  		printf("test1\n");

  		pub = _nh.advertise<geometry_msgs::PoseStamped>("poke_point_finder_node/concave_pose", 1000);


  		_nh_private.param<std::string>("point_cloud",point_cloud, "/camera/depth_registered/points");
  		printf("test2\n");
  		double dot_threshold_d;
  		_nh_private.param("dot_threshold",dot_threshold_d , 0.5);
  		dot_threshold = dot_threshold_d;

  		_cam_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_name +"/camera_info",  ros::Duration(5.0));
  		printf("test3\n");		

  		_model.fromCameraInfo(_cam_info);
  		printf("test4\n");
//		service to advertise

  		_getMaskServer = _nh.advertiseService("findPokePose", &PokePointFinder::getPokePointServiceCB, this);

//		_self_mask_client = _nh.serviceClient<camera_self_filter::mask>("self_mask");


		//create location of cam over table
  		tf_virtual_cam.setIdentity();
  		tf_virtual_cam_transl.setIdentity();
  		tf_virtual_cam_transl.setOrigin(tf::Vector3(virtual_cam_x, 0.0, virtual_cam_z));
		//optical frame is rotated by 180 degree around y-axis
		/*tf::Matrix3x3 rot(0, -1, 0, -1, 0, 0, 0, 0, -1);
		tf::Quaternion q;
		rot.getRotation(q);
		tf_virtual_cam.setRotation(q);*/


		cout<<"tf_virtual_cam"<<endl<<tf_virtual_cam<<endl;
		cout<<"tf_virtual_cam_transl"<<endl<<tf_virtual_cam_transl<<endl;
		cout<<"base_frame "<<base_frame<<endl;

		printf("test5\n");		

		_corner_finder = _nh.serviceClient<interactive_segmentation_textured::cornerFind>("find_corners");
		if (!_corner_finder.waitForExistence(ros::Duration(5.0))){
			ROS_ERROR("find_corners service not found");
			exit(1);
		}



	}


	bool getRandomPoint (cv::Mat& topview, randomPoint& random)
	{

		cv::Point2f center;
		float radius;
		cv::Mat topview2 = cv::Mat::zeros(topview.rows, topview.cols, CV_8UC3);
		topview.copyTo(topview2);
		std::vector<std::vector <cv::Point> >  contours_old;


		cv::threshold(topview,topview,100,255,CV_THRESH_BINARY);				
		cv::Canny(topview, topview2,10,50,3);
		cv::findContours(topview, contours_old, CV_RETR_LIST,
			CV_CHAIN_APPROX_NONE);
		cv::imshow("image", topview2);
		cv::waitKey();

		cv::minEnclosingCircle(cv::Mat(contours_old.back()), center, radius);
		cv::cvtColor(topview2, topview2, CV_GRAY2BGR);
		cv::circle(topview2, center, (int) radius, cv::Scalar(0, 255, 0), 3);
		cv::imshow("image", topview2);
		cv::waitKey();
		Eigen::Vector2f direction_use(1,1);
		int i_rand_use=0;

		while (1) {

			srand(time(NULL));

			int max = contours_old.back().size();
			int counter = 0;
			int i_rand = rand() % max;

			while (contours_old.back()[i_rand].y - center.y < 0) {
				i_rand = rand() % max;
				counter++;
			}

			center.x = (int) center.x;
			center.y = (int) center.y;

			cv::circle(topview2, contours_old.back()[i_rand], 5,
				cv::Scalar(0, 0, 255), 3);
			Eigen::Vector2f direction(
				(int) center.x - contours_old.back()[i_rand].x,
				(int) center.y - contours_old.back()[i_rand].y);
			direction.normalize();
			
			direction = 40 * direction;

			

			cv::Point2f direct(direction[0] + contours_old.back()[i_rand].x,
				direction[1] + contours_old.back()[i_rand].y);

			direct.x = (int) direct.x;
			direct.y = (int) direct.y;
		//DEBUG
		/*std::cout << std::endl;
		std::cout << "coordinates of the point";
		std::cout << std::endl;
		std::cout << contours_old.back()[i_rand].x;
		std::cout << std::endl;

		std::cout << contours_old.back()[i_rand].y;
		std::cout << std::endl;

		std::cout << "coordinates of direct point";
		std::cout << std::endl;

		std::cout << direct.x;
		std::cout << std::endl;
		std::cout << direct.y;
		std::cout << std::endl;
*/
		cv::circle(topview2, direct, 5, cv::Scalar(0, 0, 255), 3);

		cv::line(topview2, contours_old.back()[i_rand], direct,
			cv::Scalar(255, 0, 0), 7);


		direction_use=direction;
		i_rand_use=i_rand;


		cv::imshow("image", topview2);
		char c = cv::waitKey();
		if ((char) c == 27)
			break;
	}

	
	random.push_point=contours_old.back()[i_rand_use];
	random.direction=direction_use;

	return true;

}






bool getPokePointServiceCB(interactive_segmentation_textured::cornerPokePoseFind::Request& req, interactive_segmentation_textured::cornerPokePoseFind::Response& res){
	cv::Mat top_view_rgb;
	printf("test6\n");
	getTopView(top_view_rgb);

	cv::imshow("top_view_rgb", top_view_rgb);
	cv::waitKey();


	printf("test7\n");
	interactive_segmentation_textured::cornerFind::Response res_corner;
	pcl::PointCloud<pcl::PointNormal> grasps;
	if (random_pushing!=0)
	{
		randomPoint random;
		getRandomPoint(top_view_rgb, random);
		get3dRandomPoint(random,grasps);
	}
	else
	{
		printf("test10\n");
		getCornersToPush(top_view_rgb, res_corner);
		printf("test11\n");
		get3dPoints(res_corner, grasps);
		printf("test12\n");
	}

 		//convert grasps into poses
	std::vector<Eigen::Matrix4f> grasp_poses;
	convertPointNormalstoGraps(grasps, grasp_poses);

	BOOST_FOREACH(Eigen::Matrix4f& pose, grasp_poses){
		Eigen::Affine3d e2;
		e2.matrix() = pose.cast<double>();
		geometry_msgs::Pose p_msgs;

		tf::poseEigenToMsg(e2,p_msgs);

//DEBUG
 			  //tf::Transform grasp_tf;
 			  //geometry_msgs::PoseStamped msg;
 			  //msg.pose=p_msgs;
 			  //msg.header.frame_id="base_link";
 			  //msg.header.stamp=ros::Time::now();
 			  //msg.header.seq=0;
 			  //pub.publish(msg);

		res.corner_poses.push_back(p_msgs);
	}

 		//do the same for convex
	std::swap(res_corner.corner, res_corner.corner_convex);
	std::swap(res_corner.push_direction, res_corner.push_direction_convex);

	pcl::PointCloud<pcl::PointNormal> grasps_convex;
	get3dPoints(res_corner, grasps_convex);
	std::vector<Eigen::Matrix4f> grasp_poses_convex;
	convertPointNormalstoGraps(grasps_convex, grasp_poses_convex);

	BOOST_FOREACH(Eigen::Matrix4f& pose, grasp_poses_convex){
		Eigen::Affine3d e2;
		e2.matrix() = pose.cast<double>();
		geometry_msgs::Pose p_msgs;
		tf::poseEigenToMsg(e2,p_msgs);
		res.corner_poses_convex.push_back(p_msgs);
	}

	res.header.frame_id = base_frame;
	res.header.stamp = ros::Time::now();

	return true;

}


bool getTopView(cv::Mat& topview){
	sensor_msgs::PointCloud2ConstPtr cloud_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(point_cloud, ros::Duration(5.0));
	pcl::PointCloud<Point> cloud, cloud_in;
	pcl::fromROSMsg<Point>(*cloud_msg, cloud_in);

	ROS_INFO("got cloud with %d points in %s",cloud_in.points.size(), cloud_in.header.frame_id.c_str() );

	pcl::PassThrough<Point> pass;
	pass.setInputCloud (cloud_in.makeShared());
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (-0.35, 0.35);
		  //pass.setFilterLimitsNegative (true);
	pass.filter (cloud);

          //showPointClound (cloud.makeShared(),"111");
		//get self mask
//		camera_self_filter::mask servicecall;
//		servicecall.request.header.frame_id = cloud.header.frame_id;
//		servicecall.request.header.stamp = cloud.header.stamp;
//		_self_mask_client.call(servicecall);
//		sensor_msgs::ImageConstPtr selfMaskPtr = boost::make_shared<sensor_msgs::Image>(servicecall.response.mask_image);
//		IplImage* ipl_self = _bridge.imgMsgToCv(selfMaskPtr, "passthrough");
//		cv::Mat temp(ipl_self);
//		cv::Mat self_mask = temp.clone();

		//get all transforms


	tf::StampedTransform tf_realcam_in_base;

		//listener_.waitForTransform(base_frame,  head_mount_kinect_ir_optical_frame, ros::Time(0), ros::Duration(5.0));
		//listener_.lookupTransform(base_frame,  cloud_msg->header.frame_id, ros::Time(0), tf_realcam_in_base);

	int flag=0;

	while(!flag){
		

		try{
			listener_.waitForTransform(base_frame,  "head_mount_kinect_rgb_optical_frame", ros::Time(0), ros::Duration(5.0));
			listener_.lookupTransform(base_frame,  "head_mount_kinect_rgb_optical_frame", ros::Time(0), tf_realcam_in_base);
			flag=1;
		}
		catch (tf::TransformException ex){
                   // cerr<<ex.what()<<endl;
		}
	}





	cout<<"base_frame "<<base_frame<<endl;
	printf("=============\n");

	cout<<"cloud_msg->header.frame_id "<<cloud_msg->header.frame_id<<endl;

	printf("*************\n");


	cout<<"tf_realcam_in_base"<<endl<<tf_realcam_in_base<<endl;

		//transform cloud into baselink
	pcl::PointCloud<Point> cloud_in_virt_cam;

	tf::Transform full_tf = tf_virtual_cam.inverse() * tf_virtual_cam_transl.inverse() * tf_realcam_in_base;

	cout<<"full_tf"<<endl<<full_tf<<endl;

	Eigen::Affine3d transform_eigen;
	tf::transformTFToEigen(full_tf,transform_eigen );
	Eigen::Matrix4d transform_eigen3(transform_eigen.matrix());
	Eigen::Matrix4f transform_eigen3f = transform_eigen3.cast<float>();
	pcl::transformPointCloud(  cloud, cloud_in_virt_cam, transform_eigen3f );






	cloud_in_virt_cam.header.frame_id = "virtual_cam_frame";
	cloud_in_virt_cam.header.stamp = cloud.header.stamp;

//		//set ROI
//		double distance = tf_gripper_left_in_base.getOrigin().getX();
//		ROS_INFO("distance %d", distance);
//		roi_filter.setRoi(distance - _tolerance_x, distance+ _tolerance_x,
//				tf_gripper_right_in_base.getOrigin().getY() - _tolerance_y, tf_gripper_left_in_base.getOrigin().getY() + _tolerance_y,
//				tf_gripper_left_in_base.getOrigin().getZ() - _max_height_z, tf_gripper_left_in_base.getOrigin().getZ() + 0.03);
//
//		pcl::PointCloud<Point> cloud_in_base_roi_filtered;
//		roi_filter.apply(cloud_in_base, cloud_in_base_roi_filtered);
//
//		//transform cloud into camera_frame for projection
//		pcl::PointCloud<Point> cloud_in_cam_roi_filtered;
//		pcl_ros::transformPointCloud( _cam_info->header.frame_id, cloud_in_base_roi_filtered, cloud_in_cam_roi_filtered, listener_);
//		ROS_INFO("got cloud with %d after roi points",cloud_in_cam_roi_filtered.points.size() );




// DEBUG
//		  Eigen::Vector4f min, max;
//
//		  pcl::getMinMax3D(cloud_in_virt_cam, min, max);
//
//		  ROS_INFO_STREAM("min max"<<min<<"\n  max"<<max);

			/* pass.setInputCloud (cloud_in_virt_cam.makeShared());
			 pass.setFilterFieldName ("z");
			std::cerr<<table_height<<std::endl;
			 pass.setFilterLimits (0.0, virtual_cam_z - this->table_height); //table height - 10cm
			  //pass.setFilterLimitsNegative (true);
			 pcl::PointCloud<Point> cloud_temp;

			 pass.filter (cloud_temp);
			 cloud_in_virt_cam = cloud_temp;*/


            // showPointClound (cloud_in_virt_cam.makeShared(),"123");



		//estimate table plane
			 coefficients = boost::shared_ptr<pcl::ModelCoefficients>(new pcl::ModelCoefficients());
			 pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	  // Create the segmentation object
			 pcl::SACSegmentation<Point> seg;
	  // Optional
			 seg.setOptimizeCoefficients (false);
	  // Mandatory
			 seg.setModelType (pcl::SACMODEL_PLANE);
			 seg.setMethodType (pcl::SAC_RANSAC);
			 seg.setDistanceThreshold (0.015);

			 seg.setInputCloud (cloud_in_virt_cam.makeShared ());
			 seg.segment (*inliers, *coefficients);


			 std::cerr << "Model coefficients: " << coefficients->values[0] << " "
			 << coefficients->values[1] << " "
			 << coefficients->values[2] << " "
			 << coefficients->values[3] << std::endl;





//	pcl::io::savePCDFile("cloud_in_virt_cam.pcd", cloud_in_virt_cam);

//	return 0;

	  //extract table
			 pcl::PointCloud<Point> cloud_table_in_virt_cam;
			 pcl::ExtractIndices<Point> extract;
			 extract.setInputCloud (cloud_in_virt_cam.makeShared());
			 extract.setIndices (inliers);
			 extract.setNegative (false);
			 extract.filter (cloud_table_in_virt_cam);

	  //extract everything else
			 pcl::PointCloud<Point> cloud_objects_in_virt_cam;
			 extract.setNegative (true);
			 extract.filter (cloud_objects_in_virt_cam);


			 ROS_INFO("size table points %d size object points %d", cloud_table_in_virt_cam.points.size(), cloud_objects_in_virt_cam.points.size());
//	  pcl::io::savePCDFile("cloud_objects_in_virt_cam.pcd", cloud_objects_in_virt_cam);


 //showPointClound (cloud_objects_in_virt_cam.makeShared(),"333");


		//project

	    //all
			 cv::Mat mask = cv::Mat::zeros(cv::Size(_cam_info->width, _cam_info->height), CV_8U);
			 cv::Mat mask_cont = cv::Mat::zeros(cv::Size(_cam_info->width, _cam_info->height), CV_8U);

			 cv::Mat maskRGB = cv::Mat::zeros(cv::Size(_cam_info->width, _cam_info->height), CV_8UC3);
			 ROS_INFO("picel_coords %f %f %f", cloud.points[0].x, cloud.points[0].y,cloud.points[0].z );
			 vector<Point, Eigen::aligned_allocator<Point> >::iterator iter;
			 for (iter = cloud_in_virt_cam.points.begin(); iter != cloud_in_virt_cam.points.end(); ++iter){
			 	Point& point = *iter;

			 	if (isnan(point.x) || isnan(point.y) || isnan(point.z))
			 		continue;
			 	cv::Point3d p3d(point.x, point.y, point.z);
			 	cv::Point2d p2d;
			 	p2d = _model.project3dToPixel(p3d);
			 	int x = round(p2d.x);
			 	int y = round(p2d.y);
			 	if((x>mask.cols-1) || (x<0) || (y>mask.rows-1) || (y<0))
			 		continue;
			 	mask.at<unsigned char>(y, x) = 255;
			 	cv::circle(maskRGB, cv::Point(x,y), 3, cv::Scalar(point.b, point.g, point.r), -1);
    		//DEBUG
//    		maskRGB.at<unsigned char>(y, 3 * x) = point.b;
//    		maskRGB.at<unsigned char>(y, 3 * x + 1) = point.g;
//    		maskRGB.at<unsigned char>(y, 3 * x + 2) = point.r;
			 }

		//objects
			 cv::Mat mask_objects = cv::Mat::zeros(cv::Size(_cam_info->width, _cam_info->height), CV_8U);
			 for (iter = cloud_objects_in_virt_cam.points.begin(); iter != cloud_objects_in_virt_cam.points.end(); ++iter){
			 	Point& point = *iter;

			 	if (isnan(point.x) || isnan(point.y) || isnan(point.z))
			 		continue;
			 	cv::Point3d p3d(point.x, point.y, point.z);
			 	cv::Point2d p2d;
			 	p2d = _model.project3dToPixel(p3d);
			 	int x = round(p2d.x);
			 	int y = round(p2d.y);
			 	if((x>mask_objects.cols-1) || (x<0) || (y>mask_objects.rows-1) || (y<0))
			 		continue;
			 	mask_objects.at<unsigned char>(y, x) = 255;
			 	cv::circle(maskRGB, cv::Point(x,y), 3, cv::Scalar(point.b, point.g, point.r), -1);
    		//DEBUG
//    		maskRGB.at<unsigned char>(y, 3 * x) = point.b;
//    		maskRGB.at<unsigned char>(y, 3 * x + 1) = point.g;
//    		maskRGB.at<unsigned char>(y, 3 * x + 2) = point.r;
			 }

		//filter object parts with normals aligned with table normal

			 Eigen::Vector3f table_normal;
			 table_normal << coefficients->values[0], coefficients->values[1], coefficients->values[2];
			 table_normal.normalize();

			 cv::Mat mask_object_tops = cv::Mat::zeros(cv::Size(_cam_info->width, _cam_info->height), CV_8U);

			 pcl::NormalEstimation<Point, pcl::Normal> ne;
			 ne.setInputCloud (cloud_objects_in_virt_cam.makeShared());

			 pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>);
			 ne.setSearchMethod (tree);
			 pcl::PointCloud<pcl::Normal> cloud_normals;
			 ne.setRadiusSearch (0.01);
			 ne.compute (cloud_normals);


			 pcl::PointCloud<pcl::Normal>::iterator normal_iter = cloud_normals.points.begin();
			 for (iter = cloud_objects_in_virt_cam.points.begin(); iter != cloud_objects_in_virt_cam.points.end(); ++iter, ++normal_iter){
			 	Point& point = *iter;
			 	pcl::Normal&  normal = *normal_iter;
			 	float dot_prod = std::abs(table_normal.dot(normal.getNormalVector3fMap()));
    		//DEBUG
//			std::cout<<" table normal \n"<<table_normal<<"\n  object normal \n"<<normal.getNormalVector3fMap();
//			printf("\n dp %f dp_thresh %f \n", dot_prod, dot_threshold);
			 	if (dot_prod < dot_threshold){
			 		continue;
			 	}

			 	if (isnan(point.x) || isnan(point.y) || isnan(point.z))
			 		continue;
			 	cv::Point3d p3d(point.x, point.y, point.z);
			 	cv::Point2d p2d;
			 	p2d = _model.project3dToPixel(p3d);
			 	int x = round(p2d.x);
			 	int y = round(p2d.y);
			 	if((x>mask_object_tops.cols-1) || (x<0) || (y>mask_object_tops.rows-1) || (y<0))
			 		continue;
			 	mask_object_tops.at<unsigned char>(y, x) = 255;
			 	cv::circle(maskRGB, cv::Point(x,y), 3, cv::Scalar(point.b, point.g, point.r), -1);
    		//DEBUG
//    		maskRGB.at<unsigned char>(y, 3 * x) = point.b;
//    		maskRGB.at<unsigned char>(y, 3 * x + 1) = point.g;
//    		maskRGB.at<unsigned char>(y, 3 * x + 2) = point.r;
			 }






		//table


			 cv::Mat mask_table = cv::Mat::zeros(cv::Size(_cam_info->width, _cam_info->height), CV_8U);
			 for (iter = cloud_table_in_virt_cam.points.begin(); iter != cloud_table_in_virt_cam.points.end(); ++iter){
			 	Point& point = *iter;

			 	if (isnan(point.x) || isnan(point.y) || isnan(point.z))
			 		continue;
			 	cv::Point3d p3d(point.x, point.y, point.z);
			 	cv::Point2d p2d;
			 	p2d = _model.project3dToPixel(p3d);
			 	int x = round(p2d.x);
			 	int y = round(p2d.y);
			 	if((x>mask_table.cols-1) || (x<0) || (y>mask_table.rows-1) || (y<0))
			 		continue;
			 	mask_table.at<unsigned char>(y, x) = 127;
			 	cv::circle(maskRGB, cv::Point(x,y), 3, cv::Scalar(point.b, point.g, point.r), -1);
    		//DEBUG
//    		maskRGB.at<unsigned char>(y, 3 * x) = point.b;
//    		maskRGB.at<unsigned char>(y, 3 * x + 1) = point.g;
//    		maskRGB.at<unsigned char>(y, 3 * x + 2) = point.r;
			 }



#if X_OUTPUT

			 cv::namedWindow("test", 1);
			 cv::waitKey();

			 cv::imshow("mask", mask);

			 cv::waitKey();

			 cv::imshow("mask_table", mask_table);

			 std::cerr<<"mask table"<<std::endl;		

			 cv::waitKey();

			 cv::imshow("mask_objects", mask_objects);

			 cv::waitKey();

			 cv::imshow("mask_object_tops", mask_object_tops);
			 
			 std::cerr<<"before morphology"<<std::endl;		
			 
			 cv::waitKey();


		//DEBUG

		//cv::imshow("test", mask);

//		cv::namedWindow("testrgb", 1);
//		cv::imshow("testrgb", maskRGB);
#endif

// BUG MAKE PARAM
			 cv::morphologyEx(mask_object_tops,mask_object_tops,CV_MOP_CLOSE , getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)), cv::Point(-1,-1), this->morphology_param);
		// it can be done in other way:
//		cv::dilate(mask_object_tops,mask_object_tops,getStructuringElement(cv::MORPH_RECT, cv::Size(5,5)) );
//		cv::erode(mask_object_tops, mask_object_tops, cv::Mat(), cv::Point(-1, -1), 3 );


			 mask = cv::max(mask_table, mask_object_tops);

#if X_OUTPUT


			 cv::imshow("mask_object_tops1", mask_object_tops);
			 cv::waitKey();
#endif
		//DEBUG
//		cv::dilate(self_mask,self_mask,getStructuringElement(cv::MORPH_RECT, cv::Size(5,5)) );
//#if X_OUTPUT
//		cv::imshow("test", self_mask);
//		cv::waitKey();
//#endif
//		cv::bitwise_and(mask, self_mask, mask);


		//search for largest contour

			 std::vector<std::vector<cv::Point> > contours;
			 std::vector<cv::Vec4i> hierarchy;
			 cv::findContours(mask_object_tops, contours, hierarchy, CV_RETR_CCOMP,
			 	CV_CHAIN_APPROX_SIMPLE);

			 if (hierarchy.empty()){
			 	ROS_ERROR("no contours found");
			 	return false;
			 }

			 bool found = false;
			 int max_contour = 0;
			 double max_contour_area = 0;

			 const double contour_min_area_ = 100;
			 for (int i=0; i < hierarchy.size(); i = hierarchy[i][0])
			 {
			 	double contour_area = cv::contourArea(cv::Mat(contours[i]));
			 	if (contour_area > contour_min_area_  && contour_area > max_contour_area)
			 	{
			 		found = true;
			 		max_contour_area = contour_area;
			 		max_contour = i;
			 	}
			 }

			 if (found)
			 {

			 	cv::drawContours(mask_cont, contours, max_contour, cv::Scalar(255), CV_FILLED,
			 		8, hierarchy, 0);



			 }
			 else
			 	cout<<" not found"<<endl;

#if X_OUTPUT
			 cv::imshow("mask_cont", mask_cont);
			 cv::waitKey();
#endif

			 printf("test8\n");
			 cv::imwrite("test.png", mask_cont);


			 printf("test9\n");
		/*IplImage ipl_image = mask_cont;
		//cv::Mat ipl2mat(ipl_image);
		cv::Mat ipl2mat(cv::cvarrToMat(&ipl_image));
		//sensor_msgs::ImagePtr img_msg = sensor_msgs::CvBridge::cvToImgMsg(&ipl_image);
	
		//--- solution 1: 
		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr->image = ipl2mat;
		sensor_msgs::ImagePtr img_msg = cv_ptr->toImageMsg();
		printf("test9\n");
		//--- solution 2:
		//cv_bridge::CvImage cvi;
		//cvi.image = ipl_image;
		//sensor_msgs::Image im_msg;
		//cvi.toImageMsg(im_msg);
		
		img_msg->header.frame_id = _cam_info->header.frame_id;
		img_msg->header.stamp = cloud.header.stamp;*/
		topview = mask_cont;

		cv::imshow("topview1", topview);
		cv::waitKey();

		return true;



	}

//	function to get the corners from another service

	bool getCornersToPush(cv::Mat& topview, interactive_segmentation_textured::cornerFind::Response& res){
		interactive_segmentation_textured::cornerFind::Request req;
		IplImage temp(topview);
	//cv::Mat ipl2mat(temp);
		cv::Mat ipl2mat(cv::cvarrToMat(&temp));

		std::cout<<ipl2mat.channels()<<std::endl;

		cv::imshow("topview2", topview);
		cv::waitKey();

		printf("test10-0\n");
	//sensor_msgs::ImagePtr imgptr  = sensor_msgs::CvBridge::cvToImgMsg(&temp);
	/*cv_bridge::CvImagePtr cv_ptr;

	cv_ptr.reset (new cv_bridge::CvImage);

     printf("test10-1\n");

	//cv_ptr->image = ipl2mat;
     cv_ptr->image = cv::imread("/home/hao/test.png");
    printf("test10-1-1\n");

	cv::imshow("cv_ptr->image", cv_ptr->image);
	cv::waitKey();


   //cv_ptr->image.covertTo(v_ptr->image, CV_8UC3);
   //cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_GRAY2BGR);


    sensor_msgs::ImageConstPtr imgptr = cv_ptr->toImageMsg();


	printf("test10-2\n");
    	
	req.image = *imgptr;*/







    /*cv_bridge::CvImagePtr cv_ptr;
    cv_ptr.reset (new cv_bridge::CvImage);
   
    cv_ptr->image = cv::imread("/home/hao/test.png");

    cv_ptr->encoding = "bgr8";*/

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr.reset (new cv_bridge::CvImage);
    
    cv_ptr->image = ipl2mat;

    cv_ptr->encoding = "mono8";
    

    cv::imshow("cv_ptr->image", cv_ptr->image);
    cv::waitKey();

    sensor_msgs::ImageConstPtr imgptr = cv_ptr->toImageMsg();

    req.image = *imgptr;
    
    //image_pub_.publish(cv_ptr->toImageMsg());


    //test========================  
    
	/*cv_bridge::CvImagePtr cv_ptr_test;
    try
  	{
  	 cv_ptr_test = cv_bridge::toCvCopy(*imgptr);
     }
 	 catch (cv_bridge::Exception& e)
  	 {
         ROS_ERROR("cv_bridge exception: %s", e.what());
     }
    cv::imshow("hhh", cv_ptr_test->image);
    cv::waitKey();*/

    _corner_finder.call(req, res);

    printf("getCornersToPush\n");

    return true;
}


bool get3dPoints(const interactive_segmentation_textured::cornerFind::Response& res, pcl::PointCloud<pcl::PointNormal>& grasp_points){
	Eigen::Vector3f table_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);

	//create Points
	grasp_points.reserve(res.corner.size());
	grasp_points.width = res.corner.size();
	grasp_points.height = 1;
	for (int i = 0; i< res.corner.size() ; ++i){
		cv::Point2d p2d(res.corner[i].x, res.corner[i].y);
		cv::Point3d p3d;

//		get the ray of the pinhole camera
		p3d = _model.projectPixelTo3dRay(p2d);
		Eigen::Vector3f ray(p3d.x, p3d.y, p3d.z);

//		distance to the corner to push from the virtual camera
		float t = -1.0 * coefficients->values[3] / (table_normal.dot(ray));

//		vector to the corner
		Eigen::Vector3f intersec = t * ray;

//		do the same for the next point that is the point + push

		p2d = cv::Point2d(res.corner[i].x + this->push_distance * res.push_direction[i].x, res.corner[i].y + this->push_distance * res.push_direction[i].y);
		p3d = _model.projectPixelTo3dRay(p2d);
		ray = Eigen::Vector3f(p3d.x, p3d.y, p3d.z);
		t = -1.0 * coefficients->values[3] / (table_normal.dot(ray));

//		vector between the corner and the corner + push direction
		Eigen::Vector3f normal = t * ray - intersec;
		normal.normalize();

//		put the corner with the direction of push into the point cloud
		pcl::PointNormal p;
		p.getArray3fMap() = intersec;
//DEBUG
		//cout<<"intersec"<<endl<<intersec<<endl;

		p.getNormalVector3fMap() = normal;
		grasp_points.push_back(p);

	}


//	transform it to the base link frame
	Eigen::Affine3d transform_eigen;
	tf::Transform tf_full = tf_virtual_cam_transl * tf_virtual_cam;
	tf::transformTFToEigen(tf_full ,transform_eigen );
	Eigen::Matrix4d transform_eigen3(transform_eigen.matrix());
	Eigen::Matrix4f transform_eigen3f = transform_eigen3.cast<float>();

	pcl::PointCloud<pcl::PointNormal> grasp_points_in_base;
	pcl::transformPointCloudWithNormals(  grasp_points, grasp_points_in_base, transform_eigen3f );

	grasp_points_in_base.header.frame_id = base_frame;
	grasp_points_in_base.header.stamp = ros::Time::now();

	grasp_points = grasp_points_in_base;

	printf("get3dPoints\n");


	return true;
}


bool get3dRandomPoint(randomPoint& random, pcl::PointCloud<pcl::PointNormal>& grasp_points){
	Eigen::Vector3f table_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);

	//create Points
	grasp_points.width = 1;
	grasp_points.height = 1;

	cv::Point2d p2d(random.push_point.x,random.push_point.y);
	cv::Point3d p3d;

//		get the ray of the pinhole camera
	p3d = _model.projectPixelTo3dRay(p2d);
	Eigen::Vector3f ray(p3d.x, p3d.y, p3d.z);

//		distance to the corner to push from the virtual camera
	float t = -1.0 * coefficients->values[3] / (table_normal.dot(ray));

//		vector to the corner
	Eigen::Vector3f intersec = t * ray;

//		do the same for the next point that is the point + push
	int x=random.push_point.x;
	float xdirection=random.direction[0];
	int y=random.push_point.y;
	float ydirection=random.direction[1];

	p2d = cv::Point2d(x+this->push_distance*xdirection,y+this->push_distance*ydirection);
	p3d = _model.projectPixelTo3dRay(p2d);
	ray = Eigen::Vector3f(p3d.x, p3d.y, p3d.z);
	t = -1.0 * coefficients->values[3] / (table_normal.dot(ray));

//		vector between the corner and the corner + push direction
	Eigen::Vector3f normal = t * ray - intersec;
	normal.normalize();

//		put the corner with the direction of push into the point cloud
	pcl::PointNormal p;
	p.getArray3fMap() = intersec;
//DEBUG
		//cout<<"intersec"<<endl<<intersec<<endl;

	p.getNormalVector3fMap() = normal;
	grasp_points.push_back(p);



//	transform it to the base link frame
	Eigen::Affine3d transform_eigen;
	tf::Transform tf_full = tf_virtual_cam_transl * tf_virtual_cam;
	tf::transformTFToEigen(tf_full ,transform_eigen );
	Eigen::Matrix4d transform_eigen3(transform_eigen.matrix());
	Eigen::Matrix4f transform_eigen3f = transform_eigen3.cast<float>();

	pcl::PointCloud<pcl::PointNormal> grasp_points_in_base;
	pcl::transformPointCloudWithNormals(  grasp_points, grasp_points_in_base, transform_eigen3f );

	grasp_points_in_base.header.frame_id = base_frame;
	grasp_points_in_base.header.stamp = ros::Time::now();

	grasp_points = grasp_points_in_base;


	return true;
}



// to get the 4 x 4 matrix of the pose
bool convertPointNormalstoGraps(pcl::PointCloud<pcl::PointNormal>& cloud, std::vector<Eigen::Matrix4f>& poses){
	poses.reserve(cloud.points.size());
	BOOST_FOREACH(pcl::PointNormal& p, cloud.points){
		Eigen::Matrix4f pose;
//		position
		pose.block<3,1>(0,3) = p.getVector3fMap();
//		x rotation
		pose.block<3,1>(0,0) = p.getNormalVector3fMap();
//		get y direction if the dot product is 1 then take the other one
		Eigen::Vector3f orth(0,1,0);
		if ( std::abs(orth.dot(pose.block<3,1>(0,0))) > 0.9 ){
			orth = Eigen::Vector3f(1,0,0);
		}
//		compute the z direction
		pose.block<3,1>(0,2) = orth.cross(pose.block<3,1>(0,0));
		pose.block<3,1>(0,1) = pose.block<3,1>(0,2).cross(pose.block<3,1>(0,0));
		poses.push_back(pose);
	}

	return true;

}





};

int main(int argc, char** argv){

	ros::init(argc, argv, "poke_point_finder_node");

	printf("test0\n");

	PokePointFinder hcs;
	ros::spin();

	return 0;
}



