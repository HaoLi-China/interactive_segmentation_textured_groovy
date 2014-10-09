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

#ifndef ROI_PCL_FILTER_H_
#define ROI_PCL_FILTER_H_


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>


template <typename PointT>
class ROI_Filter{
private:
	typedef pcl::PassThrough<PointT> PassThrough;
    typedef pcl::PointCloud<PointT> PointCloud;
//    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;
	PassThrough pass_x;
	PassThrough pass_y;
	PassThrough pass_z;
	Eigen::Vector4f min;
	Eigen::Vector4f max;

public:
	ROI_Filter(){}
	ROI_Filter(double min_x, double max_x, double min_y, double max_y, double min_z, double max_z){
		setRoi(min_x,  max_x,  min_y,  max_y,  min_z,  max_z);
	}
	void setRoi(double min_x, double max_x, double min_y, double max_y, double min_z, double max_z){
		pass_x.setFilterFieldName ("x");
		pass_x.setFilterLimits (min_x,max_x);
		pass_y.setFilterFieldName ("y");
		pass_y.setFilterLimits (min_y,max_y);
		pass_z.setFilterFieldName ("z");
		pass_z.setFilterLimits (min_z,max_z);
		min.x() = min_x;
		min.y() = min_y;
		min.z() = min_z;
		max.x() = max_x;
		max.y() = max_y;
		max.z() = max_z;
	}

	void setRoi(Eigen::Matrix<double,3,2>& box ){

		Eigen::Vector3d lower = box.col(0).cwiseMin(box.col(1));
		Eigen::Vector3d upper = box.col(0).cwiseMax(box.col(1));

//		std::cout << "lower\n" << lower <<std::endl;
//		std::cout << "upper\n" << upper <<std::endl;

		pass_x.setFilterFieldName ("x");
		pass_x.setFilterLimits (lower.x(),upper.x());
		pass_y.setFilterFieldName ("y");
		pass_y.setFilterLimits (lower.y(), upper.y());
		pass_z.setFilterFieldName ("z");
		pass_z.setFilterLimits (lower.z(),upper.z());

		min.block<3,1>(0,0) = lower.cast<float>();
		max.block<3,1>(0,0) = upper.cast<float>();
	}



	void apply (const PointCloud& cloud_in, PointCloud& cloud_out ){
		pcl::PointIndices indices;
		pcl::getPointsInBox(cloud_in,min,max,indices.indices);
		pcl::ExtractIndices<PointT> extract;
		extract.setInputCloud(boost::make_shared<pcl::PointCloud<PointT> >(cloud_in) );
		extract.setIndices(boost::make_shared<pcl::PointIndices>(indices));
		extract.filter(cloud_out);
	}

	unsigned int getNumInROI(const PointCloud& cloud_in){
		pcl::PointIndices indices;
		pcl::getPointsInBox(cloud_in,min,max,indices.indices);
		return indices.indices.size();
	}

};


#endif /* ROI_PCL_FILTER_H_ */
