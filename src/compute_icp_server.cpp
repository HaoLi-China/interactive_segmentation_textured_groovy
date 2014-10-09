/*
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
  * \author Dejan Pangercic
  */

#include "ros/ros.h"
#include "interactive_segmentation_textured/computeICP.h"
#include <eigen_conversions/eigen_msg.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/console/time.h>
typedef pcl::PointXYZ PointT;

bool computeICP(interactive_segmentation_textured::computeICP::Request  &req,
         interactive_segmentation_textured::computeICP::Response &res )
{
  pcl::PointCloud<PointT> input, target;
  pcl::IterativeClosestPointNonLinear<PointT, PointT> icp;
  icp.setMaximumIterations(2000);
  icp.setTransformationEpsilon (1e-8);
  std::cerr << "[compute_icp_server:] icp.getTransformationEpsilon () " << icp.getTransformationEpsilon () << std::endl;
  std::cerr << "[compute_icp_server:] icp.getMaximumIterations() " << icp.getMaximumIterations () << std::endl;
  std::cerr << "[compute_icp_server:] icp.getMaxCorrespondenceDistance () " << icp.getMaxCorrespondenceDistance () << std::endl;

  pcl::fromROSMsg (req.input, input);
  pcl::fromROSMsg (req.target, target);
  icp.setInputCloud(input.makeShared());
  icp.setInputTarget(target.makeShared());
  pcl::PointCloud<PointT> Final;
  pcl::console::TicToc tictoc;
  tictoc.tic();
  icp.align(Final);
  std::cerr << "[compute_icp_server:] has converged: " << icp.hasConverged() << " in " << tictoc.toc() << " seconds." <<
    std:: endl << "score: " << icp.getFitnessScore() << std::endl;
  std::cerr << icp.getFinalTransformation() << std::endl;
  geometry_msgs::Pose transform;
  Eigen::Affine3d e2;
  e2.matrix() = icp.getFinalTransformation().cast<double>();
  tf::poseEigenToMsg (e2, transform);
  res.transform = transform;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "compute_icp_server");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("compute_icp_server", computeICP);
  ros::spin();
  return 0;
}
