#include <ros/ros.h>

#include <iostream>
#include <fstream>
#include <cv.h>
#include <Eigen/Eigen>
#include <cmath>
#include <string>
#include <geometry_msgs/Pose2D.h>
#include <opencv2/core/eigen.hpp>
#include <eigen_conversions/eigen_msg.h>
//#include "cv_bridge/CvBridge.h"

//#include <tf/transform_listener.h>
//#include "LinearMath/btVector3.h"

using namespace std;

class PushEvaluator
{

private:
    int num_objects_;
    int num_features_;      ///No. of features to be spawned on each object
    int num_pushes_;        ///In each round i.e. starting from a given contact point
    int num_rounds_;
    int num_tracking_steps;
    static const float residual_error_comp = 0.1; // 0.1; //deduct half a pixel from the residuals at each timestep
    //vector<cv::Point2f> obj_dims;

    vector<geometry_msgs::Pose2D> gripper_pose_;
    vector<geometry_msgs::Pose2D> initial_object_poses_;
    vector< vector<geometry_msgs::Pose2D> > ros_object_poses_;
    vector< vector<geometry_msgs::Pose2D> > ros_object_transforms_;
    vector<cv::Point2f> object_dims;

    //vector< vector<vector<cv::Point2f> > > feature_poses_;      /// Indexed by push number, then object id
    vector<vector<cv::Point2f> > initial_feature_poses_;        ///Indexed by object id
    vector<cv::Point2f> all_initial_feature_poses_;             ///Features of all objects concatenated
    vector<vector<cv::Point2f> > corner_trajectories_;          ///Indexed by push number
    std::vector<std::vector<cv::Point2f> > corner_trajectories_filtered_and_transposed;

public:
    PushEvaluator()
    {
        num_objects_ = 0;
        num_features_ = 300;
        object_dims.resize(4);         ///Max no. of objects in our simulations is 4

        ///Assuming all box objects

        ///empty1.world
        object_dims[0].x = 0.2;
        object_dims[0].y = 0.1;
        object_dims[1].x = 0.1;
        object_dims[1].y = 0.1;
        object_dims[2].x = 0.08;
        object_dims[2].y = 0.2;
        object_dims[3].x = 0.07;
        object_dims[3].y = 0.11;

 /*       ///empty2.world
        object_dims[0].x = 0.15;
        object_dims[0].y = 0.1;
        object_dims[1].x = 0.1;
        object_dims[1].y = 0.1;
        object_dims[2].x = 0.08;
        object_dims[2].y = 0.2;
        object_dims[3].x = 0.07;
        object_dims[3].y = 0.08;
*/
    }

    int read_object_poses(char *filename)
    {
        ifstream file(filename);
        if (!file.is_open())
        {
            cout<< "Unable to open file.";
            return -1;
        }

        string line;
        int time_step = 0;
        int round = 0;
        int n = 0;
        int npush = 0;
        //int k = 0;
        while (!file.eof())
        {
            getline(file,line);
//            ROS_INFO("line: %s", line.c_str());
            if (line.find("INITIAL POSES OF THE OBJECTS") != string::npos)
                continue;
            else if (line.find("INITIAL OBJECT") != string::npos)
            {
                /// Initial object pose
                string s = line.substr(15,1);
                //int obj_id = atoi(s.c_str()); //line[7];
//              ROS_INFO("Found OBJECT with id = %d", obj_id);
                n++;

                geometry_msgs::Pose2D pose;
                stringstream ss, rs;
                size_t pos1 = line.find_first_of('[');
                size_t pos2 = line.find_first_of(']', pos1+1);
                string pos_substr = line.substr(pos1+1, pos2-pos1-1);
//               ROS_INFO("pos_substr = %s", pos_substr.c_str());
                ss << pos_substr;
                ss >> pose.x;
                ss >> pose.y;
                //ss >> pose.position.z;
                size_t pos3 = line.find_first_of('[', pos2+1);
                size_t pos4 = line.find_first_of(']', pos3+1);
                string rot_substr = line.substr(pos3+1, pos4-pos3-1);
//                ROS_INFO("rot_substr = %s", rot_substr.c_str());
                rs << rot_substr;
                rs >> pose.theta;

                //vector<geometry_msgs::Pose2D> v;
                //v.push_back(pose);
                initial_object_poses_.push_back(pose);
                num_objects_ = n;
                ROS_INFO("num_ob = %d", num_objects_);
            }
            else if (line.find("Push") != string::npos)
            {
                /// Push line
                string s = line.substr(9,1);
                time_step = atoi(s.c_str()); //line[10];
                npush++;
//                ROS_INFO("Found PUSH at time step %d", time_step);
            }
            else if (line.find("GRIPPER POS") != string::npos)
            {
                /// Gripper pose
//                ROS_INFO("Found GRIPPER");
                geometry_msgs::Pose2D pose;
                stringstream ss, rs;
                size_t pos1 = line.find_first_of('[');
                size_t pos2 = line.find_first_of(']', pos1+1);
                string pos_substr = line.substr(pos1+1, pos2-pos1-1);
                ss << pos_substr;
                ss >> pose.x;
                ss >> pose.y;
                //ss >> pose.position.z;
                size_t pos3 = line.find_first_of('[', pos2+1);
                size_t pos4 = line.find_first_of(']', pos3+1);
                string rot_substr = line.substr(pos3+1, pos4-pos3-1);
                rs << rot_substr;
                rs >> pose.theta;

                gripper_pose_.push_back(pose);
            }
            else if (line.find("OBJECT") != string::npos)
            {
                /// Object pose
                string s = line.substr(7,1);
                int obj_id = atoi(s.c_str()); //line[7];
//                ROS_INFO("Found OBJECT with id = %d", obj_id);

                geometry_msgs::Pose2D pose;
                stringstream ss, rs;
                size_t pos1 = line.find_first_of('[');
                size_t pos2 = line.find_first_of(']', pos1+1);
                string pos_substr = line.substr(pos1+1, pos2-pos1-1);
 //               ROS_INFO("pos_substr = %s", pos_substr.c_str());
                ss << pos_substr;
                ss >> pose.x;
                ss >> pose.y;
                //ss >> pose.position.z;
                size_t pos3 = line.find_first_of('[', pos2+1);
                size_t pos4 = line.find_first_of(']', pos3+1);
                string rot_substr = line.substr(pos3+1, pos4-pos3-1);
//              ROS_INFO("rot_substr = %s", rot_substr.c_str());
                rs << rot_substr;
                rs >> pose.theta;

                if (time_step == 1 && round == 1)
                {
                    vector<geometry_msgs::Pose2D> v;
                    v.push_back(pose);
                    ros_object_poses_.push_back(v);
                    //num_objects_ = n;
                    //ROS_INFO("num_ob = %d", num_objects_);
                }
                else
                    ros_object_poses_[obj_id].push_back(pose);

            }
            else if (line.find("Starting") != string::npos) {
                round++;
                npush = 0;
            } else
                continue;
        }

        //num_pushes_ = (ros_object_poses_[0].size())/round;
        num_pushes_ = npush;
        num_rounds_ = round;

        ROS_INFO("No. pushes = %d, No. of objects = %d, No. of rounds = %d", num_pushes_, num_objects_, num_rounds_);
  //      for (size_t i = 0; i < ros_object_poses_[0].size(); i++)
  //          ROS_INFO("Pos: %f %f, Orientation: %f", ros_object_poses_[0][i].x, ros_object_poses_[0][i].y, ros_object_poses_[0][i].theta);

        return 0;
    }

    void spawn_features()
    {
        ROS_INFO("Spawning features");
        initial_feature_poses_.resize(num_objects_);
        //feature_poses_.resize(num_objects_);
        //vector<cv::Point2f> all_initial_feature_poses;
        for (int obj_id = 0; obj_id < num_objects_; obj_id++)
        {
            vector<cv::Point2f> features;
            /// Spawn features for time step 0
            float xlimits[] = {initial_object_poses_[obj_id].x - 0.5*object_dims[obj_id].x, initial_object_poses_[obj_id].x + 0.5*object_dims[obj_id].x};
            float ylimits[] = {initial_object_poses_[obj_id].y - 0.5*object_dims[obj_id].y, initial_object_poses_[obj_id].y + 0.5*object_dims[obj_id].y};

            //float xlimits[] = {1.5*(obj_id+1), 4.3*(obj_id+1)};         ///Temp hack
            //float ylimits[] = {0.5*(obj_id+1), 1.1*(obj_id+1)};         ///Temp hack
            srand(unsigned(time(0)));
            features.resize(50);
            for (int n = 0; n < 50; n++)
            {
                cv::Point2f point;
                point.x = xlimits[0] + ((float)rand()/RAND_MAX)*(xlimits[1] - xlimits[0]);
                point.y = ylimits[0] + ((float)rand()/RAND_MAX)*(ylimits[1] - ylimits[0]);
                features[n] = point;
            }

            initial_feature_poses_[obj_id] = features;
            //feature_poses_[obj_id].push_back(features);
            all_initial_feature_poses_.insert(all_initial_feature_poses_.end(), features.begin(), features.end());
        }
        //all_initial_feature_poses_.push_back(all_feature_poses);
        for (size_t i = 0; i < initial_feature_poses_[0].size(); i++)
        {
    //        ROS_INFO("Pos: %f %f", initial_feature_poses_[0][i].x, initial_feature_poses_[0][i].y);
        }
    }

    void calc_new_feature_poses(int round)
    {
        ROS_INFO("Calculating new feature poses");
        if (corner_trajectories_.size() != 0)
            corner_trajectories_.clear();
        corner_trajectories_.resize(num_pushes_ + 1);
        corner_trajectories_[0] = all_initial_feature_poses_;
        /*
                ROS_INFO("Sanity Check");
                for (size_t i = 0; i < corner_trajectories_[0].size(); i++)
                    printf("%f %f ", corner_trajectories_[0][i].x, corner_trajectories_[0][i].y);
        */
        vector< vector<vector<cv::Point2f> > > feature_poses_;      /// Indexed by push number, then object id
        feature_poses_.resize(num_pushes_+1);
        feature_poses_[0].resize(num_objects_);
        for (int obj_id = 0; obj_id < num_objects_; obj_id++)
            feature_poses_[0][obj_id] = initial_feature_poses_[obj_id];

        for (int t = 1; t <= num_pushes_; t++)
        {
            vector<cv::Point2f> all_new_feature_poses;
            feature_poses_[t].resize(num_objects_);
//			ROS_INFO("Check 7");
            for (int obj_id = 0; obj_id < num_objects_; obj_id++)
            {
                vector<cv::Point2f> old_feature_poses, new_feature_poses;
                cv::Point2f translation;
                float theta;
                old_feature_poses = feature_poses_[t-1][obj_id];
                //ROS_INFO("Size of old feature poses for object %d = %zu", obj_id, old_feature_poses.size());

                if (t == 1)
                {
                    //old_feature_poses = initial_feature_poses_[obj_id];
                    translation.x = ros_object_poses_[obj_id][round*num_pushes_ + t].x - initial_object_poses_[obj_id].x;
                    translation.y = ros_object_poses_[obj_id][round*num_pushes_ + t].y - initial_object_poses_[obj_id].y;
                    theta = ros_object_poses_[obj_id][round*num_pushes_ + t].theta - initial_object_poses_[obj_id].theta;
//                   ROS_INFO("Check 4");
                }
                else
                {
                    translation.x = ros_object_poses_[obj_id][round*num_pushes_ + t].x - ros_object_poses_[obj_id][round*num_pushes_ + t-1].x;
                    translation.y = ros_object_poses_[obj_id][round*num_pushes_ + t].y - ros_object_poses_[obj_id][round*num_pushes_ + t-1].y;
                    theta = ros_object_poses_[obj_id][round*num_pushes_ + t].theta - ros_object_poses_[obj_id][round*num_pushes_ + t-1].theta;
                    //ROS_INFO("Check 5");
                }
                //ROS_INFO("Object %d: Translation: %f %f, theta = %f", obj_id, translation.x, translation.y, theta);
                new_feature_poses.resize(old_feature_poses.size());
                for (size_t n = 0; n < old_feature_poses.size(); n++)
                {
                    new_feature_poses[n].x = old_feature_poses[n].x*cos(theta) - old_feature_poses[n].y*sin(theta) + translation.x;
                    new_feature_poses[n].y = old_feature_poses[n].x*sin(theta) + old_feature_poses[n].y*cos(theta) + translation.y;
                }
                feature_poses_[t][obj_id] = new_feature_poses;
                //ROS_INFO("Size of new feature poses for object %d = %zu", obj_id, new_feature_poses.size());
                all_new_feature_poses.insert(all_new_feature_poses.end(), new_feature_poses.begin(), new_feature_poses.end());
                //ROS_INFO("push = %d, obj = %d, Size of all new feature poses = %zu", t, obj_id, all_new_feature_poses.size());
            }
            corner_trajectories_[t] = all_new_feature_poses;
        }

        ROS_INFO("Transposing corner trajectory");
        corner_trajectories_filtered_and_transposed.clear();
        corner_trajectories_filtered_and_transposed.resize(corner_trajectories_[0].size());
        //ROS_INFO("Size of c_t_ = %zu", corner_trajectories_.size());
        for (size_t i = 0; i < corner_trajectories_[0].size(); i++) {
            //ROS_INFO("Size of c_t_[0] = %zu", corner_trajectories_[0].size());
            std::vector<cv::Point2f> features_trans;
            features_trans.resize(corner_trajectories_.size());
            for (size_t j = 0; j < corner_trajectories_.size(); j++) {
                //ROS_INFO(" j = %zu, s = %zu", j, corner_trajectories_[j].size());
                features_trans[j] = corner_trajectories_[j][i];
            }
            //ROS_INFO("Size of c_t_f_a_t = %zu", corner_trajectories_filtered_and_transposed.size());
            corner_trajectories_filtered_and_transposed[i] = features_trans;
        }
    }

    /*** Top-level function:
     *  Samples a number of hypothesis sets, i.e. samples sets of possible feature clusters.
     *  Hypothesis sets are scored and best one is returned/visualized.
     */
    bool evaluateTrajectories()
    {
        ROS_INFO("Evaluating trajectories.");
        num_tracking_steps = num_pushes_;
        int num_samples = 15;
        ///  int num_hypotheses = 10;
        //filterTrajectories();
        Eigen::VectorXf scores = Eigen::VectorXf::Zero(num_samples);
        std::vector<std::vector<int> > feature_associations;
        std::vector<Eigen::VectorXf> features_per_hyp;

        //create hypothesis set samples
        for (int i = 0; i < num_samples; i++)
        {
            std::vector<int> feature_association;
            Eigen::VectorXf feature_per_hyp_count;
            scores[i] = sampleHypothesisSet(feature_association, feature_per_hyp_count);
            feature_associations.push_back(feature_association);
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
        std::vector<int>& final_feature_association = feature_associations[best_hypthesis_set_idx];
        Eigen::VectorXf& final_feature_per_hyp_count = features_per_hyp[best_hypthesis_set_idx];

        for (int j = 0; j < num_objects_; j++) {
            for (int i = j*50; i < (j+1)*50; i++)
                printf("%d ", final_feature_association[i]);
            printf("\n");
        }

        //DEBUG
        //cout<<"feature association size "<<feature_assocoations.size()<<endl;
        //cout<<"feature per hyp size "<<features_per_hyp.size()<<endl;

        final_feature_per_hyp_count[0] = 0;
        int best_single_hyp;
        final_feature_per_hyp_count.maxCoeff(&best_single_hyp);

        int idx_max_features;
        idx_max_features = best_single_hyp;
        int features_best_hyp;
        features_best_hyp = final_feature_per_hyp_count[best_single_hyp];

        /*
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
                */
        ROS_INFO("Exit Evaluating trajectories.");
        return true;

    }

    /***
     * Sample recursively possible feature-to-cluster associations (hypotheses) by generating a hypothesis
     * on the input features set, calculate outliers and generate another hypothesis, ... and so on until a
     * hypothesis exists for each feature.
     */
    float sampleHypothesisSet(std::vector<int>& feature_association,
                              Eigen::VectorXf& feature_per_hyp_count)
    {
        //ROS_INFO("Sampling hypothesis set");
        int num_hypotheses = 10;
        float mask_threshold = 80;

        Eigen::MatrixXf residualMatrix(corner_trajectories_filtered_and_transposed.size(), num_hypotheses);

        std::vector<std::vector<Eigen::Matrix3f> > hypothesis;

        //create identity Matrix as first hypothesos
        std::vector<Eigen::Matrix3f> RTs_I;
        RTs_I.resize(num_tracking_steps, Eigen::Matrix3f::Identity());
        //DEBUG
        //printf ("num_tracking_steps %d RTs_I.size() %d \n", num_tracking_steps, RTs_I.size());
        hypothesis.push_back(RTs_I);

        Eigen::VectorXf mask = Eigen::VectorXf::Ones(corner_trajectories_filtered_and_transposed.size());
        bool points_left;
        int i = 1;
        for (; i < num_hypotheses; i++)
        {
            Eigen::VectorXf residuals;
            std::vector<Eigen::Matrix3f>& last_hypothesis = hypothesis[i - 1];

            //calculate the residuals incurred by current hypothesis (i.e. error between features' actual RT and the hypothetical RT).
            calcResiduals(last_hypothesis,
                          corner_trajectories_filtered_and_transposed, residuals);

            //store residuals
            residualMatrix.col(i - 1) = residuals;

            //find inliers and mark them in binary mask
            Eigen::VectorXf mask_bin =
                (residuals.array() - mask_threshold).matrix();
            for (int i = 0; i < mask_bin.rows(); i++)
            {
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
            hypothesis.push_back(new_hypothesis);

        }
        //calc last residuals if all hypothesis had to be used
        int& actual_num_hypothesis = i;
        if (actual_num_hypothesis == num_hypotheses)
        {
            Eigen::VectorXf residuals;
            calcResiduals(hypothesis.back(),
                          corner_trajectories_filtered_and_transposed, residuals);
            residualMatrix.col(residualMatrix.cols() - 1) = residuals;
        }
        else
        {
            Eigen::MatrixXf residualMatrix2 = residualMatrix.block(0, 0,
                                              residualMatrix.rows(), actual_num_hypothesis);
            residualMatrix = residualMatrix2;
        }

        Eigen::VectorXf summed_residuals = residualMatrix.colwise().sum();
        Eigen::VectorXf summed_residuals_per_h = Eigen::VectorXf::Zero(
                    residualMatrix.cols());

        //Finally run the process again and assign each feature to its best possible RT hypothesis (or mark it as complete outlier).
        std::vector<int> best_feature_h_idx;
        best_feature_h_idx.resize(
            corner_trajectories_filtered_and_transposed.size(), 0);
        feature_per_hyp_count = Eigen::VectorXf::Ones(residualMatrix.cols());
        feature_per_hyp_count *= 0.0000001f;
        int num_outliers = 0;
        for (size_t i = 0; i < corner_trajectories_filtered_and_transposed.size(); i++)
        {
            int idx;
            residualMatrix.row(i).minCoeff(&idx);
            ///        float residual_value = residualMatrix(i, idx);

            //check if feature was an outlier
            if (mask[i] > 0.0f)
            {
                num_outliers++;
                best_feature_h_idx[i] = -1;
                continue;
            }

            best_feature_h_idx[i] = idx;
            feature_per_hyp_count[idx]++;
            summed_residuals_per_h[idx] += residualMatrix(i, idx);
        }

        //vis_image = vis_image_orig.clone();
        for (size_t i = 0; i < corner_trajectories_filtered_and_transposed.size(); i++)
        {
            int idx = best_feature_h_idx[i];
            if (idx >= 0 && feature_per_hyp_count[idx] > 5.1)
            {
            }
            else     //stupid feature
            {
                best_feature_h_idx[i] = -1;
            }
        }
        summed_residuals.array() /= feature_per_hyp_count.array();
        summed_residuals_per_h.array() /= feature_per_hyp_count.array();

        feature_association = best_feature_h_idx;
        ///    float outlier_penalty = num_outliers * 100.0;

        //ROS_INFO("Sampling hypothesis set");
        return summed_residuals_per_h.sum();
    }

    /***
     * generates a rotation translation trajectory hypothesis for all features marked in mask in a RANSAC like fashion.
     */
    bool generateHypothesis(const Eigen::VectorXf& mask,
                            std::vector<Eigen::Matrix3f>& RTs)
    {
       // ROS_INFO("Generating hypothesis");
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
        for (int i = 1; i < num_tracking_steps + 1; i++)
        {
            //DEBUG
            //printf("step gen %d \n", i);
            cv::Mat new_points(features_idx.size(), 1, CV_32FC2);
            cv::Mat old_points(features_idx.size(), 1, CV_32FC2);
            for (size_t j = 0; j < features_idx.size(); j++)
            {
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
     //   ROS_INFO("exit Generating hypothesis");
        return true;
    }

    /***
     * Calculates residuals as squared reprojection error between  rigid transform trajectory predicted by hypothesis and actual trajectory.
     */
    bool calcResiduals(const std::vector<Eigen::Matrix3f>& RTs,
                       const std::vector<std::vector<cv::Point2f> >& corner_trajectories_t,
                       Eigen::VectorXf& residuals)
    {
        //ROS_INFO("Calculating residuals");
        residuals = Eigen::VectorXf(corner_trajectories_t.size());
        for (unsigned int t = 0; t < corner_trajectories_t.size(); t++)
        {
            const std::vector<cv::Point2f>& current_trajectory =
                corner_trajectories_t[t];
//DEBUG
//			printf("RTs.size() %d current_trajectory.size() %d\n",RTs.size(),current_trajectory.size());
            assert(RTs.size() == current_trajectory.size() - 1);
            float residual = 0;
            for (unsigned int c = 0; c < current_trajectory.size() - 1; c++)
            {
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
        //ROS_INFO("Exit Calculating residuals");
        return true;
    }

    bool selectRandomFeatures(std::vector<int>& features_idx,
                              const Eigen::VectorXf& mask)
    {
        //ROS_INFO("Selecting random features");
        float variance = 200.0f * 200.0f; //pixel size of objects
        //select random but depending on gaussian distance
        //first create map
        int num_non_masked_elements = mask.sum();
        if (num_non_masked_elements < 3)
        {
            //DEBUG
            //printf("num non masked elements %d\n", num_non_masked_elements);
            return false;
        }
        int* map_array = new int[num_non_masked_elements];
        int counter = 0;
        for (int i = 0; i < mask.rows(); i++)
        {
            if (mask[i] > 0.0)
            {
                map_array[counter] = i;
                counter++;
            }

        }

        //we assume 3 points are selected...
        int idx1 = rand() % num_non_masked_elements;
        Eigen::VectorXf prob_dist(num_non_masked_elements);
        cv::Point2f current_point =
            corner_trajectories_filtered_and_transposed[map_array[idx1]].back();
        for (int i = 0; i < num_non_masked_elements; i++)
        {
            if (i == idx1)
            {
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
        if (normalizer < 0.0000000001)
        {
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
        //ROS_INFO("Exit Selecting random features");
        return true;
    }

    /**
     * Draw random bucket according probability distribution.
     */
    int draw(Eigen::VectorXf& prob_dist)
    {
        //ROS_INFO("Drawing random");
        float pos = (float) rand() / RAND_MAX;
        int idx = 0;
        float accumulator = 0.0f;
        for (;;)
        {
            accumulator += prob_dist[idx];
            if (accumulator > pos)
                break;
            idx++;
        }
        //ROS_INFO("Exit Drawing random");
        return idx;
    }

    /***
     * Retrives rotation and translation minimzing the error of the rigid transformation between two sets of points.
     *
     */
    cv::Mat cvEstimateRigidTransformFrom2(const cv::Mat& A, const cv::Mat& B)
    {
        //ROS_INFO("estimating rigid transform");
        cv::Mat M = cv::Mat(2, 3, CV_64F);

        CvPoint2D32f* a = (CvPoint2D32f*) A.data;
        CvPoint2D32f* b = (CvPoint2D32f*) B.data;
        CvMat matM = M;
        icvGetRTMatrix(a, b, 2, &matM);
        //ROS_INFO("exit estimating rigid transform");
        return M;
    }

    /***
    * Retrives rotation and translation minimzing the error of the rigid transformation between two sets of points.
    * Modified opencv version.
    */
    static void icvGetRTMatrix(const CvPoint2D32f* a, const CvPoint2D32f* b,
                               int count, CvMat* M)
    {
        /** Copied from https://code.ros.org/trac/opencv/browser/trunk/opencv/src/cv/cvlkpyramid.cpp?rev=2170  For license information please see the package LICENSE file
        **/

       // ROS_INFO("icv get matrix");
        double sa[16], sb[4], m[4], *om = M->data.db;
        CvMat A = cvMat(4, 4, CV_64F, sa), B = cvMat(4, 1, CV_64F, sb);
        CvMat MM = cvMat(4, 1, CV_64F, m);

        int i;

        memset(sa, 0, sizeof(sa));
        memset(sb, 0, sizeof(sb));

        for (i = 0; i < count; i++)
        {
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
        //ROS_INFO("exit icv get matrix");
    }


    int get_num_rounds()
    {
        return num_rounds_;
    }

    int get_num_pushes()
    {
        return num_pushes_;
    }

    vector<vector<cv::Point2f> > get_corner_trajectories()
    {
        return corner_trajectories_;
    }
};

int main(int argc, char **argv)
{
    //ros::init(argc, argv, "te"); //Init ROS
    //ros::NodeHandle n;
    if (argc != 2)
    {
        ROS_ERROR("Invalid call. Enter gripper log filename as argument.");
        return -1;
    }

    ROS_INFO("1. Filename: %s", argv[1]);
    PushEvaluator push;

    push.read_object_poses(argv[1]);
    push.spawn_features();
    for (int round = 0; round < push.get_num_rounds(); round++)
    {
        //push.calc_transforms_3d(round);
        push.calc_new_feature_poses(round);

        ///Send the corner_trajectories_ to be evaluated now
        push.evaluateTrajectories();

/*
        stringstream ss;
        ss << "features_log_" << round+1 << ".txt";
        ROS_INFO("Log file name: %s", (ss.str()).c_str());
        ofstream file((ss.str()).c_str());
        if (!file.is_open())
        {
            cout<< "Unable to open file.";
            return -1;
        }
        ROS_INFO("Writing to log file for round %d", round+1);
        vector<vector<cv::Point2f> > corner_trajectories = push.get_corner_trajectories();
        for (int t = 0; t < push.get_num_pushes(); t++)
        {
            file << "Push:";
            for (vector<cv::Point2f>::iterator it = corner_trajectories[t].begin(); it != corner_trajectories[t].end(); it++)
                file << it->x << " " << it->y << " ";

            file << "\n \n";
        }
        file.close();
*/
    }

    //ros::spin();
    return 0;
}

