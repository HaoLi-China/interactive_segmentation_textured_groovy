#
#  Copyright (c) 2011, Robert Bosch LLC
#  All rights reserved.
# 
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions
#   are met:
# 
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above
#      copyright notice, this list of conditions and the following
#      disclaimer in the documentation and/or other materials provided
#      with the distribution.
#    * Neither the name of Robert Bosch LLC nor the names of its
#      contributors may be used to endorse or promote products derived
#      from this software without specific prior written permission.
# 
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#   POSSIBILITY OF SUCH DAMAGE.
# 
# 
#
#
#  \author Christian Bersch
#  

#! /usr/bin/env python

import roslib
roslib.load_manifest('interactive_segmentation_textured')
import rospy

import numpy

import sys

# import the necessary things for OpenCV
import cv
import cv_bridge
import geometry_msgs.msg
import sensor_msgs.msg
import interactive_segmentation_textured.srv
import tf.transformations as tft
import tf
import simple_robot_control
import camera_self_filter.srv
from geometry_msgs.msg import PoseStamped
from track_features_lk_class import *



class GraspExecuter():

    def __init__(self):
        
        self.mask_service_name = "/self_mask"
        self.camera_topic = "kinect/rgb/image_color"
        self.camera_topic = "prosilica/image_rect_color"
#DEBUG
        #self.pub = rospy.Publisher('execute_grasps/last_pos', PoseStamped)


#        print "waiting for findPokePose service"
        rospy.wait_for_service("findPokePose", 10.0)
        self._find_poke_point = rospy.ServiceProxy("findPokePose", interactive_segmentation_textured.srv.cornerPokePoseFind())
        print "waiting for %s service" % self.mask_service_name
        rospy.wait_for_service(self.mask_service_name, 10.0)
        self._getMask = rospy.ServiceProxy(self.mask_service_name, camera_self_filter.srv.mask())
        self.armr = simple_robot_control.Arm('r')
        self.arml = simple_robot_control.Arm('l')
        self.arm = None
        self.ft  = FeatureTracker()
        self.br =  cv_bridge.CvBridge()
        self.forward_push_increment = 0.015#0.005
        self.listener = tf.TransformListener()
        self.arm_using = 0
        self.tilt_angle = [0.5, 0.5]
        self.gripper_frmaes = ['l_gripper_tool_frame','r_gripper_tool_frame']
        seed_angles_right = (-0.93743667222127069, 0.11056515201626564, -2.064627263335348, -1.4934701151950738, -6.7198768798611708, -0.45140012228925586, 14.224670047444075)
        seed_angles_left = (0.54066820655626213, 0.057655477736996884, 2.1336739049387785, -1.449537220909964, 6.5549140761711637, -0.41655446503198001, -1.5841374868194875)
        self.seed_angles = [seed_angles_left, seed_angles_right]
        
        seed_angles_right2 = (-0.93743667222127069, 0.11056515201626564, -2.064627263335348, -1.4934701151950738, -6.7198768798611708, -0.45140012228925586, 14.224670047444075)
        seed_angles_left2 = (0.54066820655626213, 0.057655477736996884, 2.1336739049387785, -1.449537220909964, 6.5549140761711637, -0.41655446503198001, -1.5841374868194875)
        self.seed_angles2 = [seed_angles_left2, seed_angles_right2]
        self.head = simple_robot_control.Head()
        self.gripr = simple_robot_control.Gripper('r')
        self.gripl = simple_robot_control.Gripper('l')
	self.above=0.02#2cm above the table
        
        
        
    def goToPregrasp(self, side = 2, wait = True):
        angles_left = (1.605, 0.321, 1.297, -1.694, 0.988, -0.376, 12.053)
        angles_right = (-1.571, 0.374, -1.105, -1.589, -1.119, -0.276, 0.537)
        self.gripr.closeGripper(wait = False)
        self.gripl.closeGripper(wait = False)
        if side == 0:
            self.arml.goToAngle(angles_left, 5.0 , wait)
        if side == 1:
            self.armr.goToAngle(angles_right, 5.0 , wait)
        if side == 2:
            self.arml.goToAngle(angles_left, 5.0 , False)
            self.armr.goToAngle(angles_right, 5.0 , wait)
        self.head.lookAtPoint(1,0,0, 'base_link')
        
    
        
    def _getPokePose(self, pos, orient):

        print orient
        if numpy.dot(tft.quaternion_matrix(orient)[0:3,2], (0,0,1)) < 0:
            print "fliiping arm pose"
            orient = tft.quaternion_multiply(orient, (1,0,0,0))
        tilt_adjust = tft.quaternion_about_axis(self.tilt_angle[self.arm_using], (0,1,0))
        new_orient = tft.quaternion_multiply(orient, tilt_adjust)
        pos[2] += self.above #push above cm above table 
#DEBUG
        #print new_orient

        return (pos, orient, new_orient)
    
    def _findBestPose(self,poses, poses_convex):
	print "**********CONCAVE ONES***************",len(poses)
	print "**********CONVEX ONES***************",len(poses_convex)
        target_direction = numpy.array([1.0, 0.0, 0.0, 0.0])
        def findAlign(pose):
            orient = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
            rot_mat = tft.quaternion_matrix(orient)
#DEBUG
            #print "orient",orient
            #print "rot_mat[:,0]",rot_mat[:,0]
            #print "target_direction", target_direction
# look for the corners that are aligned with x direction
            dp = numpy.dot(rot_mat[:,0], target_direction)
            #print "dp", dp
            return dp
        found_concave = True
        if len(poses) > 0:
            sorted_poses = sorted(poses, key = findAlign, reverse = True)
            #print 'sorted_poses: ', sorted_poses, sorted_poses[0]
            if findAlign(sorted_poses[0]) < 0:
                found_concave = False
        else:
            found_concave = False
                
        if not found_concave:
            print "did not find concave corner = trying convex***********************************************************"
            sorted_poses = sorted(poses_convex, key = findAlign, reverse = True)
            if findAlign(sorted_poses[0]) < 0:
                print "no reachable corner whatsoever found"

        return sorted_poses


    def _getImageAndMask(self):
        img_msg = rospy.wait_for_message(self.camera_topic, sensor_msgs.msg.Image, 5.0 )
        mask_res = self._getMask(img_msg.header)
        frame = self.br.imgmsg_to_cv(img_msg)
        mask = self.br.imgmsg_to_cv(mask_res.mask_image)
        return (frame,mask)
        
        

            
    def _getInitFeatures(self):
        (frame, mask) = self._getImageAndMask()
        return self.ft.initFirstFrame(frame, mask)
    
    def _getTrackedFeatures(self):
        (frame, mask) = self._getImageAndMask()
        (features, object_feature_sets) = self.ft.track(frame, mask)
        print "found %d objects" % (len(object_feature_sets) - 1)
        return features
    
    def _process_features(self, last_features, new_features):
        #pca stuff...
        pass
    
    def _initialPush(self):
        poses_res = self._find_poke_point()
        poses = self._findBestPose(poses_res.corner_poses, poses_res.corner_poses_convex)
        pose = poses[0]


	
        print "moving arm to pose", pose
        
        pos = list((pose.position.x, pose.position.y, pose.position.z))
	
        orient = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
	#DEBUG
        #print "pose******************************************", pos, orient
              



        #select arm
# just depends on the direction 
        rot_mat = tft.quaternion_matrix(orient)
# or try this rot_mat with looking for corners
        if numpy.dot(rot_mat[0:3,0], (0,1,0)) > 0:
            print "using right arm"
            self.arm = self.armr
            self.arm_using = 1
            
        else:
            print "using left arm"
            self.arm = self.arml
            self.arm_using = 0
        
        
        (pos,pre_orient, orient) = self._getPokePose(pos, orient)
        
        #go to pre pose
        pre_pos = numpy.dot(tft.quaternion_matrix(pre_orient), (-0.1,0.0,0.0,0.0))[:3] + pos + (0,0,0.1)
	#DEBUG
        #print "prepose", pre_pos, pre_orient, 
        

            
        #backing up offset
        offs = numpy.dot(tft.quaternion_matrix(orient), (-0.07,0.0,0.0,0.0))[:3] 
        offs[2] = 0.0
        pos = pos + offs
        
        pre_pos2 = numpy.dot(tft.quaternion_matrix(orient), (-0.05,0.0,0.0,0.0))[:3] + pos
      
        
        pre_pos_0 = pre_pos + numpy.array([0.0,0.0,0.7])
       
        seed_angles = self.seed_angles[self.arm_using]
#DEBUG
        #print 'pre_pos: ', pre_pos


#DEBUG
	#pos_msg=geometry_msgs.msg.PoseStamped()
	#pos_msg.pose.position.x=pos[0]
	#pos_msg.pose.position.y=pos[1]
	#pos_msg.pose.position.z=pos[2]
	#pos_msg.pose.orientation.x=orient[0]
	#pos_msg.pose.orientation.y=orient[1]
	#pos_msg.pose.orientation.z=orient[2]
	#pos_msg.pose.orientation.w=orient[3]
	#pos_msg.header.frame_id = 'base_link'
	#pos_msg.header.stamp =rospy.Time(0)
	#pos_msg.header.seq=0;
	#self.pub.publish(pos_msg)


        self.arm.moveGripperToPose(pre_pos, pre_orient, 'base_link', 2.0, True, seed_angles)
        seed_angles = self.seed_angles2[self.arm_using]
#DEBUG
        #print 'pre_pos2: ', pre_pos2
        #print 'orientation2' ,orient
        self.arm.moveGripperToPose(pre_pos2, orient, 'base_link',2.0, True, seed_angles)
        seed_angles = self.seed_angles2[self.arm_using]
        #print 'pos: ', pos
        self.arm.moveGripperToPose(pos, orient, 'base_link',2.0, True, seed_angles)
        
        
    
    def _push(self, orient_new = None):
        pos,orient = self.listener.lookupTransform('base_link',self.gripper_frmaes[self.arm_using], rospy.Time())
        pos_update = tft.quaternion_matrix(orient)[0:3,0] * self.forward_push_increment 
        pos_update[2] = 0.0
        pos_new = pos + pos_update
        if orient_new == None:
            orient_new = orient
        self.arm.moveGripperToPose(pos_new, orient_new, 'base_link', 1.0)
         
    

    def run(self):
        
        self.goToPregrasp(2)
        last_features = self._getInitFeatures()
        self.ft.image
        self._initialPush()
        self._push()
        
        term_krit = False
        counter = 0
        while not term_krit:
            new_features = self._getTrackedFeatures()
            self._process_features(last_features, new_features)
            self._push()
            last_features = new_features
            counter += 1
            term_krit = counter > 15
            
        #run grasping     
        
            
    


        

            

        

if __name__ == '__main__':
    rospy.init_node('grasp_executer')
    rospy.sleep(1.0)
    rospy.spin()

        

                
        
                
                
                
                
        
        
        
        

    

