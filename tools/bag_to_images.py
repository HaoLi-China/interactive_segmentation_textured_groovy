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
#  \author Dejan Pangercic
#  

#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib; roslib.load_manifest('interactive_segmentation_textured')
import rosbag
from cv_bridge import CvBridge, CvBridgeError
import os
import sys
import shutil
import cv

def grab(file, folder, topic):
   bridge = CvBridge()
   if file.endswith('.bag'):
      bag = rosbag.Bag(file,'r')
      index = 1
      for topic, msg, t in bag.read_messages(topics=[topic]):
        try:
           cv_image = bridge.imgmsg_to_cv(msg, "bgr8")
           image_name = os.path.join(folder, str("%.4d" % index) + '.png')
           index = index + 1
           cv.SaveImage(image_name, cv_image)
           print "saved %s" % image_name
        except CvBridgeError, e:
           print e

if __name__ == '__main__':
    
   print "sys.argv.__len__", sys.argv.__len__()
   if sys.argv.__len__() != 4:
      print "usage bag_to_images.py <bag> <image folder> <topic>"
      sys.exit(1)
   grab(sys.argv[1], sys.argv[2], sys.argv[3])
