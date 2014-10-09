; Auto-generated. Do not edit!


(cl:in-package interactive_segmentation_textured-srv)


;//! \htmlinclude depthImage-request.msg.html

(cl:defclass <depthImage-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass depthImage-request (<depthImage-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <depthImage-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'depthImage-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name interactive_segmentation_textured-srv:<depthImage-request> is deprecated: use interactive_segmentation_textured-srv:depthImage-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <depthImage-request>) ostream)
  "Serializes a message object of type '<depthImage-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <depthImage-request>) istream)
  "Deserializes a message object of type '<depthImage-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<depthImage-request>)))
  "Returns string type for a service object of type '<depthImage-request>"
  "interactive_segmentation_textured/depthImageRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'depthImage-request)))
  "Returns string type for a service object of type 'depthImage-request"
  "interactive_segmentation_textured/depthImageRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<depthImage-request>)))
  "Returns md5sum for a message object of type '<depthImage-request>"
  "b2c36e91badbc08644feff566a2abdfe")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'depthImage-request)))
  "Returns md5sum for a message object of type 'depthImage-request"
  "b2c36e91badbc08644feff566a2abdfe")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<depthImage-request>)))
  "Returns full string definition for message of type '<depthImage-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'depthImage-request)))
  "Returns full string definition for message of type 'depthImage-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <depthImage-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <depthImage-request>))
  "Converts a ROS message object to a list"
  (cl:list 'depthImage-request
))
;//! \htmlinclude depthImage-response.msg.html

(cl:defclass <depthImage-response> (roslisp-msg-protocol:ros-message)
  ((depth_image
    :reader depth_image
    :initarg :depth_image
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image)))
)

(cl:defclass depthImage-response (<depthImage-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <depthImage-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'depthImage-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name interactive_segmentation_textured-srv:<depthImage-response> is deprecated: use interactive_segmentation_textured-srv:depthImage-response instead.")))

(cl:ensure-generic-function 'depth_image-val :lambda-list '(m))
(cl:defmethod depth_image-val ((m <depthImage-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interactive_segmentation_textured-srv:depth_image-val is deprecated.  Use interactive_segmentation_textured-srv:depth_image instead.")
  (depth_image m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <depthImage-response>) ostream)
  "Serializes a message object of type '<depthImage-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'depth_image) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <depthImage-response>) istream)
  "Deserializes a message object of type '<depthImage-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'depth_image) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<depthImage-response>)))
  "Returns string type for a service object of type '<depthImage-response>"
  "interactive_segmentation_textured/depthImageResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'depthImage-response)))
  "Returns string type for a service object of type 'depthImage-response"
  "interactive_segmentation_textured/depthImageResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<depthImage-response>)))
  "Returns md5sum for a message object of type '<depthImage-response>"
  "b2c36e91badbc08644feff566a2abdfe")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'depthImage-response)))
  "Returns md5sum for a message object of type 'depthImage-response"
  "b2c36e91badbc08644feff566a2abdfe")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<depthImage-response>)))
  "Returns full string definition for message of type '<depthImage-response>"
  (cl:format cl:nil "sensor_msgs/Image depth_image~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'depthImage-response)))
  "Returns full string definition for message of type 'depthImage-response"
  (cl:format cl:nil "sensor_msgs/Image depth_image~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <depthImage-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'depth_image))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <depthImage-response>))
  "Converts a ROS message object to a list"
  (cl:list 'depthImage-response
    (cl:cons ':depth_image (depth_image msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'depthImage)))
  'depthImage-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'depthImage)))
  'depthImage-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'depthImage)))
  "Returns string type for a service object of type '<depthImage>"
  "interactive_segmentation_textured/depthImage")