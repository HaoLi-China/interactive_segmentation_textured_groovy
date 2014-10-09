; Auto-generated. Do not edit!


(cl:in-package interactive_segmentation_textured-srv)


;//! \htmlinclude computeICP-request.msg.html

(cl:defclass <computeICP-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2))
   (target
    :reader target
    :initarg :target
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2)))
)

(cl:defclass computeICP-request (<computeICP-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <computeICP-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'computeICP-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name interactive_segmentation_textured-srv:<computeICP-request> is deprecated: use interactive_segmentation_textured-srv:computeICP-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <computeICP-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interactive_segmentation_textured-srv:input-val is deprecated.  Use interactive_segmentation_textured-srv:input instead.")
  (input m))

(cl:ensure-generic-function 'target-val :lambda-list '(m))
(cl:defmethod target-val ((m <computeICP-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interactive_segmentation_textured-srv:target-val is deprecated.  Use interactive_segmentation_textured-srv:target instead.")
  (target m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <computeICP-request>) ostream)
  "Serializes a message object of type '<computeICP-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'target) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <computeICP-request>) istream)
  "Deserializes a message object of type '<computeICP-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'target) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<computeICP-request>)))
  "Returns string type for a service object of type '<computeICP-request>"
  "interactive_segmentation_textured/computeICPRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'computeICP-request)))
  "Returns string type for a service object of type 'computeICP-request"
  "interactive_segmentation_textured/computeICPRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<computeICP-request>)))
  "Returns md5sum for a message object of type '<computeICP-request>"
  "5202812facca6cd2d14b2d6bd8203e30")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'computeICP-request)))
  "Returns md5sum for a message object of type 'computeICP-request"
  "5202812facca6cd2d14b2d6bd8203e30")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<computeICP-request>)))
  "Returns full string definition for message of type '<computeICP-request>"
  (cl:format cl:nil "sensor_msgs/PointCloud2 input~%sensor_msgs/PointCloud2 target~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'computeICP-request)))
  "Returns full string definition for message of type 'computeICP-request"
  (cl:format cl:nil "sensor_msgs/PointCloud2 input~%sensor_msgs/PointCloud2 target~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <computeICP-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'target))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <computeICP-request>))
  "Converts a ROS message object to a list"
  (cl:list 'computeICP-request
    (cl:cons ':input (input msg))
    (cl:cons ':target (target msg))
))
;//! \htmlinclude computeICP-response.msg.html

(cl:defclass <computeICP-response> (roslisp-msg-protocol:ros-message)
  ((transform
    :reader transform
    :initarg :transform
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass computeICP-response (<computeICP-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <computeICP-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'computeICP-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name interactive_segmentation_textured-srv:<computeICP-response> is deprecated: use interactive_segmentation_textured-srv:computeICP-response instead.")))

(cl:ensure-generic-function 'transform-val :lambda-list '(m))
(cl:defmethod transform-val ((m <computeICP-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interactive_segmentation_textured-srv:transform-val is deprecated.  Use interactive_segmentation_textured-srv:transform instead.")
  (transform m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <computeICP-response>) ostream)
  "Serializes a message object of type '<computeICP-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'transform) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <computeICP-response>) istream)
  "Deserializes a message object of type '<computeICP-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'transform) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<computeICP-response>)))
  "Returns string type for a service object of type '<computeICP-response>"
  "interactive_segmentation_textured/computeICPResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'computeICP-response)))
  "Returns string type for a service object of type 'computeICP-response"
  "interactive_segmentation_textured/computeICPResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<computeICP-response>)))
  "Returns md5sum for a message object of type '<computeICP-response>"
  "5202812facca6cd2d14b2d6bd8203e30")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'computeICP-response)))
  "Returns md5sum for a message object of type 'computeICP-response"
  "5202812facca6cd2d14b2d6bd8203e30")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<computeICP-response>)))
  "Returns full string definition for message of type '<computeICP-response>"
  (cl:format cl:nil "geometry_msgs/Pose transform~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'computeICP-response)))
  "Returns full string definition for message of type 'computeICP-response"
  (cl:format cl:nil "geometry_msgs/Pose transform~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <computeICP-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'transform))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <computeICP-response>))
  "Converts a ROS message object to a list"
  (cl:list 'computeICP-response
    (cl:cons ':transform (transform msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'computeICP)))
  'computeICP-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'computeICP)))
  'computeICP-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'computeICP)))
  "Returns string type for a service object of type '<computeICP>"
  "interactive_segmentation_textured/computeICP")