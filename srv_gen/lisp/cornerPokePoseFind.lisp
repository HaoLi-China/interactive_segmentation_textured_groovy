; Auto-generated. Do not edit!


(cl:in-package interactive_segmentation_textured-srv)


;//! \htmlinclude cornerPokePoseFind-request.msg.html

(cl:defclass <cornerPokePoseFind-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass cornerPokePoseFind-request (<cornerPokePoseFind-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <cornerPokePoseFind-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'cornerPokePoseFind-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name interactive_segmentation_textured-srv:<cornerPokePoseFind-request> is deprecated: use interactive_segmentation_textured-srv:cornerPokePoseFind-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <cornerPokePoseFind-request>) ostream)
  "Serializes a message object of type '<cornerPokePoseFind-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <cornerPokePoseFind-request>) istream)
  "Deserializes a message object of type '<cornerPokePoseFind-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<cornerPokePoseFind-request>)))
  "Returns string type for a service object of type '<cornerPokePoseFind-request>"
  "interactive_segmentation_textured/cornerPokePoseFindRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cornerPokePoseFind-request)))
  "Returns string type for a service object of type 'cornerPokePoseFind-request"
  "interactive_segmentation_textured/cornerPokePoseFindRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<cornerPokePoseFind-request>)))
  "Returns md5sum for a message object of type '<cornerPokePoseFind-request>"
  "ec2e96cd2342dee4d7f51d1dcb4733b7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'cornerPokePoseFind-request)))
  "Returns md5sum for a message object of type 'cornerPokePoseFind-request"
  "ec2e96cd2342dee4d7f51d1dcb4733b7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<cornerPokePoseFind-request>)))
  "Returns full string definition for message of type '<cornerPokePoseFind-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'cornerPokePoseFind-request)))
  "Returns full string definition for message of type 'cornerPokePoseFind-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <cornerPokePoseFind-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <cornerPokePoseFind-request>))
  "Converts a ROS message object to a list"
  (cl:list 'cornerPokePoseFind-request
))
;//! \htmlinclude cornerPokePoseFind-response.msg.html

(cl:defclass <cornerPokePoseFind-response> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (corner_poses
    :reader corner_poses
    :initarg :corner_poses
    :type (cl:vector geometry_msgs-msg:Pose)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Pose :initial-element (cl:make-instance 'geometry_msgs-msg:Pose)))
   (corner_poses_convex
    :reader corner_poses_convex
    :initarg :corner_poses_convex
    :type (cl:vector geometry_msgs-msg:Pose)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Pose :initial-element (cl:make-instance 'geometry_msgs-msg:Pose))))
)

(cl:defclass cornerPokePoseFind-response (<cornerPokePoseFind-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <cornerPokePoseFind-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'cornerPokePoseFind-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name interactive_segmentation_textured-srv:<cornerPokePoseFind-response> is deprecated: use interactive_segmentation_textured-srv:cornerPokePoseFind-response instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <cornerPokePoseFind-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interactive_segmentation_textured-srv:header-val is deprecated.  Use interactive_segmentation_textured-srv:header instead.")
  (header m))

(cl:ensure-generic-function 'corner_poses-val :lambda-list '(m))
(cl:defmethod corner_poses-val ((m <cornerPokePoseFind-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interactive_segmentation_textured-srv:corner_poses-val is deprecated.  Use interactive_segmentation_textured-srv:corner_poses instead.")
  (corner_poses m))

(cl:ensure-generic-function 'corner_poses_convex-val :lambda-list '(m))
(cl:defmethod corner_poses_convex-val ((m <cornerPokePoseFind-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interactive_segmentation_textured-srv:corner_poses_convex-val is deprecated.  Use interactive_segmentation_textured-srv:corner_poses_convex instead.")
  (corner_poses_convex m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <cornerPokePoseFind-response>) ostream)
  "Serializes a message object of type '<cornerPokePoseFind-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'corner_poses))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'corner_poses))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'corner_poses_convex))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'corner_poses_convex))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <cornerPokePoseFind-response>) istream)
  "Deserializes a message object of type '<cornerPokePoseFind-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'corner_poses) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'corner_poses)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Pose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'corner_poses_convex) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'corner_poses_convex)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Pose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<cornerPokePoseFind-response>)))
  "Returns string type for a service object of type '<cornerPokePoseFind-response>"
  "interactive_segmentation_textured/cornerPokePoseFindResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cornerPokePoseFind-response)))
  "Returns string type for a service object of type 'cornerPokePoseFind-response"
  "interactive_segmentation_textured/cornerPokePoseFindResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<cornerPokePoseFind-response>)))
  "Returns md5sum for a message object of type '<cornerPokePoseFind-response>"
  "ec2e96cd2342dee4d7f51d1dcb4733b7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'cornerPokePoseFind-response)))
  "Returns md5sum for a message object of type 'cornerPokePoseFind-response"
  "ec2e96cd2342dee4d7f51d1dcb4733b7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<cornerPokePoseFind-response>)))
  "Returns full string definition for message of type '<cornerPokePoseFind-response>"
  (cl:format cl:nil "Header header~%geometry_msgs/Pose[] corner_poses~%geometry_msgs/Pose[] corner_poses_convex~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'cornerPokePoseFind-response)))
  "Returns full string definition for message of type 'cornerPokePoseFind-response"
  (cl:format cl:nil "Header header~%geometry_msgs/Pose[] corner_poses~%geometry_msgs/Pose[] corner_poses_convex~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <cornerPokePoseFind-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'corner_poses) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'corner_poses_convex) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <cornerPokePoseFind-response>))
  "Converts a ROS message object to a list"
  (cl:list 'cornerPokePoseFind-response
    (cl:cons ':header (header msg))
    (cl:cons ':corner_poses (corner_poses msg))
    (cl:cons ':corner_poses_convex (corner_poses_convex msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'cornerPokePoseFind)))
  'cornerPokePoseFind-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'cornerPokePoseFind)))
  'cornerPokePoseFind-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cornerPokePoseFind)))
  "Returns string type for a service object of type '<cornerPokePoseFind>"
  "interactive_segmentation_textured/cornerPokePoseFind")