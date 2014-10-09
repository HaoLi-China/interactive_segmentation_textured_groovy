; Auto-generated. Do not edit!


(cl:in-package interactive_segmentation_textured-srv)


;//! \htmlinclude cornerFind-request.msg.html

(cl:defclass <cornerFind-request> (roslisp-msg-protocol:ros-message)
  ((image
    :reader image
    :initarg :image
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image)))
)

(cl:defclass cornerFind-request (<cornerFind-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <cornerFind-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'cornerFind-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name interactive_segmentation_textured-srv:<cornerFind-request> is deprecated: use interactive_segmentation_textured-srv:cornerFind-request instead.")))

(cl:ensure-generic-function 'image-val :lambda-list '(m))
(cl:defmethod image-val ((m <cornerFind-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interactive_segmentation_textured-srv:image-val is deprecated.  Use interactive_segmentation_textured-srv:image instead.")
  (image m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <cornerFind-request>) ostream)
  "Serializes a message object of type '<cornerFind-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <cornerFind-request>) istream)
  "Deserializes a message object of type '<cornerFind-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<cornerFind-request>)))
  "Returns string type for a service object of type '<cornerFind-request>"
  "interactive_segmentation_textured/cornerFindRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cornerFind-request)))
  "Returns string type for a service object of type 'cornerFind-request"
  "interactive_segmentation_textured/cornerFindRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<cornerFind-request>)))
  "Returns md5sum for a message object of type '<cornerFind-request>"
  "ac8d58fdb334d1d395d0c60410ea3eb1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'cornerFind-request)))
  "Returns md5sum for a message object of type 'cornerFind-request"
  "ac8d58fdb334d1d395d0c60410ea3eb1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<cornerFind-request>)))
  "Returns full string definition for message of type '<cornerFind-request>"
  (cl:format cl:nil "sensor_msgs/Image image~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'cornerFind-request)))
  "Returns full string definition for message of type 'cornerFind-request"
  (cl:format cl:nil "sensor_msgs/Image image~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <cornerFind-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <cornerFind-request>))
  "Converts a ROS message object to a list"
  (cl:list 'cornerFind-request
    (cl:cons ':image (image msg))
))
;//! \htmlinclude cornerFind-response.msg.html

(cl:defclass <cornerFind-response> (roslisp-msg-protocol:ros-message)
  ((corner
    :reader corner
    :initarg :corner
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (push_direction
    :reader push_direction
    :initarg :push_direction
    :type (cl:vector geometry_msgs-msg:Point32)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point32 :initial-element (cl:make-instance 'geometry_msgs-msg:Point32)))
   (corner_convex
    :reader corner_convex
    :initarg :corner_convex
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (push_direction_convex
    :reader push_direction_convex
    :initarg :push_direction_convex
    :type (cl:vector geometry_msgs-msg:Point32)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point32 :initial-element (cl:make-instance 'geometry_msgs-msg:Point32))))
)

(cl:defclass cornerFind-response (<cornerFind-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <cornerFind-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'cornerFind-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name interactive_segmentation_textured-srv:<cornerFind-response> is deprecated: use interactive_segmentation_textured-srv:cornerFind-response instead.")))

(cl:ensure-generic-function 'corner-val :lambda-list '(m))
(cl:defmethod corner-val ((m <cornerFind-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interactive_segmentation_textured-srv:corner-val is deprecated.  Use interactive_segmentation_textured-srv:corner instead.")
  (corner m))

(cl:ensure-generic-function 'push_direction-val :lambda-list '(m))
(cl:defmethod push_direction-val ((m <cornerFind-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interactive_segmentation_textured-srv:push_direction-val is deprecated.  Use interactive_segmentation_textured-srv:push_direction instead.")
  (push_direction m))

(cl:ensure-generic-function 'corner_convex-val :lambda-list '(m))
(cl:defmethod corner_convex-val ((m <cornerFind-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interactive_segmentation_textured-srv:corner_convex-val is deprecated.  Use interactive_segmentation_textured-srv:corner_convex instead.")
  (corner_convex m))

(cl:ensure-generic-function 'push_direction_convex-val :lambda-list '(m))
(cl:defmethod push_direction_convex-val ((m <cornerFind-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interactive_segmentation_textured-srv:push_direction_convex-val is deprecated.  Use interactive_segmentation_textured-srv:push_direction_convex instead.")
  (push_direction_convex m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <cornerFind-response>) ostream)
  "Serializes a message object of type '<cornerFind-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'corner))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'corner))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'push_direction))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'push_direction))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'corner_convex))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'corner_convex))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'push_direction_convex))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'push_direction_convex))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <cornerFind-response>) istream)
  "Deserializes a message object of type '<cornerFind-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'corner) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'corner)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'push_direction) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'push_direction)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point32))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'corner_convex) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'corner_convex)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'push_direction_convex) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'push_direction_convex)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point32))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<cornerFind-response>)))
  "Returns string type for a service object of type '<cornerFind-response>"
  "interactive_segmentation_textured/cornerFindResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cornerFind-response)))
  "Returns string type for a service object of type 'cornerFind-response"
  "interactive_segmentation_textured/cornerFindResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<cornerFind-response>)))
  "Returns md5sum for a message object of type '<cornerFind-response>"
  "ac8d58fdb334d1d395d0c60410ea3eb1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'cornerFind-response)))
  "Returns md5sum for a message object of type 'cornerFind-response"
  "ac8d58fdb334d1d395d0c60410ea3eb1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<cornerFind-response>)))
  "Returns full string definition for message of type '<cornerFind-response>"
  (cl:format cl:nil "geometry_msgs/Point[] corner~%geometry_msgs/Point32[] push_direction~%geometry_msgs/Point[] corner_convex~%geometry_msgs/Point32[] push_direction_convex~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'cornerFind-response)))
  "Returns full string definition for message of type 'cornerFind-response"
  (cl:format cl:nil "geometry_msgs/Point[] corner~%geometry_msgs/Point32[] push_direction~%geometry_msgs/Point[] corner_convex~%geometry_msgs/Point32[] push_direction_convex~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <cornerFind-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'corner) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'push_direction) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'corner_convex) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'push_direction_convex) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <cornerFind-response>))
  "Converts a ROS message object to a list"
  (cl:list 'cornerFind-response
    (cl:cons ':corner (corner msg))
    (cl:cons ':push_direction (push_direction msg))
    (cl:cons ':corner_convex (corner_convex msg))
    (cl:cons ':push_direction_convex (push_direction_convex msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'cornerFind)))
  'cornerFind-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'cornerFind)))
  'cornerFind-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cornerFind)))
  "Returns string type for a service object of type '<cornerFind>"
  "interactive_segmentation_textured/cornerFind")