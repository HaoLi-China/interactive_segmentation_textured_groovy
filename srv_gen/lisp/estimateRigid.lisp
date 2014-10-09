; Auto-generated. Do not edit!


(cl:in-package interactive_segmentation_textured-srv)


;//! \htmlinclude estimateRigid-request.msg.html

(cl:defclass <estimateRigid-request> (roslisp-msg-protocol:ros-message)
  ((features_old
    :reader features_old
    :initarg :features_old
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (features_new
    :reader features_new
    :initarg :features_new
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point))))
)

(cl:defclass estimateRigid-request (<estimateRigid-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <estimateRigid-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'estimateRigid-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name interactive_segmentation_textured-srv:<estimateRigid-request> is deprecated: use interactive_segmentation_textured-srv:estimateRigid-request instead.")))

(cl:ensure-generic-function 'features_old-val :lambda-list '(m))
(cl:defmethod features_old-val ((m <estimateRigid-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interactive_segmentation_textured-srv:features_old-val is deprecated.  Use interactive_segmentation_textured-srv:features_old instead.")
  (features_old m))

(cl:ensure-generic-function 'features_new-val :lambda-list '(m))
(cl:defmethod features_new-val ((m <estimateRigid-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interactive_segmentation_textured-srv:features_new-val is deprecated.  Use interactive_segmentation_textured-srv:features_new instead.")
  (features_new m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <estimateRigid-request>) ostream)
  "Serializes a message object of type '<estimateRigid-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'features_old))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'features_old))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'features_new))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'features_new))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <estimateRigid-request>) istream)
  "Deserializes a message object of type '<estimateRigid-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'features_old) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'features_old)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'features_new) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'features_new)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<estimateRigid-request>)))
  "Returns string type for a service object of type '<estimateRigid-request>"
  "interactive_segmentation_textured/estimateRigidRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'estimateRigid-request)))
  "Returns string type for a service object of type 'estimateRigid-request"
  "interactive_segmentation_textured/estimateRigidRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<estimateRigid-request>)))
  "Returns md5sum for a message object of type '<estimateRigid-request>"
  "536808d5a729d88f5d653c8f3bc1c5db")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'estimateRigid-request)))
  "Returns md5sum for a message object of type 'estimateRigid-request"
  "536808d5a729d88f5d653c8f3bc1c5db")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<estimateRigid-request>)))
  "Returns full string definition for message of type '<estimateRigid-request>"
  (cl:format cl:nil "geometry_msgs/Point[] features_old~%geometry_msgs/Point[] features_new~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'estimateRigid-request)))
  "Returns full string definition for message of type 'estimateRigid-request"
  (cl:format cl:nil "geometry_msgs/Point[] features_old~%geometry_msgs/Point[] features_new~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <estimateRigid-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'features_old) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'features_new) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <estimateRigid-request>))
  "Converts a ROS message object to a list"
  (cl:list 'estimateRigid-request
    (cl:cons ':features_old (features_old msg))
    (cl:cons ':features_new (features_new msg))
))
;//! \htmlinclude estimateRigid-response.msg.html

(cl:defclass <estimateRigid-response> (roslisp-msg-protocol:ros-message)
  ((inliers
    :reader inliers
    :initarg :inliers
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (success
    :reader success
    :initarg :success
    :type cl:integer
    :initform 0))
)

(cl:defclass estimateRigid-response (<estimateRigid-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <estimateRigid-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'estimateRigid-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name interactive_segmentation_textured-srv:<estimateRigid-response> is deprecated: use interactive_segmentation_textured-srv:estimateRigid-response instead.")))

(cl:ensure-generic-function 'inliers-val :lambda-list '(m))
(cl:defmethod inliers-val ((m <estimateRigid-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interactive_segmentation_textured-srv:inliers-val is deprecated.  Use interactive_segmentation_textured-srv:inliers instead.")
  (inliers m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <estimateRigid-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader interactive_segmentation_textured-srv:success-val is deprecated.  Use interactive_segmentation_textured-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <estimateRigid-response>) ostream)
  "Serializes a message object of type '<estimateRigid-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'inliers))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'inliers))
  (cl:let* ((signed (cl:slot-value msg 'success)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <estimateRigid-response>) istream)
  "Deserializes a message object of type '<estimateRigid-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'inliers) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'inliers)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'success) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<estimateRigid-response>)))
  "Returns string type for a service object of type '<estimateRigid-response>"
  "interactive_segmentation_textured/estimateRigidResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'estimateRigid-response)))
  "Returns string type for a service object of type 'estimateRigid-response"
  "interactive_segmentation_textured/estimateRigidResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<estimateRigid-response>)))
  "Returns md5sum for a message object of type '<estimateRigid-response>"
  "536808d5a729d88f5d653c8f3bc1c5db")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'estimateRigid-response)))
  "Returns md5sum for a message object of type 'estimateRigid-response"
  "536808d5a729d88f5d653c8f3bc1c5db")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<estimateRigid-response>)))
  "Returns full string definition for message of type '<estimateRigid-response>"
  (cl:format cl:nil "int32[] inliers~%int32 success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'estimateRigid-response)))
  "Returns full string definition for message of type 'estimateRigid-response"
  (cl:format cl:nil "int32[] inliers~%int32 success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <estimateRigid-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'inliers) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <estimateRigid-response>))
  "Converts a ROS message object to a list"
  (cl:list 'estimateRigid-response
    (cl:cons ':inliers (inliers msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'estimateRigid)))
  'estimateRigid-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'estimateRigid)))
  'estimateRigid-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'estimateRigid)))
  "Returns string type for a service object of type '<estimateRigid>"
  "interactive_segmentation_textured/estimateRigid")