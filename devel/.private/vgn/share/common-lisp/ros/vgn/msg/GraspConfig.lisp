; Auto-generated. Do not edit!


(cl:in-package vgn-msg)


;//! \htmlinclude GraspConfig.msg.html

(cl:defclass <GraspConfig> (roslisp-msg-protocol:ros-message)
  ((pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (width
    :reader width
    :initarg :width
    :type cl:float
    :initform 0.0)
   (quality
    :reader quality
    :initarg :quality
    :type cl:float
    :initform 0.0))
)

(cl:defclass GraspConfig (<GraspConfig>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GraspConfig>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GraspConfig)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vgn-msg:<GraspConfig> is deprecated: use vgn-msg:GraspConfig instead.")))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <GraspConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vgn-msg:pose-val is deprecated.  Use vgn-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <GraspConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vgn-msg:width-val is deprecated.  Use vgn-msg:width instead.")
  (width m))

(cl:ensure-generic-function 'quality-val :lambda-list '(m))
(cl:defmethod quality-val ((m <GraspConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vgn-msg:quality-val is deprecated.  Use vgn-msg:quality instead.")
  (quality m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GraspConfig>) ostream)
  "Serializes a message object of type '<GraspConfig>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'width))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'quality))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GraspConfig>) istream)
  "Deserializes a message object of type '<GraspConfig>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'width) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'quality) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GraspConfig>)))
  "Returns string type for a message object of type '<GraspConfig>"
  "vgn/GraspConfig")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GraspConfig)))
  "Returns string type for a message object of type 'GraspConfig"
  "vgn/GraspConfig")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GraspConfig>)))
  "Returns md5sum for a message object of type '<GraspConfig>"
  "ffb74d75fb92764666a6fb85638db3e0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GraspConfig)))
  "Returns md5sum for a message object of type 'GraspConfig"
  "ffb74d75fb92764666a6fb85638db3e0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GraspConfig>)))
  "Returns full string definition for message of type '<GraspConfig>"
  (cl:format cl:nil "geometry_msgs/Pose pose~%float32 width~%float32 quality~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GraspConfig)))
  "Returns full string definition for message of type 'GraspConfig"
  (cl:format cl:nil "geometry_msgs/Pose pose~%float32 width~%float32 quality~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GraspConfig>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GraspConfig>))
  "Converts a ROS message object to a list"
  (cl:list 'GraspConfig
    (cl:cons ':pose (pose msg))
    (cl:cons ':width (width msg))
    (cl:cons ':quality (quality msg))
))
