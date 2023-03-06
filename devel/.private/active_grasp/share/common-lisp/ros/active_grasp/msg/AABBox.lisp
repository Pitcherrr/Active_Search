; Auto-generated. Do not edit!


(cl:in-package active_grasp-msg)


;//! \htmlinclude AABBox.msg.html

(cl:defclass <AABBox> (roslisp-msg-protocol:ros-message)
  ((min
    :reader min
    :initarg :min
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (max
    :reader max
    :initarg :max
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass AABBox (<AABBox>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AABBox>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AABBox)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name active_grasp-msg:<AABBox> is deprecated: use active_grasp-msg:AABBox instead.")))

(cl:ensure-generic-function 'min-val :lambda-list '(m))
(cl:defmethod min-val ((m <AABBox>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader active_grasp-msg:min-val is deprecated.  Use active_grasp-msg:min instead.")
  (min m))

(cl:ensure-generic-function 'max-val :lambda-list '(m))
(cl:defmethod max-val ((m <AABBox>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader active_grasp-msg:max-val is deprecated.  Use active_grasp-msg:max instead.")
  (max m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AABBox>) ostream)
  "Serializes a message object of type '<AABBox>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'min) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'max) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AABBox>) istream)
  "Deserializes a message object of type '<AABBox>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'min) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'max) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AABBox>)))
  "Returns string type for a message object of type '<AABBox>"
  "active_grasp/AABBox")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AABBox)))
  "Returns string type for a message object of type 'AABBox"
  "active_grasp/AABBox")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AABBox>)))
  "Returns md5sum for a message object of type '<AABBox>"
  "93aa3d73b866f04880927745f4aab303")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AABBox)))
  "Returns md5sum for a message object of type 'AABBox"
  "93aa3d73b866f04880927745f4aab303")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AABBox>)))
  "Returns full string definition for message of type '<AABBox>"
  (cl:format cl:nil "  geometry_msgs/Point min~%  geometry_msgs/Point max~%  ~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AABBox)))
  "Returns full string definition for message of type 'AABBox"
  (cl:format cl:nil "  geometry_msgs/Point min~%  geometry_msgs/Point max~%  ~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AABBox>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'min))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'max))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AABBox>))
  "Converts a ROS message object to a list"
  (cl:list 'AABBox
    (cl:cons ':min (min msg))
    (cl:cons ':max (max msg))
))
