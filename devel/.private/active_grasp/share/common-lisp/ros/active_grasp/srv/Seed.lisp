; Auto-generated. Do not edit!


(cl:in-package active_grasp-srv)


;//! \htmlinclude Seed-request.msg.html

(cl:defclass <Seed-request> (roslisp-msg-protocol:ros-message)
  ((seed
    :reader seed
    :initarg :seed
    :type cl:integer
    :initform 0))
)

(cl:defclass Seed-request (<Seed-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Seed-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Seed-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name active_grasp-srv:<Seed-request> is deprecated: use active_grasp-srv:Seed-request instead.")))

(cl:ensure-generic-function 'seed-val :lambda-list '(m))
(cl:defmethod seed-val ((m <Seed-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader active_grasp-srv:seed-val is deprecated.  Use active_grasp-srv:seed instead.")
  (seed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Seed-request>) ostream)
  "Serializes a message object of type '<Seed-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'seed)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'seed)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'seed)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'seed)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Seed-request>) istream)
  "Deserializes a message object of type '<Seed-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'seed)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'seed)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'seed)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'seed)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Seed-request>)))
  "Returns string type for a service object of type '<Seed-request>"
  "active_grasp/SeedRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Seed-request)))
  "Returns string type for a service object of type 'Seed-request"
  "active_grasp/SeedRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Seed-request>)))
  "Returns md5sum for a message object of type '<Seed-request>"
  "56fd131f75eb969fd4909892624a2fdc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Seed-request)))
  "Returns md5sum for a message object of type 'Seed-request"
  "56fd131f75eb969fd4909892624a2fdc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Seed-request>)))
  "Returns full string definition for message of type '<Seed-request>"
  (cl:format cl:nil "uint32 seed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Seed-request)))
  "Returns full string definition for message of type 'Seed-request"
  (cl:format cl:nil "uint32 seed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Seed-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Seed-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Seed-request
    (cl:cons ':seed (seed msg))
))
;//! \htmlinclude Seed-response.msg.html

(cl:defclass <Seed-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Seed-response (<Seed-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Seed-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Seed-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name active_grasp-srv:<Seed-response> is deprecated: use active_grasp-srv:Seed-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Seed-response>) ostream)
  "Serializes a message object of type '<Seed-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Seed-response>) istream)
  "Deserializes a message object of type '<Seed-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Seed-response>)))
  "Returns string type for a service object of type '<Seed-response>"
  "active_grasp/SeedResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Seed-response)))
  "Returns string type for a service object of type 'Seed-response"
  "active_grasp/SeedResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Seed-response>)))
  "Returns md5sum for a message object of type '<Seed-response>"
  "56fd131f75eb969fd4909892624a2fdc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Seed-response)))
  "Returns md5sum for a message object of type 'Seed-response"
  "56fd131f75eb969fd4909892624a2fdc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Seed-response>)))
  "Returns full string definition for message of type '<Seed-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Seed-response)))
  "Returns full string definition for message of type 'Seed-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Seed-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Seed-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Seed-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Seed)))
  'Seed-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Seed)))
  'Seed-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Seed)))
  "Returns string type for a service object of type '<Seed>"
  "active_grasp/Seed")