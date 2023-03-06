; Auto-generated. Do not edit!


(cl:in-package vgn-srv)


;//! \htmlinclude PredictGrasps-request.msg.html

(cl:defclass <PredictGrasps-request> (roslisp-msg-protocol:ros-message)
  ((voxel_size
    :reader voxel_size
    :initarg :voxel_size
    :type cl:float
    :initform 0.0)
   (map_cloud
    :reader map_cloud
    :initarg :map_cloud
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2)))
)

(cl:defclass PredictGrasps-request (<PredictGrasps-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PredictGrasps-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PredictGrasps-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vgn-srv:<PredictGrasps-request> is deprecated: use vgn-srv:PredictGrasps-request instead.")))

(cl:ensure-generic-function 'voxel_size-val :lambda-list '(m))
(cl:defmethod voxel_size-val ((m <PredictGrasps-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vgn-srv:voxel_size-val is deprecated.  Use vgn-srv:voxel_size instead.")
  (voxel_size m))

(cl:ensure-generic-function 'map_cloud-val :lambda-list '(m))
(cl:defmethod map_cloud-val ((m <PredictGrasps-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vgn-srv:map_cloud-val is deprecated.  Use vgn-srv:map_cloud instead.")
  (map_cloud m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PredictGrasps-request>) ostream)
  "Serializes a message object of type '<PredictGrasps-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'voxel_size))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'map_cloud) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PredictGrasps-request>) istream)
  "Deserializes a message object of type '<PredictGrasps-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'voxel_size) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'map_cloud) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PredictGrasps-request>)))
  "Returns string type for a service object of type '<PredictGrasps-request>"
  "vgn/PredictGraspsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PredictGrasps-request)))
  "Returns string type for a service object of type 'PredictGrasps-request"
  "vgn/PredictGraspsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PredictGrasps-request>)))
  "Returns md5sum for a message object of type '<PredictGrasps-request>"
  "c81c71905509ce157d8c512e48cd31ad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PredictGrasps-request)))
  "Returns md5sum for a message object of type 'PredictGrasps-request"
  "c81c71905509ce157d8c512e48cd31ad")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PredictGrasps-request>)))
  "Returns full string definition for message of type '<PredictGrasps-request>"
  (cl:format cl:nil "float64 voxel_size~%sensor_msgs/PointCloud2 map_cloud~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PredictGrasps-request)))
  "Returns full string definition for message of type 'PredictGrasps-request"
  (cl:format cl:nil "float64 voxel_size~%sensor_msgs/PointCloud2 map_cloud~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PredictGrasps-request>))
  (cl:+ 0
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'map_cloud))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PredictGrasps-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PredictGrasps-request
    (cl:cons ':voxel_size (voxel_size msg))
    (cl:cons ':map_cloud (map_cloud msg))
))
;//! \htmlinclude PredictGrasps-response.msg.html

(cl:defclass <PredictGrasps-response> (roslisp-msg-protocol:ros-message)
  ((grasps
    :reader grasps
    :initarg :grasps
    :type (cl:vector vgn-msg:GraspConfig)
   :initform (cl:make-array 0 :element-type 'vgn-msg:GraspConfig :initial-element (cl:make-instance 'vgn-msg:GraspConfig))))
)

(cl:defclass PredictGrasps-response (<PredictGrasps-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PredictGrasps-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PredictGrasps-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vgn-srv:<PredictGrasps-response> is deprecated: use vgn-srv:PredictGrasps-response instead.")))

(cl:ensure-generic-function 'grasps-val :lambda-list '(m))
(cl:defmethod grasps-val ((m <PredictGrasps-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vgn-srv:grasps-val is deprecated.  Use vgn-srv:grasps instead.")
  (grasps m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PredictGrasps-response>) ostream)
  "Serializes a message object of type '<PredictGrasps-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'grasps))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'grasps))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PredictGrasps-response>) istream)
  "Deserializes a message object of type '<PredictGrasps-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'grasps) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'grasps)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'vgn-msg:GraspConfig))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PredictGrasps-response>)))
  "Returns string type for a service object of type '<PredictGrasps-response>"
  "vgn/PredictGraspsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PredictGrasps-response)))
  "Returns string type for a service object of type 'PredictGrasps-response"
  "vgn/PredictGraspsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PredictGrasps-response>)))
  "Returns md5sum for a message object of type '<PredictGrasps-response>"
  "c81c71905509ce157d8c512e48cd31ad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PredictGrasps-response)))
  "Returns md5sum for a message object of type 'PredictGrasps-response"
  "c81c71905509ce157d8c512e48cd31ad")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PredictGrasps-response>)))
  "Returns full string definition for message of type '<PredictGrasps-response>"
  (cl:format cl:nil "vgn/GraspConfig[] grasps~%~%~%================================================================================~%MSG: vgn/GraspConfig~%geometry_msgs/Pose pose~%float32 width~%float32 quality~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PredictGrasps-response)))
  "Returns full string definition for message of type 'PredictGrasps-response"
  (cl:format cl:nil "vgn/GraspConfig[] grasps~%~%~%================================================================================~%MSG: vgn/GraspConfig~%geometry_msgs/Pose pose~%float32 width~%float32 quality~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PredictGrasps-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'grasps) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PredictGrasps-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PredictGrasps-response
    (cl:cons ':grasps (grasps msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PredictGrasps)))
  'PredictGrasps-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PredictGrasps)))
  'PredictGrasps-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PredictGrasps)))
  "Returns string type for a service object of type '<PredictGrasps>"
  "vgn/PredictGrasps")