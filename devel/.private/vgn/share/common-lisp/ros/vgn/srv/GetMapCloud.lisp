; Auto-generated. Do not edit!


(cl:in-package vgn-srv)


;//! \htmlinclude GetMapCloud-request.msg.html

(cl:defclass <GetMapCloud-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetMapCloud-request (<GetMapCloud-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetMapCloud-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetMapCloud-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vgn-srv:<GetMapCloud-request> is deprecated: use vgn-srv:GetMapCloud-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetMapCloud-request>) ostream)
  "Serializes a message object of type '<GetMapCloud-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetMapCloud-request>) istream)
  "Deserializes a message object of type '<GetMapCloud-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetMapCloud-request>)))
  "Returns string type for a service object of type '<GetMapCloud-request>"
  "vgn/GetMapCloudRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetMapCloud-request)))
  "Returns string type for a service object of type 'GetMapCloud-request"
  "vgn/GetMapCloudRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetMapCloud-request>)))
  "Returns md5sum for a message object of type '<GetMapCloud-request>"
  "40b2d5c19b1ba0f3eae539bbfbf252ac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetMapCloud-request)))
  "Returns md5sum for a message object of type 'GetMapCloud-request"
  "40b2d5c19b1ba0f3eae539bbfbf252ac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetMapCloud-request>)))
  "Returns full string definition for message of type '<GetMapCloud-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetMapCloud-request)))
  "Returns full string definition for message of type 'GetMapCloud-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetMapCloud-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetMapCloud-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetMapCloud-request
))
;//! \htmlinclude GetMapCloud-response.msg.html

(cl:defclass <GetMapCloud-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass GetMapCloud-response (<GetMapCloud-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetMapCloud-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetMapCloud-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vgn-srv:<GetMapCloud-response> is deprecated: use vgn-srv:GetMapCloud-response instead.")))

(cl:ensure-generic-function 'voxel_size-val :lambda-list '(m))
(cl:defmethod voxel_size-val ((m <GetMapCloud-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vgn-srv:voxel_size-val is deprecated.  Use vgn-srv:voxel_size instead.")
  (voxel_size m))

(cl:ensure-generic-function 'map_cloud-val :lambda-list '(m))
(cl:defmethod map_cloud-val ((m <GetMapCloud-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vgn-srv:map_cloud-val is deprecated.  Use vgn-srv:map_cloud instead.")
  (map_cloud m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetMapCloud-response>) ostream)
  "Serializes a message object of type '<GetMapCloud-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetMapCloud-response>) istream)
  "Deserializes a message object of type '<GetMapCloud-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetMapCloud-response>)))
  "Returns string type for a service object of type '<GetMapCloud-response>"
  "vgn/GetMapCloudResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetMapCloud-response)))
  "Returns string type for a service object of type 'GetMapCloud-response"
  "vgn/GetMapCloudResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetMapCloud-response>)))
  "Returns md5sum for a message object of type '<GetMapCloud-response>"
  "40b2d5c19b1ba0f3eae539bbfbf252ac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetMapCloud-response)))
  "Returns md5sum for a message object of type 'GetMapCloud-response"
  "40b2d5c19b1ba0f3eae539bbfbf252ac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetMapCloud-response>)))
  "Returns full string definition for message of type '<GetMapCloud-response>"
  (cl:format cl:nil "float64 voxel_size~%sensor_msgs/PointCloud2 map_cloud~%~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetMapCloud-response)))
  "Returns full string definition for message of type 'GetMapCloud-response"
  (cl:format cl:nil "float64 voxel_size~%sensor_msgs/PointCloud2 map_cloud~%~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetMapCloud-response>))
  (cl:+ 0
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'map_cloud))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetMapCloud-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetMapCloud-response
    (cl:cons ':voxel_size (voxel_size msg))
    (cl:cons ':map_cloud (map_cloud msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetMapCloud)))
  'GetMapCloud-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetMapCloud)))
  'GetMapCloud-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetMapCloud)))
  "Returns string type for a service object of type '<GetMapCloud>"
  "vgn/GetMapCloud")