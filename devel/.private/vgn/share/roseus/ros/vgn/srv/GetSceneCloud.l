;; Auto-generated. Do not edit!


(when (boundp 'vgn::GetSceneCloud)
  (if (not (find-package "VGN"))
    (make-package "VGN"))
  (shadow 'GetSceneCloud (find-package "VGN")))
(unless (find-package "VGN::GETSCENECLOUD")
  (make-package "VGN::GETSCENECLOUD"))
(unless (find-package "VGN::GETSCENECLOUDREQUEST")
  (make-package "VGN::GETSCENECLOUDREQUEST"))
(unless (find-package "VGN::GETSCENECLOUDRESPONSE")
  (make-package "VGN::GETSCENECLOUDRESPONSE"))

(in-package "ROS")



(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(defclass vgn::GetSceneCloudRequest
  :super ros::object
  :slots ())

(defmethod vgn::GetSceneCloudRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass vgn::GetSceneCloudResponse
  :super ros::object
  :slots (_scene_cloud ))

(defmethod vgn::GetSceneCloudResponse
  (:init
   (&key
    ((:scene_cloud __scene_cloud) (instance sensor_msgs::PointCloud2 :init))
    )
   (send-super :init)
   (setq _scene_cloud __scene_cloud)
   self)
  (:scene_cloud
   (&rest __scene_cloud)
   (if (keywordp (car __scene_cloud))
       (send* _scene_cloud __scene_cloud)
     (progn
       (if __scene_cloud (setq _scene_cloud (car __scene_cloud)))
       _scene_cloud)))
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/PointCloud2 _scene_cloud
    (send _scene_cloud :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/PointCloud2 _scene_cloud
       (send _scene_cloud :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/PointCloud2 _scene_cloud
     (send _scene_cloud :deserialize buf ptr-) (incf ptr- (send _scene_cloud :serialization-length))
   ;;
   self)
  )

(defclass vgn::GetSceneCloud
  :super ros::object
  :slots ())

(setf (get vgn::GetSceneCloud :md5sum-) "c698231a2452033f0ed17ae0a4ebd981")
(setf (get vgn::GetSceneCloud :datatype-) "vgn/GetSceneCloud")
(setf (get vgn::GetSceneCloud :request) vgn::GetSceneCloudRequest)
(setf (get vgn::GetSceneCloud :response) vgn::GetSceneCloudResponse)

(defmethod vgn::GetSceneCloudRequest
  (:response () (instance vgn::GetSceneCloudResponse :init)))

(setf (get vgn::GetSceneCloudRequest :md5sum-) "c698231a2452033f0ed17ae0a4ebd981")
(setf (get vgn::GetSceneCloudRequest :datatype-) "vgn/GetSceneCloudRequest")
(setf (get vgn::GetSceneCloudRequest :definition-)
      "---
sensor_msgs/PointCloud2 scene_cloud


================================================================================
MSG: sensor_msgs/PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the \"fields\" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field
")

(setf (get vgn::GetSceneCloudResponse :md5sum-) "c698231a2452033f0ed17ae0a4ebd981")
(setf (get vgn::GetSceneCloudResponse :datatype-) "vgn/GetSceneCloudResponse")
(setf (get vgn::GetSceneCloudResponse :definition-)
      "---
sensor_msgs/PointCloud2 scene_cloud


================================================================================
MSG: sensor_msgs/PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the \"fields\" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field
")



(provide :vgn/GetSceneCloud "c698231a2452033f0ed17ae0a4ebd981")


