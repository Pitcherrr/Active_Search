;; Auto-generated. Do not edit!


(when (boundp 'vgn::PredictGrasps)
  (if (not (find-package "VGN"))
    (make-package "VGN"))
  (shadow 'PredictGrasps (find-package "VGN")))
(unless (find-package "VGN::PREDICTGRASPS")
  (make-package "VGN::PREDICTGRASPS"))
(unless (find-package "VGN::PREDICTGRASPSREQUEST")
  (make-package "VGN::PREDICTGRASPSREQUEST"))
(unless (find-package "VGN::PREDICTGRASPSRESPONSE")
  (make-package "VGN::PREDICTGRASPSRESPONSE"))

(in-package "ROS")

(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))




(defclass vgn::PredictGraspsRequest
  :super ros::object
  :slots (_voxel_size _map_cloud ))

(defmethod vgn::PredictGraspsRequest
  (:init
   (&key
    ((:voxel_size __voxel_size) 0.0)
    ((:map_cloud __map_cloud) (instance sensor_msgs::PointCloud2 :init))
    )
   (send-super :init)
   (setq _voxel_size (float __voxel_size))
   (setq _map_cloud __map_cloud)
   self)
  (:voxel_size
   (&optional __voxel_size)
   (if __voxel_size (setq _voxel_size __voxel_size)) _voxel_size)
  (:map_cloud
   (&rest __map_cloud)
   (if (keywordp (car __map_cloud))
       (send* _map_cloud __map_cloud)
     (progn
       (if __map_cloud (setq _map_cloud (car __map_cloud)))
       _map_cloud)))
  (:serialization-length
   ()
   (+
    ;; float64 _voxel_size
    8
    ;; sensor_msgs/PointCloud2 _map_cloud
    (send _map_cloud :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _voxel_size
       (sys::poke _voxel_size (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; sensor_msgs/PointCloud2 _map_cloud
       (send _map_cloud :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _voxel_size
     (setq _voxel_size (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; sensor_msgs/PointCloud2 _map_cloud
     (send _map_cloud :deserialize buf ptr-) (incf ptr- (send _map_cloud :serialization-length))
   ;;
   self)
  )

(defclass vgn::PredictGraspsResponse
  :super ros::object
  :slots (_grasps ))

(defmethod vgn::PredictGraspsResponse
  (:init
   (&key
    ((:grasps __grasps) ())
    )
   (send-super :init)
   (setq _grasps __grasps)
   self)
  (:grasps
   (&rest __grasps)
   (if (keywordp (car __grasps))
       (send* _grasps __grasps)
     (progn
       (if __grasps (setq _grasps (car __grasps)))
       _grasps)))
  (:serialization-length
   ()
   (+
    ;; vgn/GraspConfig[] _grasps
    (apply #'+ (send-all _grasps :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; vgn/GraspConfig[] _grasps
     (write-long (length _grasps) s)
     (dolist (elem _grasps)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; vgn/GraspConfig[] _grasps
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _grasps (let (r) (dotimes (i n) (push (instance vgn::GraspConfig :init) r)) r))
     (dolist (elem- _grasps)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass vgn::PredictGrasps
  :super ros::object
  :slots ())

(setf (get vgn::PredictGrasps :md5sum-) "c81c71905509ce157d8c512e48cd31ad")
(setf (get vgn::PredictGrasps :datatype-) "vgn/PredictGrasps")
(setf (get vgn::PredictGrasps :request) vgn::PredictGraspsRequest)
(setf (get vgn::PredictGrasps :response) vgn::PredictGraspsResponse)

(defmethod vgn::PredictGraspsRequest
  (:response () (instance vgn::PredictGraspsResponse :init)))

(setf (get vgn::PredictGraspsRequest :md5sum-) "c81c71905509ce157d8c512e48cd31ad")
(setf (get vgn::PredictGraspsRequest :datatype-) "vgn/PredictGraspsRequest")
(setf (get vgn::PredictGraspsRequest :definition-)
      "float64 voxel_size
sensor_msgs/PointCloud2 map_cloud

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
---
vgn/GraspConfig[] grasps


================================================================================
MSG: vgn/GraspConfig
geometry_msgs/Pose pose
float32 width
float32 quality

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")

(setf (get vgn::PredictGraspsResponse :md5sum-) "c81c71905509ce157d8c512e48cd31ad")
(setf (get vgn::PredictGraspsResponse :datatype-) "vgn/PredictGraspsResponse")
(setf (get vgn::PredictGraspsResponse :definition-)
      "float64 voxel_size
sensor_msgs/PointCloud2 map_cloud

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
---
vgn/GraspConfig[] grasps


================================================================================
MSG: vgn/GraspConfig
geometry_msgs/Pose pose
float32 width
float32 quality

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")



(provide :vgn/PredictGrasps "c81c71905509ce157d8c512e48cd31ad")


