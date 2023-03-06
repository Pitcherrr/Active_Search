;; Auto-generated. Do not edit!


(when (boundp 'active_grasp::AABBox)
  (if (not (find-package "ACTIVE_GRASP"))
    (make-package "ACTIVE_GRASP"))
  (shadow 'AABBox (find-package "ACTIVE_GRASP")))
(unless (find-package "ACTIVE_GRASP::AABBOX")
  (make-package "ACTIVE_GRASP::AABBOX"))

(in-package "ROS")
;;//! \htmlinclude AABBox.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass active_grasp::AABBox
  :super ros::object
  :slots (_min _max ))

(defmethod active_grasp::AABBox
  (:init
   (&key
    ((:min __min) (instance geometry_msgs::Point :init))
    ((:max __max) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _min __min)
   (setq _max __max)
   self)
  (:min
   (&rest __min)
   (if (keywordp (car __min))
       (send* _min __min)
     (progn
       (if __min (setq _min (car __min)))
       _min)))
  (:max
   (&rest __max)
   (if (keywordp (car __max))
       (send* _max __max)
     (progn
       (if __max (setq _max (car __max)))
       _max)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _min
    (send _min :serialization-length)
    ;; geometry_msgs/Point _max
    (send _max :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _min
       (send _min :serialize s)
     ;; geometry_msgs/Point _max
       (send _max :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _min
     (send _min :deserialize buf ptr-) (incf ptr- (send _min :serialization-length))
   ;; geometry_msgs/Point _max
     (send _max :deserialize buf ptr-) (incf ptr- (send _max :serialization-length))
   ;;
   self)
  )

(setf (get active_grasp::AABBox :md5sum-) "93aa3d73b866f04880927745f4aab303")
(setf (get active_grasp::AABBox :datatype-) "active_grasp/AABBox")
(setf (get active_grasp::AABBox :definition-)
      "  geometry_msgs/Point min
  geometry_msgs/Point max
  
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :active_grasp/AABBox "93aa3d73b866f04880927745f4aab303")


