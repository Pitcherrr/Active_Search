;; Auto-generated. Do not edit!


(when (boundp 'active_grasp::Reset)
  (if (not (find-package "ACTIVE_GRASP"))
    (make-package "ACTIVE_GRASP"))
  (shadow 'Reset (find-package "ACTIVE_GRASP")))
(unless (find-package "ACTIVE_GRASP::RESET")
  (make-package "ACTIVE_GRASP::RESET"))
(unless (find-package "ACTIVE_GRASP::RESETREQUEST")
  (make-package "ACTIVE_GRASP::RESETREQUEST"))
(unless (find-package "ACTIVE_GRASP::RESETRESPONSE")
  (make-package "ACTIVE_GRASP::RESETRESPONSE"))

(in-package "ROS")





(defclass active_grasp::ResetRequest
  :super ros::object
  :slots ())

(defmethod active_grasp::ResetRequest
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

(defclass active_grasp::ResetResponse
  :super ros::object
  :slots (_bbox ))

(defmethod active_grasp::ResetResponse
  (:init
   (&key
    ((:bbox __bbox) (instance active_grasp::AABBox :init))
    )
   (send-super :init)
   (setq _bbox __bbox)
   self)
  (:bbox
   (&rest __bbox)
   (if (keywordp (car __bbox))
       (send* _bbox __bbox)
     (progn
       (if __bbox (setq _bbox (car __bbox)))
       _bbox)))
  (:serialization-length
   ()
   (+
    ;; active_grasp/AABBox _bbox
    (send _bbox :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; active_grasp/AABBox _bbox
       (send _bbox :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; active_grasp/AABBox _bbox
     (send _bbox :deserialize buf ptr-) (incf ptr- (send _bbox :serialization-length))
   ;;
   self)
  )

(defclass active_grasp::Reset
  :super ros::object
  :slots ())

(setf (get active_grasp::Reset :md5sum-) "4dd9de15958dc8059c48f7ba8054e4b8")
(setf (get active_grasp::Reset :datatype-) "active_grasp/Reset")
(setf (get active_grasp::Reset :request) active_grasp::ResetRequest)
(setf (get active_grasp::Reset :response) active_grasp::ResetResponse)

(defmethod active_grasp::ResetRequest
  (:response () (instance active_grasp::ResetResponse :init)))

(setf (get active_grasp::ResetRequest :md5sum-) "4dd9de15958dc8059c48f7ba8054e4b8")
(setf (get active_grasp::ResetRequest :datatype-) "active_grasp/ResetRequest")
(setf (get active_grasp::ResetRequest :definition-)
      "---
active_grasp/AABBox bbox


================================================================================
MSG: active_grasp/AABBox
  geometry_msgs/Point min
  geometry_msgs/Point max
  
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
")

(setf (get active_grasp::ResetResponse :md5sum-) "4dd9de15958dc8059c48f7ba8054e4b8")
(setf (get active_grasp::ResetResponse :datatype-) "active_grasp/ResetResponse")
(setf (get active_grasp::ResetResponse :definition-)
      "---
active_grasp/AABBox bbox


================================================================================
MSG: active_grasp/AABBox
  geometry_msgs/Point min
  geometry_msgs/Point max
  
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
")



(provide :active_grasp/Reset "4dd9de15958dc8059c48f7ba8054e4b8")


