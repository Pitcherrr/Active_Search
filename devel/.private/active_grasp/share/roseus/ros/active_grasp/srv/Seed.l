;; Auto-generated. Do not edit!


(when (boundp 'active_grasp::Seed)
  (if (not (find-package "ACTIVE_GRASP"))
    (make-package "ACTIVE_GRASP"))
  (shadow 'Seed (find-package "ACTIVE_GRASP")))
(unless (find-package "ACTIVE_GRASP::SEED")
  (make-package "ACTIVE_GRASP::SEED"))
(unless (find-package "ACTIVE_GRASP::SEEDREQUEST")
  (make-package "ACTIVE_GRASP::SEEDREQUEST"))
(unless (find-package "ACTIVE_GRASP::SEEDRESPONSE")
  (make-package "ACTIVE_GRASP::SEEDRESPONSE"))

(in-package "ROS")





(defclass active_grasp::SeedRequest
  :super ros::object
  :slots (_seed ))

(defmethod active_grasp::SeedRequest
  (:init
   (&key
    ((:seed __seed) 0)
    )
   (send-super :init)
   (setq _seed (round __seed))
   self)
  (:seed
   (&optional __seed)
   (if __seed (setq _seed __seed)) _seed)
  (:serialization-length
   ()
   (+
    ;; uint32 _seed
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _seed
       (write-long _seed s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _seed
     (setq _seed (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass active_grasp::SeedResponse
  :super ros::object
  :slots ())

(defmethod active_grasp::SeedResponse
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

(defclass active_grasp::Seed
  :super ros::object
  :slots ())

(setf (get active_grasp::Seed :md5sum-) "56fd131f75eb969fd4909892624a2fdc")
(setf (get active_grasp::Seed :datatype-) "active_grasp/Seed")
(setf (get active_grasp::Seed :request) active_grasp::SeedRequest)
(setf (get active_grasp::Seed :response) active_grasp::SeedResponse)

(defmethod active_grasp::SeedRequest
  (:response () (instance active_grasp::SeedResponse :init)))

(setf (get active_grasp::SeedRequest :md5sum-) "56fd131f75eb969fd4909892624a2fdc")
(setf (get active_grasp::SeedRequest :datatype-) "active_grasp/SeedRequest")
(setf (get active_grasp::SeedRequest :definition-)
      "uint32 seed
---

")

(setf (get active_grasp::SeedResponse :md5sum-) "56fd131f75eb969fd4909892624a2fdc")
(setf (get active_grasp::SeedResponse :datatype-) "active_grasp/SeedResponse")
(setf (get active_grasp::SeedResponse :definition-)
      "uint32 seed
---

")



(provide :active_grasp/Seed "56fd131f75eb969fd4909892624a2fdc")


