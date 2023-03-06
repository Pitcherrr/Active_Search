
(cl:in-package :asdf)

(defsystem "vgn-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
               :vgn-msg
)
  :components ((:file "_package")
    (:file "GetMapCloud" :depends-on ("_package_GetMapCloud"))
    (:file "_package_GetMapCloud" :depends-on ("_package"))
    (:file "GetSceneCloud" :depends-on ("_package_GetSceneCloud"))
    (:file "_package_GetSceneCloud" :depends-on ("_package"))
    (:file "PredictGrasps" :depends-on ("_package_PredictGrasps"))
    (:file "_package_PredictGrasps" :depends-on ("_package"))
  ))