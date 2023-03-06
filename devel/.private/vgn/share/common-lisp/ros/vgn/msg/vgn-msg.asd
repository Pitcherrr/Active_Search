
(cl:in-package :asdf)

(defsystem "vgn-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "GraspConfig" :depends-on ("_package_GraspConfig"))
    (:file "_package_GraspConfig" :depends-on ("_package"))
  ))