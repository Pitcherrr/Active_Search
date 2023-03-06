
(cl:in-package :asdf)

(defsystem "active_grasp-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "AABBox" :depends-on ("_package_AABBox"))
    (:file "_package_AABBox" :depends-on ("_package"))
  ))