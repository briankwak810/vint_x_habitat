
(cl:in-package :asdf)

(defsystem "ros_x_habitat-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "DepthImage" :depends-on ("_package_DepthImage"))
    (:file "_package_DepthImage" :depends-on ("_package"))
    (:file "PointGoalWithGPSCompass" :depends-on ("_package_PointGoalWithGPSCompass"))
    (:file "_package_PointGoalWithGPSCompass" :depends-on ("_package"))
  ))