;; Auto-generated. Do not edit!


(when (boundp 'ros_x_habitat::GetAgentPose)
  (if (not (find-package "ROS_X_HABITAT"))
    (make-package "ROS_X_HABITAT"))
  (shadow 'GetAgentPose (find-package "ROS_X_HABITAT")))
(unless (find-package "ROS_X_HABITAT::GETAGENTPOSE")
  (make-package "ROS_X_HABITAT::GETAGENTPOSE"))
(unless (find-package "ROS_X_HABITAT::GETAGENTPOSEREQUEST")
  (make-package "ROS_X_HABITAT::GETAGENTPOSEREQUEST"))
(unless (find-package "ROS_X_HABITAT::GETAGENTPOSERESPONSE")
  (make-package "ROS_X_HABITAT::GETAGENTPOSERESPONSE"))

(in-package "ROS")



(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass ros_x_habitat::GetAgentPoseRequest
  :super ros::object
  :slots ())

(defmethod ros_x_habitat::GetAgentPoseRequest
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

(defclass ros_x_habitat::GetAgentPoseResponse
  :super ros::object
  :slots (_pose ))

(defmethod ros_x_habitat::GetAgentPoseResponse
  (:init
   (&key
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _pose __pose)
   self)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;;
   self)
  )

(defclass ros_x_habitat::GetAgentPose
  :super ros::object
  :slots ())

(setf (get ros_x_habitat::GetAgentPose :md5sum-) "f192399f711a48924df9a394d37edd67")
(setf (get ros_x_habitat::GetAgentPose :datatype-) "ros_x_habitat/GetAgentPose")
(setf (get ros_x_habitat::GetAgentPose :request) ros_x_habitat::GetAgentPoseRequest)
(setf (get ros_x_habitat::GetAgentPose :response) ros_x_habitat::GetAgentPoseResponse)

(defmethod ros_x_habitat::GetAgentPoseRequest
  (:response () (instance ros_x_habitat::GetAgentPoseResponse :init)))

(setf (get ros_x_habitat::GetAgentPoseRequest :md5sum-) "f192399f711a48924df9a394d37edd67")
(setf (get ros_x_habitat::GetAgentPoseRequest :datatype-) "ros_x_habitat/GetAgentPoseRequest")
(setf (get ros_x_habitat::GetAgentPoseRequest :definition-)
      "---
geometry_msgs/Pose pose

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

(setf (get ros_x_habitat::GetAgentPoseResponse :md5sum-) "f192399f711a48924df9a394d37edd67")
(setf (get ros_x_habitat::GetAgentPoseResponse :datatype-) "ros_x_habitat/GetAgentPoseResponse")
(setf (get ros_x_habitat::GetAgentPoseResponse :definition-)
      "---
geometry_msgs/Pose pose

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



(provide :ros_x_habitat/GetAgentPose "f192399f711a48924df9a394d37edd67")


