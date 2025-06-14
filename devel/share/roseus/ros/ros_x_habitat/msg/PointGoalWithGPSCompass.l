;; Auto-generated. Do not edit!


(when (boundp 'ros_x_habitat::PointGoalWithGPSCompass)
  (if (not (find-package "ROS_X_HABITAT"))
    (make-package "ROS_X_HABITAT"))
  (shadow 'PointGoalWithGPSCompass (find-package "ROS_X_HABITAT")))
(unless (find-package "ROS_X_HABITAT::POINTGOALWITHGPSCOMPASS")
  (make-package "ROS_X_HABITAT::POINTGOALWITHGPSCOMPASS"))

(in-package "ROS")
;;//! \htmlinclude PointGoalWithGPSCompass.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ros_x_habitat::PointGoalWithGPSCompass
  :super ros::object
  :slots (_header _distance_to_goal _angle_to_goal ))

(defmethod ros_x_habitat::PointGoalWithGPSCompass
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:distance_to_goal __distance_to_goal) 0.0)
    ((:angle_to_goal __angle_to_goal) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _distance_to_goal (float __distance_to_goal))
   (setq _angle_to_goal (float __angle_to_goal))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:distance_to_goal
   (&optional __distance_to_goal)
   (if __distance_to_goal (setq _distance_to_goal __distance_to_goal)) _distance_to_goal)
  (:angle_to_goal
   (&optional __angle_to_goal)
   (if __angle_to_goal (setq _angle_to_goal __angle_to_goal)) _angle_to_goal)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _distance_to_goal
    8
    ;; float64 _angle_to_goal
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _distance_to_goal
       (sys::poke _distance_to_goal (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _angle_to_goal
       (sys::poke _angle_to_goal (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _distance_to_goal
     (setq _distance_to_goal (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _angle_to_goal
     (setq _angle_to_goal (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get ros_x_habitat::PointGoalWithGPSCompass :md5sum-) "f1c2ae08f20a219947d5152f75072a85")
(setf (get ros_x_habitat::PointGoalWithGPSCompass :datatype-) "ros_x_habitat/PointGoalWithGPSCompass")
(setf (get ros_x_habitat::PointGoalWithGPSCompass :definition-)
      "# A representation of readings from Habitat's POINTGOAL_WITH_GPS_COMPASS sensor

Header header
float64 distance_to_goal
float64 angle_to_goal # in radians
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

")



(provide :ros_x_habitat/PointGoalWithGPSCompass "f1c2ae08f20a219947d5152f75072a85")


