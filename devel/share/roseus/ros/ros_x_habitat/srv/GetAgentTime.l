;; Auto-generated. Do not edit!


(when (boundp 'ros_x_habitat::GetAgentTime)
  (if (not (find-package "ROS_X_HABITAT"))
    (make-package "ROS_X_HABITAT"))
  (shadow 'GetAgentTime (find-package "ROS_X_HABITAT")))
(unless (find-package "ROS_X_HABITAT::GETAGENTTIME")
  (make-package "ROS_X_HABITAT::GETAGENTTIME"))
(unless (find-package "ROS_X_HABITAT::GETAGENTTIMEREQUEST")
  (make-package "ROS_X_HABITAT::GETAGENTTIMEREQUEST"))
(unless (find-package "ROS_X_HABITAT::GETAGENTTIMERESPONSE")
  (make-package "ROS_X_HABITAT::GETAGENTTIMERESPONSE"))

(in-package "ROS")





(defclass ros_x_habitat::GetAgentTimeRequest
  :super ros::object
  :slots ())

(defmethod ros_x_habitat::GetAgentTimeRequest
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

(defclass ros_x_habitat::GetAgentTimeResponse
  :super ros::object
  :slots (_agent_time ))

(defmethod ros_x_habitat::GetAgentTimeResponse
  (:init
   (&key
    ((:agent_time __agent_time) 0.0)
    )
   (send-super :init)
   (setq _agent_time (float __agent_time))
   self)
  (:agent_time
   (&optional __agent_time)
   (if __agent_time (setq _agent_time __agent_time)) _agent_time)
  (:serialization-length
   ()
   (+
    ;; float32 _agent_time
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _agent_time
       (sys::poke _agent_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _agent_time
     (setq _agent_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass ros_x_habitat::GetAgentTime
  :super ros::object
  :slots ())

(setf (get ros_x_habitat::GetAgentTime :md5sum-) "823e9883d7a51ebd6a56062eea85e4b1")
(setf (get ros_x_habitat::GetAgentTime :datatype-) "ros_x_habitat/GetAgentTime")
(setf (get ros_x_habitat::GetAgentTime :request) ros_x_habitat::GetAgentTimeRequest)
(setf (get ros_x_habitat::GetAgentTime :response) ros_x_habitat::GetAgentTimeResponse)

(defmethod ros_x_habitat::GetAgentTimeRequest
  (:response () (instance ros_x_habitat::GetAgentTimeResponse :init)))

(setf (get ros_x_habitat::GetAgentTimeRequest :md5sum-) "823e9883d7a51ebd6a56062eea85e4b1")
(setf (get ros_x_habitat::GetAgentTimeRequest :datatype-) "ros_x_habitat/GetAgentTimeRequest")
(setf (get ros_x_habitat::GetAgentTimeRequest :definition-)
      "---
float32 agent_time
")

(setf (get ros_x_habitat::GetAgentTimeResponse :md5sum-) "823e9883d7a51ebd6a56062eea85e4b1")
(setf (get ros_x_habitat::GetAgentTimeResponse :datatype-) "ros_x_habitat/GetAgentTimeResponse")
(setf (get ros_x_habitat::GetAgentTimeResponse :definition-)
      "---
float32 agent_time
")



(provide :ros_x_habitat/GetAgentTime "823e9883d7a51ebd6a56062eea85e4b1")


