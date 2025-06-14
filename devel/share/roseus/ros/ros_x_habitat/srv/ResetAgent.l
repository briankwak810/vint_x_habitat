;; Auto-generated. Do not edit!


(when (boundp 'ros_x_habitat::ResetAgent)
  (if (not (find-package "ROS_X_HABITAT"))
    (make-package "ROS_X_HABITAT"))
  (shadow 'ResetAgent (find-package "ROS_X_HABITAT")))
(unless (find-package "ROS_X_HABITAT::RESETAGENT")
  (make-package "ROS_X_HABITAT::RESETAGENT"))
(unless (find-package "ROS_X_HABITAT::RESETAGENTREQUEST")
  (make-package "ROS_X_HABITAT::RESETAGENTREQUEST"))
(unless (find-package "ROS_X_HABITAT::RESETAGENTRESPONSE")
  (make-package "ROS_X_HABITAT::RESETAGENTRESPONSE"))

(in-package "ROS")





(defclass ros_x_habitat::ResetAgentRequest
  :super ros::object
  :slots (_reset _seed ))

(defmethod ros_x_habitat::ResetAgentRequest
  (:init
   (&key
    ((:reset __reset) 0)
    ((:seed __seed) 0)
    )
   (send-super :init)
   (setq _reset (round __reset))
   (setq _seed (round __seed))
   self)
  (:reset
   (&optional __reset)
   (if __reset (setq _reset __reset)) _reset)
  (:seed
   (&optional __seed)
   (if __seed (setq _seed __seed)) _seed)
  (:serialization-length
   ()
   (+
    ;; int16 _reset
    2
    ;; int32 _seed
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _reset
       (write-word _reset s)
     ;; int32 _seed
       (write-long _seed s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _reset
     (setq _reset (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int32 _seed
     (setq _seed (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass ros_x_habitat::ResetAgentResponse
  :super ros::object
  :slots (_done ))

(defmethod ros_x_habitat::ResetAgentResponse
  (:init
   (&key
    ((:done __done) nil)
    )
   (send-super :init)
   (setq _done __done)
   self)
  (:done
   (&optional (__done :null))
   (if (not (eq __done :null)) (setq _done __done)) _done)
  (:serialization-length
   ()
   (+
    ;; bool _done
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _done
       (if _done (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _done
     (setq _done (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass ros_x_habitat::ResetAgent
  :super ros::object
  :slots ())

(setf (get ros_x_habitat::ResetAgent :md5sum-) "ce8257351a1f8b1ef5fb3350a09efd5a")
(setf (get ros_x_habitat::ResetAgent :datatype-) "ros_x_habitat/ResetAgent")
(setf (get ros_x_habitat::ResetAgent :request) ros_x_habitat::ResetAgentRequest)
(setf (get ros_x_habitat::ResetAgent :response) ros_x_habitat::ResetAgentResponse)

(defmethod ros_x_habitat::ResetAgentRequest
  (:response () (instance ros_x_habitat::ResetAgentResponse :init)))

(setf (get ros_x_habitat::ResetAgentRequest :md5sum-) "ce8257351a1f8b1ef5fb3350a09efd5a")
(setf (get ros_x_habitat::ResetAgentRequest :datatype-) "ros_x_habitat/ResetAgentRequest")
(setf (get ros_x_habitat::ResetAgentRequest :definition-)
      "int16 reset # 0 for reset, 1 for shutdown
int32 seed # seed to instantiate agent
---
bool done
")

(setf (get ros_x_habitat::ResetAgentResponse :md5sum-) "ce8257351a1f8b1ef5fb3350a09efd5a")
(setf (get ros_x_habitat::ResetAgentResponse :datatype-) "ros_x_habitat/ResetAgentResponse")
(setf (get ros_x_habitat::ResetAgentResponse :definition-)
      "int16 reset # 0 for reset, 1 for shutdown
int32 seed # seed to instantiate agent
---
bool done
")



(provide :ros_x_habitat/ResetAgent "ce8257351a1f8b1ef5fb3350a09efd5a")


