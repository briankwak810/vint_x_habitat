;; Auto-generated. Do not edit!


(when (boundp 'ros_x_habitat::EvalEpisode)
  (if (not (find-package "ROS_X_HABITAT"))
    (make-package "ROS_X_HABITAT"))
  (shadow 'EvalEpisode (find-package "ROS_X_HABITAT")))
(unless (find-package "ROS_X_HABITAT::EVALEPISODE")
  (make-package "ROS_X_HABITAT::EVALEPISODE"))
(unless (find-package "ROS_X_HABITAT::EVALEPISODEREQUEST")
  (make-package "ROS_X_HABITAT::EVALEPISODEREQUEST"))
(unless (find-package "ROS_X_HABITAT::EVALEPISODERESPONSE")
  (make-package "ROS_X_HABITAT::EVALEPISODERESPONSE"))

(in-package "ROS")





(defclass ros_x_habitat::EvalEpisodeRequest
  :super ros::object
  :slots (_episode_id_last _scene_id_last ))

(defmethod ros_x_habitat::EvalEpisodeRequest
  (:init
   (&key
    ((:episode_id_last __episode_id_last) "")
    ((:scene_id_last __scene_id_last) "")
    )
   (send-super :init)
   (setq _episode_id_last (string __episode_id_last))
   (setq _scene_id_last (string __scene_id_last))
   self)
  (:episode_id_last
   (&optional __episode_id_last)
   (if __episode_id_last (setq _episode_id_last __episode_id_last)) _episode_id_last)
  (:scene_id_last
   (&optional __scene_id_last)
   (if __scene_id_last (setq _scene_id_last __scene_id_last)) _scene_id_last)
  (:serialization-length
   ()
   (+
    ;; string _episode_id_last
    4 (length _episode_id_last)
    ;; string _scene_id_last
    4 (length _scene_id_last)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _episode_id_last
       (write-long (length _episode_id_last) s) (princ _episode_id_last s)
     ;; string _scene_id_last
       (write-long (length _scene_id_last) s) (princ _scene_id_last s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _episode_id_last
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _episode_id_last (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _scene_id_last
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _scene_id_last (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass ros_x_habitat::EvalEpisodeResponse
  :super ros::object
  :slots (_episode_id _scene_id _distance_to_goal _success _spl _num_steps _agent_time _sim_time _reset_time ))

(defmethod ros_x_habitat::EvalEpisodeResponse
  (:init
   (&key
    ((:episode_id __episode_id) "")
    ((:scene_id __scene_id) "")
    ((:distance_to_goal __distance_to_goal) 0.0)
    ((:success __success) 0.0)
    ((:spl __spl) 0.0)
    ((:num_steps __num_steps) 0)
    ((:agent_time __agent_time) 0.0)
    ((:sim_time __sim_time) 0.0)
    ((:reset_time __reset_time) 0.0)
    )
   (send-super :init)
   (setq _episode_id (string __episode_id))
   (setq _scene_id (string __scene_id))
   (setq _distance_to_goal (float __distance_to_goal))
   (setq _success (float __success))
   (setq _spl (float __spl))
   (setq _num_steps (round __num_steps))
   (setq _agent_time (float __agent_time))
   (setq _sim_time (float __sim_time))
   (setq _reset_time (float __reset_time))
   self)
  (:episode_id
   (&optional __episode_id)
   (if __episode_id (setq _episode_id __episode_id)) _episode_id)
  (:scene_id
   (&optional __scene_id)
   (if __scene_id (setq _scene_id __scene_id)) _scene_id)
  (:distance_to_goal
   (&optional __distance_to_goal)
   (if __distance_to_goal (setq _distance_to_goal __distance_to_goal)) _distance_to_goal)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:spl
   (&optional __spl)
   (if __spl (setq _spl __spl)) _spl)
  (:num_steps
   (&optional __num_steps)
   (if __num_steps (setq _num_steps __num_steps)) _num_steps)
  (:agent_time
   (&optional __agent_time)
   (if __agent_time (setq _agent_time __agent_time)) _agent_time)
  (:sim_time
   (&optional __sim_time)
   (if __sim_time (setq _sim_time __sim_time)) _sim_time)
  (:reset_time
   (&optional __reset_time)
   (if __reset_time (setq _reset_time __reset_time)) _reset_time)
  (:serialization-length
   ()
   (+
    ;; string _episode_id
    4 (length _episode_id)
    ;; string _scene_id
    4 (length _scene_id)
    ;; float32 _distance_to_goal
    4
    ;; float32 _success
    4
    ;; float32 _spl
    4
    ;; int32 _num_steps
    4
    ;; float32 _agent_time
    4
    ;; float32 _sim_time
    4
    ;; float32 _reset_time
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _episode_id
       (write-long (length _episode_id) s) (princ _episode_id s)
     ;; string _scene_id
       (write-long (length _scene_id) s) (princ _scene_id s)
     ;; float32 _distance_to_goal
       (sys::poke _distance_to_goal (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _success
       (sys::poke _success (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _spl
       (sys::poke _spl (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _num_steps
       (write-long _num_steps s)
     ;; float32 _agent_time
       (sys::poke _agent_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _sim_time
       (sys::poke _sim_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _reset_time
       (sys::poke _reset_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _episode_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _episode_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _scene_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _scene_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _distance_to_goal
     (setq _distance_to_goal (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _success
     (setq _success (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _spl
     (setq _spl (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _num_steps
     (setq _num_steps (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _agent_time
     (setq _agent_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _sim_time
     (setq _sim_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _reset_time
     (setq _reset_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass ros_x_habitat::EvalEpisode
  :super ros::object
  :slots ())

(setf (get ros_x_habitat::EvalEpisode :md5sum-) "4cf4a4cf4a1d60db72496f8e3635a2d3")
(setf (get ros_x_habitat::EvalEpisode :datatype-) "ros_x_habitat/EvalEpisode")
(setf (get ros_x_habitat::EvalEpisode :request) ros_x_habitat::EvalEpisodeRequest)
(setf (get ros_x_habitat::EvalEpisode :response) ros_x_habitat::EvalEpisodeResponse)

(defmethod ros_x_habitat::EvalEpisodeRequest
  (:response () (instance ros_x_habitat::EvalEpisodeResponse :init)))

(setf (get ros_x_habitat::EvalEpisodeRequest :md5sum-) "4cf4a4cf4a1d60db72496f8e3635a2d3")
(setf (get ros_x_habitat::EvalEpisodeRequest :datatype-) "ros_x_habitat/EvalEpisodeRequest")
(setf (get ros_x_habitat::EvalEpisodeRequest :definition-)
      "string episode_id_last
string scene_id_last
---
string episode_id
string scene_id
float32 distance_to_goal
float32 success
float32 spl
int32 num_steps
float32 agent_time
float32 sim_time
float32 reset_time

")

(setf (get ros_x_habitat::EvalEpisodeResponse :md5sum-) "4cf4a4cf4a1d60db72496f8e3635a2d3")
(setf (get ros_x_habitat::EvalEpisodeResponse :datatype-) "ros_x_habitat/EvalEpisodeResponse")
(setf (get ros_x_habitat::EvalEpisodeResponse :definition-)
      "string episode_id_last
string scene_id_last
---
string episode_id
string scene_id
float32 distance_to_goal
float32 success
float32 spl
int32 num_steps
float32 agent_time
float32 sim_time
float32 reset_time

")



(provide :ros_x_habitat/EvalEpisode "4cf4a4cf4a1d60db72496f8e3635a2d3")


