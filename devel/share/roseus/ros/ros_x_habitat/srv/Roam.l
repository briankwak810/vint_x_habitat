;; Auto-generated. Do not edit!


(when (boundp 'ros_x_habitat::Roam)
  (if (not (find-package "ROS_X_HABITAT"))
    (make-package "ROS_X_HABITAT"))
  (shadow 'Roam (find-package "ROS_X_HABITAT")))
(unless (find-package "ROS_X_HABITAT::ROAM")
  (make-package "ROS_X_HABITAT::ROAM"))
(unless (find-package "ROS_X_HABITAT::ROAMREQUEST")
  (make-package "ROS_X_HABITAT::ROAMREQUEST"))
(unless (find-package "ROS_X_HABITAT::ROAMRESPONSE")
  (make-package "ROS_X_HABITAT::ROAMRESPONSE"))

(in-package "ROS")





(defclass ros_x_habitat::RoamRequest
  :super ros::object
  :slots (_episode_id_last _scene_id_last _make_video _video_frame_period ))

(defmethod ros_x_habitat::RoamRequest
  (:init
   (&key
    ((:episode_id_last __episode_id_last) "")
    ((:scene_id_last __scene_id_last) "")
    ((:make_video __make_video) nil)
    ((:video_frame_period __video_frame_period) 0)
    )
   (send-super :init)
   (setq _episode_id_last (string __episode_id_last))
   (setq _scene_id_last (string __scene_id_last))
   (setq _make_video __make_video)
   (setq _video_frame_period (round __video_frame_period))
   self)
  (:episode_id_last
   (&optional __episode_id_last)
   (if __episode_id_last (setq _episode_id_last __episode_id_last)) _episode_id_last)
  (:scene_id_last
   (&optional __scene_id_last)
   (if __scene_id_last (setq _scene_id_last __scene_id_last)) _scene_id_last)
  (:make_video
   (&optional (__make_video :null))
   (if (not (eq __make_video :null)) (setq _make_video __make_video)) _make_video)
  (:video_frame_period
   (&optional __video_frame_period)
   (if __video_frame_period (setq _video_frame_period __video_frame_period)) _video_frame_period)
  (:serialization-length
   ()
   (+
    ;; string _episode_id_last
    4 (length _episode_id_last)
    ;; string _scene_id_last
    4 (length _scene_id_last)
    ;; bool _make_video
    1
    ;; int32 _video_frame_period
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _episode_id_last
       (write-long (length _episode_id_last) s) (princ _episode_id_last s)
     ;; string _scene_id_last
       (write-long (length _scene_id_last) s) (princ _scene_id_last s)
     ;; bool _make_video
       (if _make_video (write-byte -1 s) (write-byte 0 s))
     ;; int32 _video_frame_period
       (write-long _video_frame_period s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _episode_id_last
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _episode_id_last (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _scene_id_last
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _scene_id_last (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _make_video
     (setq _make_video (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _video_frame_period
     (setq _video_frame_period (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass ros_x_habitat::RoamResponse
  :super ros::object
  :slots (_ack ))

(defmethod ros_x_habitat::RoamResponse
  (:init
   (&key
    ((:ack __ack) nil)
    )
   (send-super :init)
   (setq _ack __ack)
   self)
  (:ack
   (&optional (__ack :null))
   (if (not (eq __ack :null)) (setq _ack __ack)) _ack)
  (:serialization-length
   ()
   (+
    ;; bool _ack
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _ack
       (if _ack (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _ack
     (setq _ack (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass ros_x_habitat::Roam
  :super ros::object
  :slots ())

(setf (get ros_x_habitat::Roam :md5sum-) "1c6692b437c77a3364ed4415992a3409")
(setf (get ros_x_habitat::Roam :datatype-) "ros_x_habitat/Roam")
(setf (get ros_x_habitat::Roam :request) ros_x_habitat::RoamRequest)
(setf (get ros_x_habitat::Roam :response) ros_x_habitat::RoamResponse)

(defmethod ros_x_habitat::RoamRequest
  (:response () (instance ros_x_habitat::RoamResponse :init)))

(setf (get ros_x_habitat::RoamRequest :md5sum-) "1c6692b437c77a3364ed4415992a3409")
(setf (get ros_x_habitat::RoamRequest :datatype-) "ros_x_habitat/RoamRequest")
(setf (get ros_x_habitat::RoamRequest :definition-)
      "string episode_id_last
string scene_id_last
bool make_video
int32 video_frame_period
---
bool ack
")

(setf (get ros_x_habitat::RoamResponse :md5sum-) "1c6692b437c77a3364ed4415992a3409")
(setf (get ros_x_habitat::RoamResponse :datatype-) "ros_x_habitat/RoamResponse")
(setf (get ros_x_habitat::RoamResponse :definition-)
      "string episode_id_last
string scene_id_last
bool make_video
int32 video_frame_period
---
bool ack
")



(provide :ros_x_habitat/Roam "1c6692b437c77a3364ed4415992a3409")


