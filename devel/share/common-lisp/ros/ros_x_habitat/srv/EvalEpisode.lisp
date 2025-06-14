; Auto-generated. Do not edit!


(cl:in-package ros_x_habitat-srv)


;//! \htmlinclude EvalEpisode-request.msg.html

(cl:defclass <EvalEpisode-request> (roslisp-msg-protocol:ros-message)
  ((episode_id_last
    :reader episode_id_last
    :initarg :episode_id_last
    :type cl:string
    :initform "")
   (scene_id_last
    :reader scene_id_last
    :initarg :scene_id_last
    :type cl:string
    :initform ""))
)

(cl:defclass EvalEpisode-request (<EvalEpisode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EvalEpisode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EvalEpisode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_x_habitat-srv:<EvalEpisode-request> is deprecated: use ros_x_habitat-srv:EvalEpisode-request instead.")))

(cl:ensure-generic-function 'episode_id_last-val :lambda-list '(m))
(cl:defmethod episode_id_last-val ((m <EvalEpisode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_x_habitat-srv:episode_id_last-val is deprecated.  Use ros_x_habitat-srv:episode_id_last instead.")
  (episode_id_last m))

(cl:ensure-generic-function 'scene_id_last-val :lambda-list '(m))
(cl:defmethod scene_id_last-val ((m <EvalEpisode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_x_habitat-srv:scene_id_last-val is deprecated.  Use ros_x_habitat-srv:scene_id_last instead.")
  (scene_id_last m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EvalEpisode-request>) ostream)
  "Serializes a message object of type '<EvalEpisode-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'episode_id_last))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'episode_id_last))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'scene_id_last))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'scene_id_last))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EvalEpisode-request>) istream)
  "Deserializes a message object of type '<EvalEpisode-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'episode_id_last) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'episode_id_last) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'scene_id_last) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'scene_id_last) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EvalEpisode-request>)))
  "Returns string type for a service object of type '<EvalEpisode-request>"
  "ros_x_habitat/EvalEpisodeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EvalEpisode-request)))
  "Returns string type for a service object of type 'EvalEpisode-request"
  "ros_x_habitat/EvalEpisodeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EvalEpisode-request>)))
  "Returns md5sum for a message object of type '<EvalEpisode-request>"
  "4cf4a4cf4a1d60db72496f8e3635a2d3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EvalEpisode-request)))
  "Returns md5sum for a message object of type 'EvalEpisode-request"
  "4cf4a4cf4a1d60db72496f8e3635a2d3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EvalEpisode-request>)))
  "Returns full string definition for message of type '<EvalEpisode-request>"
  (cl:format cl:nil "string episode_id_last~%string scene_id_last~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EvalEpisode-request)))
  "Returns full string definition for message of type 'EvalEpisode-request"
  (cl:format cl:nil "string episode_id_last~%string scene_id_last~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EvalEpisode-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'episode_id_last))
     4 (cl:length (cl:slot-value msg 'scene_id_last))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EvalEpisode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'EvalEpisode-request
    (cl:cons ':episode_id_last (episode_id_last msg))
    (cl:cons ':scene_id_last (scene_id_last msg))
))
;//! \htmlinclude EvalEpisode-response.msg.html

(cl:defclass <EvalEpisode-response> (roslisp-msg-protocol:ros-message)
  ((episode_id
    :reader episode_id
    :initarg :episode_id
    :type cl:string
    :initform "")
   (scene_id
    :reader scene_id
    :initarg :scene_id
    :type cl:string
    :initform "")
   (distance_to_goal
    :reader distance_to_goal
    :initarg :distance_to_goal
    :type cl:float
    :initform 0.0)
   (success
    :reader success
    :initarg :success
    :type cl:float
    :initform 0.0)
   (spl
    :reader spl
    :initarg :spl
    :type cl:float
    :initform 0.0)
   (num_steps
    :reader num_steps
    :initarg :num_steps
    :type cl:integer
    :initform 0)
   (agent_time
    :reader agent_time
    :initarg :agent_time
    :type cl:float
    :initform 0.0)
   (sim_time
    :reader sim_time
    :initarg :sim_time
    :type cl:float
    :initform 0.0)
   (reset_time
    :reader reset_time
    :initarg :reset_time
    :type cl:float
    :initform 0.0))
)

(cl:defclass EvalEpisode-response (<EvalEpisode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EvalEpisode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EvalEpisode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_x_habitat-srv:<EvalEpisode-response> is deprecated: use ros_x_habitat-srv:EvalEpisode-response instead.")))

(cl:ensure-generic-function 'episode_id-val :lambda-list '(m))
(cl:defmethod episode_id-val ((m <EvalEpisode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_x_habitat-srv:episode_id-val is deprecated.  Use ros_x_habitat-srv:episode_id instead.")
  (episode_id m))

(cl:ensure-generic-function 'scene_id-val :lambda-list '(m))
(cl:defmethod scene_id-val ((m <EvalEpisode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_x_habitat-srv:scene_id-val is deprecated.  Use ros_x_habitat-srv:scene_id instead.")
  (scene_id m))

(cl:ensure-generic-function 'distance_to_goal-val :lambda-list '(m))
(cl:defmethod distance_to_goal-val ((m <EvalEpisode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_x_habitat-srv:distance_to_goal-val is deprecated.  Use ros_x_habitat-srv:distance_to_goal instead.")
  (distance_to_goal m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <EvalEpisode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_x_habitat-srv:success-val is deprecated.  Use ros_x_habitat-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'spl-val :lambda-list '(m))
(cl:defmethod spl-val ((m <EvalEpisode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_x_habitat-srv:spl-val is deprecated.  Use ros_x_habitat-srv:spl instead.")
  (spl m))

(cl:ensure-generic-function 'num_steps-val :lambda-list '(m))
(cl:defmethod num_steps-val ((m <EvalEpisode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_x_habitat-srv:num_steps-val is deprecated.  Use ros_x_habitat-srv:num_steps instead.")
  (num_steps m))

(cl:ensure-generic-function 'agent_time-val :lambda-list '(m))
(cl:defmethod agent_time-val ((m <EvalEpisode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_x_habitat-srv:agent_time-val is deprecated.  Use ros_x_habitat-srv:agent_time instead.")
  (agent_time m))

(cl:ensure-generic-function 'sim_time-val :lambda-list '(m))
(cl:defmethod sim_time-val ((m <EvalEpisode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_x_habitat-srv:sim_time-val is deprecated.  Use ros_x_habitat-srv:sim_time instead.")
  (sim_time m))

(cl:ensure-generic-function 'reset_time-val :lambda-list '(m))
(cl:defmethod reset_time-val ((m <EvalEpisode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_x_habitat-srv:reset_time-val is deprecated.  Use ros_x_habitat-srv:reset_time instead.")
  (reset_time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EvalEpisode-response>) ostream)
  "Serializes a message object of type '<EvalEpisode-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'episode_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'episode_id))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'scene_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'scene_id))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distance_to_goal))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'success))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'spl))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'num_steps)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'agent_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'sim_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'reset_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EvalEpisode-response>) istream)
  "Deserializes a message object of type '<EvalEpisode-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'episode_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'episode_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'scene_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'scene_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance_to_goal) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'success) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'spl) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_steps) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'agent_time) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'sim_time) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'reset_time) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EvalEpisode-response>)))
  "Returns string type for a service object of type '<EvalEpisode-response>"
  "ros_x_habitat/EvalEpisodeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EvalEpisode-response)))
  "Returns string type for a service object of type 'EvalEpisode-response"
  "ros_x_habitat/EvalEpisodeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EvalEpisode-response>)))
  "Returns md5sum for a message object of type '<EvalEpisode-response>"
  "4cf4a4cf4a1d60db72496f8e3635a2d3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EvalEpisode-response)))
  "Returns md5sum for a message object of type 'EvalEpisode-response"
  "4cf4a4cf4a1d60db72496f8e3635a2d3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EvalEpisode-response>)))
  "Returns full string definition for message of type '<EvalEpisode-response>"
  (cl:format cl:nil "string episode_id~%string scene_id~%float32 distance_to_goal~%float32 success~%float32 spl~%int32 num_steps~%float32 agent_time~%float32 sim_time~%float32 reset_time~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EvalEpisode-response)))
  "Returns full string definition for message of type 'EvalEpisode-response"
  (cl:format cl:nil "string episode_id~%string scene_id~%float32 distance_to_goal~%float32 success~%float32 spl~%int32 num_steps~%float32 agent_time~%float32 sim_time~%float32 reset_time~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EvalEpisode-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'episode_id))
     4 (cl:length (cl:slot-value msg 'scene_id))
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EvalEpisode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'EvalEpisode-response
    (cl:cons ':episode_id (episode_id msg))
    (cl:cons ':scene_id (scene_id msg))
    (cl:cons ':distance_to_goal (distance_to_goal msg))
    (cl:cons ':success (success msg))
    (cl:cons ':spl (spl msg))
    (cl:cons ':num_steps (num_steps msg))
    (cl:cons ':agent_time (agent_time msg))
    (cl:cons ':sim_time (sim_time msg))
    (cl:cons ':reset_time (reset_time msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'EvalEpisode)))
  'EvalEpisode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'EvalEpisode)))
  'EvalEpisode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EvalEpisode)))
  "Returns string type for a service object of type '<EvalEpisode>"
  "ros_x_habitat/EvalEpisode")