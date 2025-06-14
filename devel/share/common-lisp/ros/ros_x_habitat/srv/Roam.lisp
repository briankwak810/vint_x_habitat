; Auto-generated. Do not edit!


(cl:in-package ros_x_habitat-srv)


;//! \htmlinclude Roam-request.msg.html

(cl:defclass <Roam-request> (roslisp-msg-protocol:ros-message)
  ((episode_id_last
    :reader episode_id_last
    :initarg :episode_id_last
    :type cl:string
    :initform "")
   (scene_id_last
    :reader scene_id_last
    :initarg :scene_id_last
    :type cl:string
    :initform "")
   (make_video
    :reader make_video
    :initarg :make_video
    :type cl:boolean
    :initform cl:nil)
   (video_frame_period
    :reader video_frame_period
    :initarg :video_frame_period
    :type cl:integer
    :initform 0))
)

(cl:defclass Roam-request (<Roam-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Roam-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Roam-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_x_habitat-srv:<Roam-request> is deprecated: use ros_x_habitat-srv:Roam-request instead.")))

(cl:ensure-generic-function 'episode_id_last-val :lambda-list '(m))
(cl:defmethod episode_id_last-val ((m <Roam-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_x_habitat-srv:episode_id_last-val is deprecated.  Use ros_x_habitat-srv:episode_id_last instead.")
  (episode_id_last m))

(cl:ensure-generic-function 'scene_id_last-val :lambda-list '(m))
(cl:defmethod scene_id_last-val ((m <Roam-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_x_habitat-srv:scene_id_last-val is deprecated.  Use ros_x_habitat-srv:scene_id_last instead.")
  (scene_id_last m))

(cl:ensure-generic-function 'make_video-val :lambda-list '(m))
(cl:defmethod make_video-val ((m <Roam-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_x_habitat-srv:make_video-val is deprecated.  Use ros_x_habitat-srv:make_video instead.")
  (make_video m))

(cl:ensure-generic-function 'video_frame_period-val :lambda-list '(m))
(cl:defmethod video_frame_period-val ((m <Roam-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_x_habitat-srv:video_frame_period-val is deprecated.  Use ros_x_habitat-srv:video_frame_period instead.")
  (video_frame_period m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Roam-request>) ostream)
  "Serializes a message object of type '<Roam-request>"
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
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'make_video) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'video_frame_period)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Roam-request>) istream)
  "Deserializes a message object of type '<Roam-request>"
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
    (cl:setf (cl:slot-value msg 'make_video) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'video_frame_period) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Roam-request>)))
  "Returns string type for a service object of type '<Roam-request>"
  "ros_x_habitat/RoamRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Roam-request)))
  "Returns string type for a service object of type 'Roam-request"
  "ros_x_habitat/RoamRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Roam-request>)))
  "Returns md5sum for a message object of type '<Roam-request>"
  "1c6692b437c77a3364ed4415992a3409")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Roam-request)))
  "Returns md5sum for a message object of type 'Roam-request"
  "1c6692b437c77a3364ed4415992a3409")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Roam-request>)))
  "Returns full string definition for message of type '<Roam-request>"
  (cl:format cl:nil "string episode_id_last~%string scene_id_last~%bool make_video~%int32 video_frame_period~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Roam-request)))
  "Returns full string definition for message of type 'Roam-request"
  (cl:format cl:nil "string episode_id_last~%string scene_id_last~%bool make_video~%int32 video_frame_period~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Roam-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'episode_id_last))
     4 (cl:length (cl:slot-value msg 'scene_id_last))
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Roam-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Roam-request
    (cl:cons ':episode_id_last (episode_id_last msg))
    (cl:cons ':scene_id_last (scene_id_last msg))
    (cl:cons ':make_video (make_video msg))
    (cl:cons ':video_frame_period (video_frame_period msg))
))
;//! \htmlinclude Roam-response.msg.html

(cl:defclass <Roam-response> (roslisp-msg-protocol:ros-message)
  ((ack
    :reader ack
    :initarg :ack
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Roam-response (<Roam-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Roam-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Roam-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_x_habitat-srv:<Roam-response> is deprecated: use ros_x_habitat-srv:Roam-response instead.")))

(cl:ensure-generic-function 'ack-val :lambda-list '(m))
(cl:defmethod ack-val ((m <Roam-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_x_habitat-srv:ack-val is deprecated.  Use ros_x_habitat-srv:ack instead.")
  (ack m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Roam-response>) ostream)
  "Serializes a message object of type '<Roam-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ack) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Roam-response>) istream)
  "Deserializes a message object of type '<Roam-response>"
    (cl:setf (cl:slot-value msg 'ack) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Roam-response>)))
  "Returns string type for a service object of type '<Roam-response>"
  "ros_x_habitat/RoamResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Roam-response)))
  "Returns string type for a service object of type 'Roam-response"
  "ros_x_habitat/RoamResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Roam-response>)))
  "Returns md5sum for a message object of type '<Roam-response>"
  "1c6692b437c77a3364ed4415992a3409")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Roam-response)))
  "Returns md5sum for a message object of type 'Roam-response"
  "1c6692b437c77a3364ed4415992a3409")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Roam-response>)))
  "Returns full string definition for message of type '<Roam-response>"
  (cl:format cl:nil "bool ack~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Roam-response)))
  "Returns full string definition for message of type 'Roam-response"
  (cl:format cl:nil "bool ack~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Roam-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Roam-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Roam-response
    (cl:cons ':ack (ack msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Roam)))
  'Roam-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Roam)))
  'Roam-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Roam)))
  "Returns string type for a service object of type '<Roam>"
  "ros_x_habitat/Roam")