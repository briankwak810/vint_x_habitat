; Auto-generated. Do not edit!


(cl:in-package ros_x_habitat-srv)


;//! \htmlinclude ResetAgent-request.msg.html

(cl:defclass <ResetAgent-request> (roslisp-msg-protocol:ros-message)
  ((reset
    :reader reset
    :initarg :reset
    :type cl:fixnum
    :initform 0)
   (seed
    :reader seed
    :initarg :seed
    :type cl:integer
    :initform 0))
)

(cl:defclass ResetAgent-request (<ResetAgent-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetAgent-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetAgent-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_x_habitat-srv:<ResetAgent-request> is deprecated: use ros_x_habitat-srv:ResetAgent-request instead.")))

(cl:ensure-generic-function 'reset-val :lambda-list '(m))
(cl:defmethod reset-val ((m <ResetAgent-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_x_habitat-srv:reset-val is deprecated.  Use ros_x_habitat-srv:reset instead.")
  (reset m))

(cl:ensure-generic-function 'seed-val :lambda-list '(m))
(cl:defmethod seed-val ((m <ResetAgent-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_x_habitat-srv:seed-val is deprecated.  Use ros_x_habitat-srv:seed instead.")
  (seed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetAgent-request>) ostream)
  "Serializes a message object of type '<ResetAgent-request>"
  (cl:let* ((signed (cl:slot-value msg 'reset)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'seed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetAgent-request>) istream)
  "Deserializes a message object of type '<ResetAgent-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'reset) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'seed) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetAgent-request>)))
  "Returns string type for a service object of type '<ResetAgent-request>"
  "ros_x_habitat/ResetAgentRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetAgent-request)))
  "Returns string type for a service object of type 'ResetAgent-request"
  "ros_x_habitat/ResetAgentRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetAgent-request>)))
  "Returns md5sum for a message object of type '<ResetAgent-request>"
  "ce8257351a1f8b1ef5fb3350a09efd5a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetAgent-request)))
  "Returns md5sum for a message object of type 'ResetAgent-request"
  "ce8257351a1f8b1ef5fb3350a09efd5a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetAgent-request>)))
  "Returns full string definition for message of type '<ResetAgent-request>"
  (cl:format cl:nil "int16 reset # 0 for reset, 1 for shutdown~%int32 seed # seed to instantiate agent~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetAgent-request)))
  "Returns full string definition for message of type 'ResetAgent-request"
  (cl:format cl:nil "int16 reset # 0 for reset, 1 for shutdown~%int32 seed # seed to instantiate agent~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetAgent-request>))
  (cl:+ 0
     2
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetAgent-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetAgent-request
    (cl:cons ':reset (reset msg))
    (cl:cons ':seed (seed msg))
))
;//! \htmlinclude ResetAgent-response.msg.html

(cl:defclass <ResetAgent-response> (roslisp-msg-protocol:ros-message)
  ((done
    :reader done
    :initarg :done
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ResetAgent-response (<ResetAgent-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetAgent-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetAgent-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_x_habitat-srv:<ResetAgent-response> is deprecated: use ros_x_habitat-srv:ResetAgent-response instead.")))

(cl:ensure-generic-function 'done-val :lambda-list '(m))
(cl:defmethod done-val ((m <ResetAgent-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_x_habitat-srv:done-val is deprecated.  Use ros_x_habitat-srv:done instead.")
  (done m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetAgent-response>) ostream)
  "Serializes a message object of type '<ResetAgent-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'done) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetAgent-response>) istream)
  "Deserializes a message object of type '<ResetAgent-response>"
    (cl:setf (cl:slot-value msg 'done) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetAgent-response>)))
  "Returns string type for a service object of type '<ResetAgent-response>"
  "ros_x_habitat/ResetAgentResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetAgent-response)))
  "Returns string type for a service object of type 'ResetAgent-response"
  "ros_x_habitat/ResetAgentResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetAgent-response>)))
  "Returns md5sum for a message object of type '<ResetAgent-response>"
  "ce8257351a1f8b1ef5fb3350a09efd5a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetAgent-response)))
  "Returns md5sum for a message object of type 'ResetAgent-response"
  "ce8257351a1f8b1ef5fb3350a09efd5a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetAgent-response>)))
  "Returns full string definition for message of type '<ResetAgent-response>"
  (cl:format cl:nil "bool done~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetAgent-response)))
  "Returns full string definition for message of type 'ResetAgent-response"
  (cl:format cl:nil "bool done~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetAgent-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetAgent-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetAgent-response
    (cl:cons ':done (done msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ResetAgent)))
  'ResetAgent-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ResetAgent)))
  'ResetAgent-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetAgent)))
  "Returns string type for a service object of type '<ResetAgent>"
  "ros_x_habitat/ResetAgent")