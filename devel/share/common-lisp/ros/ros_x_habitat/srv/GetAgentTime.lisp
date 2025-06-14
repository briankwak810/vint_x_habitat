; Auto-generated. Do not edit!


(cl:in-package ros_x_habitat-srv)


;//! \htmlinclude GetAgentTime-request.msg.html

(cl:defclass <GetAgentTime-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetAgentTime-request (<GetAgentTime-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAgentTime-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAgentTime-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_x_habitat-srv:<GetAgentTime-request> is deprecated: use ros_x_habitat-srv:GetAgentTime-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAgentTime-request>) ostream)
  "Serializes a message object of type '<GetAgentTime-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAgentTime-request>) istream)
  "Deserializes a message object of type '<GetAgentTime-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAgentTime-request>)))
  "Returns string type for a service object of type '<GetAgentTime-request>"
  "ros_x_habitat/GetAgentTimeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAgentTime-request)))
  "Returns string type for a service object of type 'GetAgentTime-request"
  "ros_x_habitat/GetAgentTimeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAgentTime-request>)))
  "Returns md5sum for a message object of type '<GetAgentTime-request>"
  "823e9883d7a51ebd6a56062eea85e4b1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAgentTime-request)))
  "Returns md5sum for a message object of type 'GetAgentTime-request"
  "823e9883d7a51ebd6a56062eea85e4b1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAgentTime-request>)))
  "Returns full string definition for message of type '<GetAgentTime-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAgentTime-request)))
  "Returns full string definition for message of type 'GetAgentTime-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAgentTime-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAgentTime-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAgentTime-request
))
;//! \htmlinclude GetAgentTime-response.msg.html

(cl:defclass <GetAgentTime-response> (roslisp-msg-protocol:ros-message)
  ((agent_time
    :reader agent_time
    :initarg :agent_time
    :type cl:float
    :initform 0.0))
)

(cl:defclass GetAgentTime-response (<GetAgentTime-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAgentTime-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAgentTime-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_x_habitat-srv:<GetAgentTime-response> is deprecated: use ros_x_habitat-srv:GetAgentTime-response instead.")))

(cl:ensure-generic-function 'agent_time-val :lambda-list '(m))
(cl:defmethod agent_time-val ((m <GetAgentTime-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_x_habitat-srv:agent_time-val is deprecated.  Use ros_x_habitat-srv:agent_time instead.")
  (agent_time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAgentTime-response>) ostream)
  "Serializes a message object of type '<GetAgentTime-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'agent_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAgentTime-response>) istream)
  "Deserializes a message object of type '<GetAgentTime-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'agent_time) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAgentTime-response>)))
  "Returns string type for a service object of type '<GetAgentTime-response>"
  "ros_x_habitat/GetAgentTimeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAgentTime-response)))
  "Returns string type for a service object of type 'GetAgentTime-response"
  "ros_x_habitat/GetAgentTimeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAgentTime-response>)))
  "Returns md5sum for a message object of type '<GetAgentTime-response>"
  "823e9883d7a51ebd6a56062eea85e4b1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAgentTime-response)))
  "Returns md5sum for a message object of type 'GetAgentTime-response"
  "823e9883d7a51ebd6a56062eea85e4b1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAgentTime-response>)))
  "Returns full string definition for message of type '<GetAgentTime-response>"
  (cl:format cl:nil "float32 agent_time~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAgentTime-response)))
  "Returns full string definition for message of type 'GetAgentTime-response"
  (cl:format cl:nil "float32 agent_time~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAgentTime-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAgentTime-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAgentTime-response
    (cl:cons ':agent_time (agent_time msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetAgentTime)))
  'GetAgentTime-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetAgentTime)))
  'GetAgentTime-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAgentTime)))
  "Returns string type for a service object of type '<GetAgentTime>"
  "ros_x_habitat/GetAgentTime")