; Auto-generated. Do not edit!


(cl:in-package ros_x_habitat-srv)


;//! \htmlinclude GetAgentPose-request.msg.html

(cl:defclass <GetAgentPose-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetAgentPose-request (<GetAgentPose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAgentPose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAgentPose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_x_habitat-srv:<GetAgentPose-request> is deprecated: use ros_x_habitat-srv:GetAgentPose-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAgentPose-request>) ostream)
  "Serializes a message object of type '<GetAgentPose-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAgentPose-request>) istream)
  "Deserializes a message object of type '<GetAgentPose-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAgentPose-request>)))
  "Returns string type for a service object of type '<GetAgentPose-request>"
  "ros_x_habitat/GetAgentPoseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAgentPose-request)))
  "Returns string type for a service object of type 'GetAgentPose-request"
  "ros_x_habitat/GetAgentPoseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAgentPose-request>)))
  "Returns md5sum for a message object of type '<GetAgentPose-request>"
  "f192399f711a48924df9a394d37edd67")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAgentPose-request)))
  "Returns md5sum for a message object of type 'GetAgentPose-request"
  "f192399f711a48924df9a394d37edd67")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAgentPose-request>)))
  "Returns full string definition for message of type '<GetAgentPose-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAgentPose-request)))
  "Returns full string definition for message of type 'GetAgentPose-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAgentPose-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAgentPose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAgentPose-request
))
;//! \htmlinclude GetAgentPose-response.msg.html

(cl:defclass <GetAgentPose-response> (roslisp-msg-protocol:ros-message)
  ((pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass GetAgentPose-response (<GetAgentPose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAgentPose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAgentPose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_x_habitat-srv:<GetAgentPose-response> is deprecated: use ros_x_habitat-srv:GetAgentPose-response instead.")))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <GetAgentPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_x_habitat-srv:pose-val is deprecated.  Use ros_x_habitat-srv:pose instead.")
  (pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAgentPose-response>) ostream)
  "Serializes a message object of type '<GetAgentPose-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAgentPose-response>) istream)
  "Deserializes a message object of type '<GetAgentPose-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAgentPose-response>)))
  "Returns string type for a service object of type '<GetAgentPose-response>"
  "ros_x_habitat/GetAgentPoseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAgentPose-response)))
  "Returns string type for a service object of type 'GetAgentPose-response"
  "ros_x_habitat/GetAgentPoseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAgentPose-response>)))
  "Returns md5sum for a message object of type '<GetAgentPose-response>"
  "f192399f711a48924df9a394d37edd67")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAgentPose-response)))
  "Returns md5sum for a message object of type 'GetAgentPose-response"
  "f192399f711a48924df9a394d37edd67")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAgentPose-response>)))
  "Returns full string definition for message of type '<GetAgentPose-response>"
  (cl:format cl:nil "geometry_msgs/Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAgentPose-response)))
  "Returns full string definition for message of type 'GetAgentPose-response"
  (cl:format cl:nil "geometry_msgs/Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAgentPose-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAgentPose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAgentPose-response
    (cl:cons ':pose (pose msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetAgentPose)))
  'GetAgentPose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetAgentPose)))
  'GetAgentPose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAgentPose)))
  "Returns string type for a service object of type '<GetAgentPose>"
  "ros_x_habitat/GetAgentPose")