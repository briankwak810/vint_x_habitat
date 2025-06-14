; Auto-generated. Do not edit!


(cl:in-package ros_x_habitat-msg)


;//! \htmlinclude PointGoalWithGPSCompass.msg.html

(cl:defclass <PointGoalWithGPSCompass> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (distance_to_goal
    :reader distance_to_goal
    :initarg :distance_to_goal
    :type cl:float
    :initform 0.0)
   (angle_to_goal
    :reader angle_to_goal
    :initarg :angle_to_goal
    :type cl:float
    :initform 0.0))
)

(cl:defclass PointGoalWithGPSCompass (<PointGoalWithGPSCompass>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PointGoalWithGPSCompass>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PointGoalWithGPSCompass)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_x_habitat-msg:<PointGoalWithGPSCompass> is deprecated: use ros_x_habitat-msg:PointGoalWithGPSCompass instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PointGoalWithGPSCompass>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_x_habitat-msg:header-val is deprecated.  Use ros_x_habitat-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'distance_to_goal-val :lambda-list '(m))
(cl:defmethod distance_to_goal-val ((m <PointGoalWithGPSCompass>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_x_habitat-msg:distance_to_goal-val is deprecated.  Use ros_x_habitat-msg:distance_to_goal instead.")
  (distance_to_goal m))

(cl:ensure-generic-function 'angle_to_goal-val :lambda-list '(m))
(cl:defmethod angle_to_goal-val ((m <PointGoalWithGPSCompass>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_x_habitat-msg:angle_to_goal-val is deprecated.  Use ros_x_habitat-msg:angle_to_goal instead.")
  (angle_to_goal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PointGoalWithGPSCompass>) ostream)
  "Serializes a message object of type '<PointGoalWithGPSCompass>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'distance_to_goal))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angle_to_goal))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PointGoalWithGPSCompass>) istream)
  "Deserializes a message object of type '<PointGoalWithGPSCompass>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance_to_goal) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle_to_goal) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PointGoalWithGPSCompass>)))
  "Returns string type for a message object of type '<PointGoalWithGPSCompass>"
  "ros_x_habitat/PointGoalWithGPSCompass")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PointGoalWithGPSCompass)))
  "Returns string type for a message object of type 'PointGoalWithGPSCompass"
  "ros_x_habitat/PointGoalWithGPSCompass")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PointGoalWithGPSCompass>)))
  "Returns md5sum for a message object of type '<PointGoalWithGPSCompass>"
  "f1c2ae08f20a219947d5152f75072a85")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PointGoalWithGPSCompass)))
  "Returns md5sum for a message object of type 'PointGoalWithGPSCompass"
  "f1c2ae08f20a219947d5152f75072a85")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PointGoalWithGPSCompass>)))
  "Returns full string definition for message of type '<PointGoalWithGPSCompass>"
  (cl:format cl:nil "# A representation of readings from Habitat's POINTGOAL_WITH_GPS_COMPASS sensor~%~%Header header~%float64 distance_to_goal~%float64 angle_to_goal # in radians~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PointGoalWithGPSCompass)))
  "Returns full string definition for message of type 'PointGoalWithGPSCompass"
  (cl:format cl:nil "# A representation of readings from Habitat's POINTGOAL_WITH_GPS_COMPASS sensor~%~%Header header~%float64 distance_to_goal~%float64 angle_to_goal # in radians~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PointGoalWithGPSCompass>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PointGoalWithGPSCompass>))
  "Converts a ROS message object to a list"
  (cl:list 'PointGoalWithGPSCompass
    (cl:cons ':header (header msg))
    (cl:cons ':distance_to_goal (distance_to_goal msg))
    (cl:cons ':angle_to_goal (angle_to_goal msg))
))
