; Auto-generated. Do not edit!


(cl:in-package my_robot-msg)


;//! \htmlinclude ackermann.msg.html

(cl:defclass <ackermann> (roslisp-msg-protocol:ros-message)
  ((twist
    :reader twist
    :initarg :twist
    :type geometry_msgs-msg:Twist
    :initform (cl:make-instance 'geometry_msgs-msg:Twist))
   (direction
    :reader direction
    :initarg :direction
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64)))
)

(cl:defclass ackermann (<ackermann>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ackermann>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ackermann)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name my_robot-msg:<ackermann> is deprecated: use my_robot-msg:ackermann instead.")))

(cl:ensure-generic-function 'twist-val :lambda-list '(m))
(cl:defmethod twist-val ((m <ackermann>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_robot-msg:twist-val is deprecated.  Use my_robot-msg:twist instead.")
  (twist m))

(cl:ensure-generic-function 'direction-val :lambda-list '(m))
(cl:defmethod direction-val ((m <ackermann>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_robot-msg:direction-val is deprecated.  Use my_robot-msg:direction instead.")
  (direction m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ackermann>) ostream)
  "Serializes a message object of type '<ackermann>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'twist) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'direction) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ackermann>) istream)
  "Deserializes a message object of type '<ackermann>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'twist) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'direction) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ackermann>)))
  "Returns string type for a message object of type '<ackermann>"
  "my_robot/ackermann")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ackermann)))
  "Returns string type for a message object of type 'ackermann"
  "my_robot/ackermann")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ackermann>)))
  "Returns md5sum for a message object of type '<ackermann>"
  "0bbc953f7e50c6c3cf3e95445bab2fd3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ackermann)))
  "Returns md5sum for a message object of type 'ackermann"
  "0bbc953f7e50c6c3cf3e95445bab2fd3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ackermann>)))
  "Returns full string definition for message of type '<ackermann>"
  (cl:format cl:nil "geometry_msgs/Twist twist~%std_msgs/Float64 direction~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ackermann)))
  "Returns full string definition for message of type 'ackermann"
  (cl:format cl:nil "geometry_msgs/Twist twist~%std_msgs/Float64 direction~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ackermann>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'twist))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'direction))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ackermann>))
  "Converts a ROS message object to a list"
  (cl:list 'ackermann
    (cl:cons ':twist (twist msg))
    (cl:cons ':direction (direction msg))
))
