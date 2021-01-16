;; Auto-generated. Do not edit!


(when (boundp 'my_robot::ackermann)
  (if (not (find-package "MY_ROBOT"))
    (make-package "MY_ROBOT"))
  (shadow 'ackermann (find-package "MY_ROBOT")))
(unless (find-package "MY_ROBOT::ACKERMANN")
  (make-package "MY_ROBOT::ACKERMANN"))

(in-package "ROS")
;;//! \htmlinclude ackermann.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass my_robot::ackermann
  :super ros::object
  :slots (_twist _direction ))

(defmethod my_robot::ackermann
  (:init
   (&key
    ((:twist __twist) (instance geometry_msgs::Twist :init))
    ((:direction __direction) (instance std_msgs::Float64 :init))
    )
   (send-super :init)
   (setq _twist __twist)
   (setq _direction __direction)
   self)
  (:twist
   (&rest __twist)
   (if (keywordp (car __twist))
       (send* _twist __twist)
     (progn
       (if __twist (setq _twist (car __twist)))
       _twist)))
  (:direction
   (&rest __direction)
   (if (keywordp (car __direction))
       (send* _direction __direction)
     (progn
       (if __direction (setq _direction (car __direction)))
       _direction)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Twist _twist
    (send _twist :serialization-length)
    ;; std_msgs/Float64 _direction
    (send _direction :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Twist _twist
       (send _twist :serialize s)
     ;; std_msgs/Float64 _direction
       (send _direction :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Twist _twist
     (send _twist :deserialize buf ptr-) (incf ptr- (send _twist :serialization-length))
   ;; std_msgs/Float64 _direction
     (send _direction :deserialize buf ptr-) (incf ptr- (send _direction :serialization-length))
   ;;
   self)
  )

(setf (get my_robot::ackermann :md5sum-) "0bbc953f7e50c6c3cf3e95445bab2fd3")
(setf (get my_robot::ackermann :datatype-) "my_robot/ackermann")
(setf (get my_robot::ackermann :definition-)
      "geometry_msgs/Twist twist
std_msgs/Float64 direction

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Float64
float64 data
")



(provide :my_robot/ackermann "0bbc953f7e50c6c3cf3e95445bab2fd3")


