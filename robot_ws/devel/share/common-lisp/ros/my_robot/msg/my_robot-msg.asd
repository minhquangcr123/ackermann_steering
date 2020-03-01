
(cl:in-package :asdf)

(defsystem "my_robot-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "ackermann" :depends-on ("_package_ackermann"))
    (:file "_package_ackermann" :depends-on ("_package"))
  ))