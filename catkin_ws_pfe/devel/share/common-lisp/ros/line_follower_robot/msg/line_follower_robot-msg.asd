
(cl:in-package :asdf)

(defsystem "line_follower_robot-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "LineFollowerStatus" :depends-on ("_package_LineFollowerStatus"))
    (:file "_package_LineFollowerStatus" :depends-on ("_package"))
  ))