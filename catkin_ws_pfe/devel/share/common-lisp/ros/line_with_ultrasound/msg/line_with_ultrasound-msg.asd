
(cl:in-package :asdf)

(defsystem "line_with_ultrasound-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "LineFollowerStatus" :depends-on ("_package_LineFollowerStatus"))
    (:file "_package_LineFollowerStatus" :depends-on ("_package"))
  ))