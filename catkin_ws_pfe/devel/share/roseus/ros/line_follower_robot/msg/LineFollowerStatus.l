;; Auto-generated. Do not edit!


(when (boundp 'line_follower_robot::LineFollowerStatus)
  (if (not (find-package "LINE_FOLLOWER_ROBOT"))
    (make-package "LINE_FOLLOWER_ROBOT"))
  (shadow 'LineFollowerStatus (find-package "LINE_FOLLOWER_ROBOT")))
(unless (find-package "LINE_FOLLOWER_ROBOT::LINEFOLLOWERSTATUS")
  (make-package "LINE_FOLLOWER_ROBOT::LINEFOLLOWERSTATUS"))

(in-package "ROS")
;;//! \htmlinclude LineFollowerStatus.msg.html


(defclass line_follower_robot::LineFollowerStatus
  :super ros::object
  :slots (_status ))

(defmethod line_follower_robot::LineFollowerStatus
  (:init
   (&key
    ((:status __status) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _status __status)
   self)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; int32[] _status
    (* 4    (length _status)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32[] _status
     (write-long (length _status) s)
     (dotimes (i (length _status))
       (write-long (elt _status i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32[] _status
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _status (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _status i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get line_follower_robot::LineFollowerStatus :md5sum-) "672ccc7edd3a5621529e608443618e1f")
(setf (get line_follower_robot::LineFollowerStatus :datatype-) "line_follower_robot/LineFollowerStatus")
(setf (get line_follower_robot::LineFollowerStatus :definition-)
      "int32[] status

")



(provide :line_follower_robot/LineFollowerStatus "672ccc7edd3a5621529e608443618e1f")


