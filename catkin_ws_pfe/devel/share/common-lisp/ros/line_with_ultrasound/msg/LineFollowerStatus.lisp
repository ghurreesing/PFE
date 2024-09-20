; Auto-generated. Do not edit!


(cl:in-package line_with_ultrasound-msg)


;//! \htmlinclude LineFollowerStatus.msg.html

(cl:defclass <LineFollowerStatus> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass LineFollowerStatus (<LineFollowerStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LineFollowerStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LineFollowerStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name line_with_ultrasound-msg:<LineFollowerStatus> is deprecated: use line_with_ultrasound-msg:LineFollowerStatus instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <LineFollowerStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader line_with_ultrasound-msg:status-val is deprecated.  Use line_with_ultrasound-msg:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LineFollowerStatus>) ostream)
  "Serializes a message object of type '<LineFollowerStatus>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'status))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LineFollowerStatus>) istream)
  "Deserializes a message object of type '<LineFollowerStatus>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'status) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'status)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LineFollowerStatus>)))
  "Returns string type for a message object of type '<LineFollowerStatus>"
  "line_with_ultrasound/LineFollowerStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LineFollowerStatus)))
  "Returns string type for a message object of type 'LineFollowerStatus"
  "line_with_ultrasound/LineFollowerStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LineFollowerStatus>)))
  "Returns md5sum for a message object of type '<LineFollowerStatus>"
  "672ccc7edd3a5621529e608443618e1f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LineFollowerStatus)))
  "Returns md5sum for a message object of type 'LineFollowerStatus"
  "672ccc7edd3a5621529e608443618e1f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LineFollowerStatus>)))
  "Returns full string definition for message of type '<LineFollowerStatus>"
  (cl:format cl:nil "int32[] status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LineFollowerStatus)))
  "Returns full string definition for message of type 'LineFollowerStatus"
  (cl:format cl:nil "int32[] status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LineFollowerStatus>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'status) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LineFollowerStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'LineFollowerStatus
    (cl:cons ':status (status msg))
))
