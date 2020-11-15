; Auto-generated. Do not edit!


(cl:in-package differential_robot_sample-msg)


;//! \htmlinclude counter_message.msg.html

(cl:defclass <counter_message> (roslisp-msg-protocol:ros-message)
  ((count_left
    :reader count_left
    :initarg :count_left
    :type cl:integer
    :initform 0)
   (count_right
    :reader count_right
    :initarg :count_right
    :type cl:integer
    :initform 0))
)

(cl:defclass counter_message (<counter_message>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <counter_message>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'counter_message)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name differential_robot_sample-msg:<counter_message> is deprecated: use differential_robot_sample-msg:counter_message instead.")))

(cl:ensure-generic-function 'count_left-val :lambda-list '(m))
(cl:defmethod count_left-val ((m <counter_message>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_robot_sample-msg:count_left-val is deprecated.  Use differential_robot_sample-msg:count_left instead.")
  (count_left m))

(cl:ensure-generic-function 'count_right-val :lambda-list '(m))
(cl:defmethod count_right-val ((m <counter_message>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader differential_robot_sample-msg:count_right-val is deprecated.  Use differential_robot_sample-msg:count_right instead.")
  (count_right m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <counter_message>) ostream)
  "Serializes a message object of type '<counter_message>"
  (cl:let* ((signed (cl:slot-value msg 'count_left)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'count_right)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <counter_message>) istream)
  "Deserializes a message object of type '<counter_message>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'count_left) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'count_right) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<counter_message>)))
  "Returns string type for a message object of type '<counter_message>"
  "differential_robot_sample/counter_message")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'counter_message)))
  "Returns string type for a message object of type 'counter_message"
  "differential_robot_sample/counter_message")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<counter_message>)))
  "Returns md5sum for a message object of type '<counter_message>"
  "9acad0024d496a45d7194e5310734a3c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'counter_message)))
  "Returns md5sum for a message object of type 'counter_message"
  "9acad0024d496a45d7194e5310734a3c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<counter_message>)))
  "Returns full string definition for message of type '<counter_message>"
  (cl:format cl:nil "~%int32 count_left~%int32 count_right~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'counter_message)))
  "Returns full string definition for message of type 'counter_message"
  (cl:format cl:nil "~%int32 count_left~%int32 count_right~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <counter_message>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <counter_message>))
  "Converts a ROS message object to a list"
  (cl:list 'counter_message
    (cl:cons ':count_left (count_left msg))
    (cl:cons ':count_right (count_right msg))
))
