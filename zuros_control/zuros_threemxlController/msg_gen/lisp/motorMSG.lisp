; Auto-generated. Do not edit!


(cl:in-package zuros_threemxlController-msg)


;//! \htmlinclude motorMSG.msg.html

(cl:defclass <motorMSG> (roslisp-msg-protocol:ros-message)
  ((left_accel
    :reader left_accel
    :initarg :left_accel
    :type cl:integer
    :initform 0)
   (left_speed
    :reader left_speed
    :initarg :left_speed
    :type cl:integer
    :initform 0)
   (right_accel
    :reader right_accel
    :initarg :right_accel
    :type cl:integer
    :initform 0)
   (right_speed
    :reader right_speed
    :initarg :right_speed
    :type cl:integer
    :initform 0))
)

(cl:defclass motorMSG (<motorMSG>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <motorMSG>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'motorMSG)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name zuros_threemxlController-msg:<motorMSG> is deprecated: use zuros_threemxlController-msg:motorMSG instead.")))

(cl:ensure-generic-function 'left_accel-val :lambda-list '(m))
(cl:defmethod left_accel-val ((m <motorMSG>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader zuros_threemxlController-msg:left_accel-val is deprecated.  Use zuros_threemxlController-msg:left_accel instead.")
  (left_accel m))

(cl:ensure-generic-function 'left_speed-val :lambda-list '(m))
(cl:defmethod left_speed-val ((m <motorMSG>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader zuros_threemxlController-msg:left_speed-val is deprecated.  Use zuros_threemxlController-msg:left_speed instead.")
  (left_speed m))

(cl:ensure-generic-function 'right_accel-val :lambda-list '(m))
(cl:defmethod right_accel-val ((m <motorMSG>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader zuros_threemxlController-msg:right_accel-val is deprecated.  Use zuros_threemxlController-msg:right_accel instead.")
  (right_accel m))

(cl:ensure-generic-function 'right_speed-val :lambda-list '(m))
(cl:defmethod right_speed-val ((m <motorMSG>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader zuros_threemxlController-msg:right_speed-val is deprecated.  Use zuros_threemxlController-msg:right_speed instead.")
  (right_speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <motorMSG>) ostream)
  "Serializes a message object of type '<motorMSG>"
  (cl:let* ((signed (cl:slot-value msg 'left_accel)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'left_speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'right_accel)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'right_speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <motorMSG>) istream)
  "Deserializes a message object of type '<motorMSG>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'left_accel) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'left_speed) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'right_accel) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'right_speed) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<motorMSG>)))
  "Returns string type for a message object of type '<motorMSG>"
  "zuros_threemxlController/motorMSG")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motorMSG)))
  "Returns string type for a message object of type 'motorMSG"
  "zuros_threemxlController/motorMSG")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<motorMSG>)))
  "Returns md5sum for a message object of type '<motorMSG>"
  "a2a9eec3fd2b159cc6afa5290066e5df")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'motorMSG)))
  "Returns md5sum for a message object of type 'motorMSG"
  "a2a9eec3fd2b159cc6afa5290066e5df")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<motorMSG>)))
  "Returns full string definition for message of type '<motorMSG>"
  (cl:format cl:nil "#left motor~%int32 left_accel~%int32 left_speed~%~%#right motor~%int32 right_accel~%int32 right_speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'motorMSG)))
  "Returns full string definition for message of type 'motorMSG"
  (cl:format cl:nil "#left motor~%int32 left_accel~%int32 left_speed~%~%#right motor~%int32 right_accel~%int32 right_speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <motorMSG>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <motorMSG>))
  "Converts a ROS message object to a list"
  (cl:list 'motorMSG
    (cl:cons ':left_accel (left_accel msg))
    (cl:cons ':left_speed (left_speed msg))
    (cl:cons ':right_accel (right_accel msg))
    (cl:cons ':right_speed (right_speed msg))
))
