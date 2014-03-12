; Auto-generated. Do not edit!


(cl:in-package zuros_motor_transformation-msg)


;//! \htmlinclude differential.msg.html

(cl:defclass <differential> (roslisp-msg-protocol:ros-message)
  ((left_motor_speed
    :reader left_motor_speed
    :initarg :left_motor_speed
    :type cl:float
    :initform 0.0)
   (right_motor_speed
    :reader right_motor_speed
    :initarg :right_motor_speed
    :type cl:float
    :initform 0.0))
)

(cl:defclass differential (<differential>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <differential>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'differential)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name zuros_motor_transformation-msg:<differential> is deprecated: use zuros_motor_transformation-msg:differential instead.")))

(cl:ensure-generic-function 'left_motor_speed-val :lambda-list '(m))
(cl:defmethod left_motor_speed-val ((m <differential>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader zuros_motor_transformation-msg:left_motor_speed-val is deprecated.  Use zuros_motor_transformation-msg:left_motor_speed instead.")
  (left_motor_speed m))

(cl:ensure-generic-function 'right_motor_speed-val :lambda-list '(m))
(cl:defmethod right_motor_speed-val ((m <differential>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader zuros_motor_transformation-msg:right_motor_speed-val is deprecated.  Use zuros_motor_transformation-msg:right_motor_speed instead.")
  (right_motor_speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <differential>) ostream)
  "Serializes a message object of type '<differential>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left_motor_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_motor_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <differential>) istream)
  "Deserializes a message object of type '<differential>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_motor_speed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_motor_speed) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<differential>)))
  "Returns string type for a message object of type '<differential>"
  "zuros_motor_transformation/differential")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'differential)))
  "Returns string type for a message object of type 'differential"
  "zuros_motor_transformation/differential")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<differential>)))
  "Returns md5sum for a message object of type '<differential>"
  "64774e203c8f0b9062c05b152b874c89")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'differential)))
  "Returns md5sum for a message object of type 'differential"
  "64774e203c8f0b9062c05b152b874c89")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<differential>)))
  "Returns full string definition for message of type '<differential>"
  (cl:format cl:nil "float32 left_motor_speed~%float32 right_motor_speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'differential)))
  "Returns full string definition for message of type 'differential"
  (cl:format cl:nil "float32 left_motor_speed~%float32 right_motor_speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <differential>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <differential>))
  "Converts a ROS message object to a list"
  (cl:list 'differential
    (cl:cons ':left_motor_speed (left_motor_speed msg))
    (cl:cons ':right_motor_speed (right_motor_speed msg))
))
