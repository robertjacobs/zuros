; Auto-generated. Do not edit!


(cl:in-package zuros_laptop-msg)


;//! \htmlinclude MSG_LAPTOP_BATTERY.msg.html

(cl:defclass <MSG_LAPTOP_BATTERY> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (state
    :reader state
    :initarg :state
    :type cl:string
    :initform "")
   (percentage
    :reader percentage
    :initarg :percentage
    :type cl:string
    :initform "")
   (remaining
    :reader remaining
    :initarg :remaining
    :type cl:string
    :initform ""))
)

(cl:defclass MSG_LAPTOP_BATTERY (<MSG_LAPTOP_BATTERY>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MSG_LAPTOP_BATTERY>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MSG_LAPTOP_BATTERY)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name zuros_laptop-msg:<MSG_LAPTOP_BATTERY> is deprecated: use zuros_laptop-msg:MSG_LAPTOP_BATTERY instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <MSG_LAPTOP_BATTERY>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader zuros_laptop-msg:name-val is deprecated.  Use zuros_laptop-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <MSG_LAPTOP_BATTERY>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader zuros_laptop-msg:state-val is deprecated.  Use zuros_laptop-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'percentage-val :lambda-list '(m))
(cl:defmethod percentage-val ((m <MSG_LAPTOP_BATTERY>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader zuros_laptop-msg:percentage-val is deprecated.  Use zuros_laptop-msg:percentage instead.")
  (percentage m))

(cl:ensure-generic-function 'remaining-val :lambda-list '(m))
(cl:defmethod remaining-val ((m <MSG_LAPTOP_BATTERY>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader zuros_laptop-msg:remaining-val is deprecated.  Use zuros_laptop-msg:remaining instead.")
  (remaining m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MSG_LAPTOP_BATTERY>) ostream)
  "Serializes a message object of type '<MSG_LAPTOP_BATTERY>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'state))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'percentage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'percentage))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'remaining))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'remaining))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MSG_LAPTOP_BATTERY>) istream)
  "Deserializes a message object of type '<MSG_LAPTOP_BATTERY>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'state) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'state) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'percentage) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'percentage) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'remaining) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'remaining) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MSG_LAPTOP_BATTERY>)))
  "Returns string type for a message object of type '<MSG_LAPTOP_BATTERY>"
  "zuros_laptop/MSG_LAPTOP_BATTERY")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MSG_LAPTOP_BATTERY)))
  "Returns string type for a message object of type 'MSG_LAPTOP_BATTERY"
  "zuros_laptop/MSG_LAPTOP_BATTERY")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MSG_LAPTOP_BATTERY>)))
  "Returns md5sum for a message object of type '<MSG_LAPTOP_BATTERY>"
  "8d8fe15d198a3ee81e16217998944f3e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MSG_LAPTOP_BATTERY)))
  "Returns md5sum for a message object of type 'MSG_LAPTOP_BATTERY"
  "8d8fe15d198a3ee81e16217998944f3e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MSG_LAPTOP_BATTERY>)))
  "Returns full string definition for message of type '<MSG_LAPTOP_BATTERY>"
  (cl:format cl:nil "string name~%string state~%string percentage~%string remaining~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MSG_LAPTOP_BATTERY)))
  "Returns full string definition for message of type 'MSG_LAPTOP_BATTERY"
  (cl:format cl:nil "string name~%string state~%string percentage~%string remaining~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MSG_LAPTOP_BATTERY>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4 (cl:length (cl:slot-value msg 'state))
     4 (cl:length (cl:slot-value msg 'percentage))
     4 (cl:length (cl:slot-value msg 'remaining))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MSG_LAPTOP_BATTERY>))
  "Converts a ROS message object to a list"
  (cl:list 'MSG_LAPTOP_BATTERY
    (cl:cons ':name (name msg))
    (cl:cons ':state (state msg))
    (cl:cons ':percentage (percentage msg))
    (cl:cons ':remaining (remaining msg))
))
