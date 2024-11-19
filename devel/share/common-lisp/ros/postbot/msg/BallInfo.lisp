; Auto-generated. Do not edit!


(cl:in-package postbot-msg)


;//! \htmlinclude BallInfo.msg.html

(cl:defclass <BallInfo> (roslisp-msg-protocol:ros-message)
  ((color
    :reader color
    :initarg :color
    :type cl:string
    :initform "")
   (x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0))
)

(cl:defclass BallInfo (<BallInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BallInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BallInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name postbot-msg:<BallInfo> is deprecated: use postbot-msg:BallInfo instead.")))

(cl:ensure-generic-function 'color-val :lambda-list '(m))
(cl:defmethod color-val ((m <BallInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader postbot-msg:color-val is deprecated.  Use postbot-msg:color instead.")
  (color m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <BallInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader postbot-msg:x-val is deprecated.  Use postbot-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <BallInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader postbot-msg:y-val is deprecated.  Use postbot-msg:y instead.")
  (y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BallInfo>) ostream)
  "Serializes a message object of type '<BallInfo>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'color))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'color))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BallInfo>) istream)
  "Deserializes a message object of type '<BallInfo>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'color) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'color) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BallInfo>)))
  "Returns string type for a message object of type '<BallInfo>"
  "postbot/BallInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BallInfo)))
  "Returns string type for a message object of type 'BallInfo"
  "postbot/BallInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BallInfo>)))
  "Returns md5sum for a message object of type '<BallInfo>"
  "9eab6da5b2968819e8b305355e8d814c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BallInfo)))
  "Returns md5sum for a message object of type 'BallInfo"
  "9eab6da5b2968819e8b305355e8d814c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BallInfo>)))
  "Returns full string definition for message of type '<BallInfo>"
  (cl:format cl:nil "string color~%float32 x~%float32 y~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BallInfo)))
  "Returns full string definition for message of type 'BallInfo"
  (cl:format cl:nil "string color~%float32 x~%float32 y~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BallInfo>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'color))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BallInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'BallInfo
    (cl:cons ':color (color msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
))
