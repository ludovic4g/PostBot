; Auto-generated. Do not edit!


(cl:in-package postbot-srv)


;//! \htmlinclude spawn_ball-request.msg.html

(cl:defclass <spawn_ball-request> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (color
    :reader color
    :initarg :color
    :type cl:string
    :initform ""))
)

(cl:defclass spawn_ball-request (<spawn_ball-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <spawn_ball-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'spawn_ball-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name postbot-srv:<spawn_ball-request> is deprecated: use postbot-srv:spawn_ball-request instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <spawn_ball-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader postbot-srv:x-val is deprecated.  Use postbot-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <spawn_ball-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader postbot-srv:y-val is deprecated.  Use postbot-srv:y instead.")
  (y m))

(cl:ensure-generic-function 'color-val :lambda-list '(m))
(cl:defmethod color-val ((m <spawn_ball-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader postbot-srv:color-val is deprecated.  Use postbot-srv:color instead.")
  (color m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <spawn_ball-request>) ostream)
  "Serializes a message object of type '<spawn_ball-request>"
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
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'color))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'color))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <spawn_ball-request>) istream)
  "Deserializes a message object of type '<spawn_ball-request>"
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
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'color) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'color) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<spawn_ball-request>)))
  "Returns string type for a service object of type '<spawn_ball-request>"
  "postbot/spawn_ballRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'spawn_ball-request)))
  "Returns string type for a service object of type 'spawn_ball-request"
  "postbot/spawn_ballRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<spawn_ball-request>)))
  "Returns md5sum for a message object of type '<spawn_ball-request>"
  "3b23249ad591553cd39c3c9bc459fc3d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'spawn_ball-request)))
  "Returns md5sum for a message object of type 'spawn_ball-request"
  "3b23249ad591553cd39c3c9bc459fc3d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<spawn_ball-request>)))
  "Returns full string definition for message of type '<spawn_ball-request>"
  (cl:format cl:nil "float32 x~%float32 y~%string color~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'spawn_ball-request)))
  "Returns full string definition for message of type 'spawn_ball-request"
  (cl:format cl:nil "float32 x~%float32 y~%string color~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <spawn_ball-request>))
  (cl:+ 0
     4
     4
     4 (cl:length (cl:slot-value msg 'color))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <spawn_ball-request>))
  "Converts a ROS message object to a list"
  (cl:list 'spawn_ball-request
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':color (color msg))
))
;//! \htmlinclude spawn_ball-response.msg.html

(cl:defclass <spawn_ball-response> (roslisp-msg-protocol:ros-message)
  ((done
    :reader done
    :initarg :done
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass spawn_ball-response (<spawn_ball-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <spawn_ball-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'spawn_ball-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name postbot-srv:<spawn_ball-response> is deprecated: use postbot-srv:spawn_ball-response instead.")))

(cl:ensure-generic-function 'done-val :lambda-list '(m))
(cl:defmethod done-val ((m <spawn_ball-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader postbot-srv:done-val is deprecated.  Use postbot-srv:done instead.")
  (done m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <spawn_ball-response>) ostream)
  "Serializes a message object of type '<spawn_ball-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'done) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <spawn_ball-response>) istream)
  "Deserializes a message object of type '<spawn_ball-response>"
    (cl:setf (cl:slot-value msg 'done) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<spawn_ball-response>)))
  "Returns string type for a service object of type '<spawn_ball-response>"
  "postbot/spawn_ballResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'spawn_ball-response)))
  "Returns string type for a service object of type 'spawn_ball-response"
  "postbot/spawn_ballResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<spawn_ball-response>)))
  "Returns md5sum for a message object of type '<spawn_ball-response>"
  "3b23249ad591553cd39c3c9bc459fc3d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'spawn_ball-response)))
  "Returns md5sum for a message object of type 'spawn_ball-response"
  "3b23249ad591553cd39c3c9bc459fc3d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<spawn_ball-response>)))
  "Returns full string definition for message of type '<spawn_ball-response>"
  (cl:format cl:nil "bool done~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'spawn_ball-response)))
  "Returns full string definition for message of type 'spawn_ball-response"
  (cl:format cl:nil "bool done~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <spawn_ball-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <spawn_ball-response>))
  "Converts a ROS message object to a list"
  (cl:list 'spawn_ball-response
    (cl:cons ':done (done msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'spawn_ball)))
  'spawn_ball-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'spawn_ball)))
  'spawn_ball-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'spawn_ball)))
  "Returns string type for a service object of type '<spawn_ball>"
  "postbot/spawn_ball")