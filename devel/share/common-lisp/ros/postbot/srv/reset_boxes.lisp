; Auto-generated. Do not edit!


(cl:in-package postbot-srv)


;//! \htmlinclude reset_boxes-request.msg.html

(cl:defclass <reset_boxes-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass reset_boxes-request (<reset_boxes-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <reset_boxes-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'reset_boxes-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name postbot-srv:<reset_boxes-request> is deprecated: use postbot-srv:reset_boxes-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <reset_boxes-request>) ostream)
  "Serializes a message object of type '<reset_boxes-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <reset_boxes-request>) istream)
  "Deserializes a message object of type '<reset_boxes-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<reset_boxes-request>)))
  "Returns string type for a service object of type '<reset_boxes-request>"
  "postbot/reset_boxesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'reset_boxes-request)))
  "Returns string type for a service object of type 'reset_boxes-request"
  "postbot/reset_boxesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<reset_boxes-request>)))
  "Returns md5sum for a message object of type '<reset_boxes-request>"
  "89bb254424e4cffedbf494e7b0ddbfea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'reset_boxes-request)))
  "Returns md5sum for a message object of type 'reset_boxes-request"
  "89bb254424e4cffedbf494e7b0ddbfea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<reset_boxes-request>)))
  "Returns full string definition for message of type '<reset_boxes-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'reset_boxes-request)))
  "Returns full string definition for message of type 'reset_boxes-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <reset_boxes-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <reset_boxes-request>))
  "Converts a ROS message object to a list"
  (cl:list 'reset_boxes-request
))
;//! \htmlinclude reset_boxes-response.msg.html

(cl:defclass <reset_boxes-response> (roslisp-msg-protocol:ros-message)
  ((done
    :reader done
    :initarg :done
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass reset_boxes-response (<reset_boxes-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <reset_boxes-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'reset_boxes-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name postbot-srv:<reset_boxes-response> is deprecated: use postbot-srv:reset_boxes-response instead.")))

(cl:ensure-generic-function 'done-val :lambda-list '(m))
(cl:defmethod done-val ((m <reset_boxes-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader postbot-srv:done-val is deprecated.  Use postbot-srv:done instead.")
  (done m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <reset_boxes-response>) ostream)
  "Serializes a message object of type '<reset_boxes-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'done) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <reset_boxes-response>) istream)
  "Deserializes a message object of type '<reset_boxes-response>"
    (cl:setf (cl:slot-value msg 'done) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<reset_boxes-response>)))
  "Returns string type for a service object of type '<reset_boxes-response>"
  "postbot/reset_boxesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'reset_boxes-response)))
  "Returns string type for a service object of type 'reset_boxes-response"
  "postbot/reset_boxesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<reset_boxes-response>)))
  "Returns md5sum for a message object of type '<reset_boxes-response>"
  "89bb254424e4cffedbf494e7b0ddbfea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'reset_boxes-response)))
  "Returns md5sum for a message object of type 'reset_boxes-response"
  "89bb254424e4cffedbf494e7b0ddbfea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<reset_boxes-response>)))
  "Returns full string definition for message of type '<reset_boxes-response>"
  (cl:format cl:nil "bool done~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'reset_boxes-response)))
  "Returns full string definition for message of type 'reset_boxes-response"
  (cl:format cl:nil "bool done~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <reset_boxes-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <reset_boxes-response>))
  "Converts a ROS message object to a list"
  (cl:list 'reset_boxes-response
    (cl:cons ':done (done msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'reset_boxes)))
  'reset_boxes-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'reset_boxes)))
  'reset_boxes-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'reset_boxes)))
  "Returns string type for a service object of type '<reset_boxes>"
  "postbot/reset_boxes")