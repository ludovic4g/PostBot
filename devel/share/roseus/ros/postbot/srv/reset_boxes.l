;; Auto-generated. Do not edit!


(when (boundp 'postbot::reset_boxes)
  (if (not (find-package "POSTBOT"))
    (make-package "POSTBOT"))
  (shadow 'reset_boxes (find-package "POSTBOT")))
(unless (find-package "POSTBOT::RESET_BOXES")
  (make-package "POSTBOT::RESET_BOXES"))
(unless (find-package "POSTBOT::RESET_BOXESREQUEST")
  (make-package "POSTBOT::RESET_BOXESREQUEST"))
(unless (find-package "POSTBOT::RESET_BOXESRESPONSE")
  (make-package "POSTBOT::RESET_BOXESRESPONSE"))

(in-package "ROS")





(defclass postbot::reset_boxesRequest
  :super ros::object
  :slots ())

(defmethod postbot::reset_boxesRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass postbot::reset_boxesResponse
  :super ros::object
  :slots (_done ))

(defmethod postbot::reset_boxesResponse
  (:init
   (&key
    ((:done __done) nil)
    )
   (send-super :init)
   (setq _done __done)
   self)
  (:done
   (&optional (__done :null))
   (if (not (eq __done :null)) (setq _done __done)) _done)
  (:serialization-length
   ()
   (+
    ;; bool _done
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _done
       (if _done (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _done
     (setq _done (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass postbot::reset_boxes
  :super ros::object
  :slots ())

(setf (get postbot::reset_boxes :md5sum-) "89bb254424e4cffedbf494e7b0ddbfea")
(setf (get postbot::reset_boxes :datatype-) "postbot/reset_boxes")
(setf (get postbot::reset_boxes :request) postbot::reset_boxesRequest)
(setf (get postbot::reset_boxes :response) postbot::reset_boxesResponse)

(defmethod postbot::reset_boxesRequest
  (:response () (instance postbot::reset_boxesResponse :init)))

(setf (get postbot::reset_boxesRequest :md5sum-) "89bb254424e4cffedbf494e7b0ddbfea")
(setf (get postbot::reset_boxesRequest :datatype-) "postbot/reset_boxesRequest")
(setf (get postbot::reset_boxesRequest :definition-)
      "---
bool done
")

(setf (get postbot::reset_boxesResponse :md5sum-) "89bb254424e4cffedbf494e7b0ddbfea")
(setf (get postbot::reset_boxesResponse :datatype-) "postbot/reset_boxesResponse")
(setf (get postbot::reset_boxesResponse :definition-)
      "---
bool done
")



(provide :postbot/reset_boxes "89bb254424e4cffedbf494e7b0ddbfea")


