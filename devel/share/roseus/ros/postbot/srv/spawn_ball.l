;; Auto-generated. Do not edit!


(when (boundp 'postbot::spawn_ball)
  (if (not (find-package "POSTBOT"))
    (make-package "POSTBOT"))
  (shadow 'spawn_ball (find-package "POSTBOT")))
(unless (find-package "POSTBOT::SPAWN_BALL")
  (make-package "POSTBOT::SPAWN_BALL"))
(unless (find-package "POSTBOT::SPAWN_BALLREQUEST")
  (make-package "POSTBOT::SPAWN_BALLREQUEST"))
(unless (find-package "POSTBOT::SPAWN_BALLRESPONSE")
  (make-package "POSTBOT::SPAWN_BALLRESPONSE"))

(in-package "ROS")





(defclass postbot::spawn_ballRequest
  :super ros::object
  :slots (_x _y _color ))

(defmethod postbot::spawn_ballRequest
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:color __color) "")
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _color (string __color))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:color
   (&optional __color)
   (if __color (setq _color __color)) _color)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; string _color
    4 (length _color)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _color
       (write-long (length _color) s) (princ _color s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _color
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _color (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass postbot::spawn_ballResponse
  :super ros::object
  :slots (_done ))

(defmethod postbot::spawn_ballResponse
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

(defclass postbot::spawn_ball
  :super ros::object
  :slots ())

(setf (get postbot::spawn_ball :md5sum-) "3b23249ad591553cd39c3c9bc459fc3d")
(setf (get postbot::spawn_ball :datatype-) "postbot/spawn_ball")
(setf (get postbot::spawn_ball :request) postbot::spawn_ballRequest)
(setf (get postbot::spawn_ball :response) postbot::spawn_ballResponse)

(defmethod postbot::spawn_ballRequest
  (:response () (instance postbot::spawn_ballResponse :init)))

(setf (get postbot::spawn_ballRequest :md5sum-) "3b23249ad591553cd39c3c9bc459fc3d")
(setf (get postbot::spawn_ballRequest :datatype-) "postbot/spawn_ballRequest")
(setf (get postbot::spawn_ballRequest :definition-)
      "float32 x
float32 y
string color
---
bool done
")

(setf (get postbot::spawn_ballResponse :md5sum-) "3b23249ad591553cd39c3c9bc459fc3d")
(setf (get postbot::spawn_ballResponse :datatype-) "postbot/spawn_ballResponse")
(setf (get postbot::spawn_ballResponse :definition-)
      "float32 x
float32 y
string color
---
bool done
")



(provide :postbot/spawn_ball "3b23249ad591553cd39c3c9bc459fc3d")


