;; Auto-generated. Do not edit!


(when (boundp 'postbot::BoxInfo)
  (if (not (find-package "POSTBOT"))
    (make-package "POSTBOT"))
  (shadow 'BoxInfo (find-package "POSTBOT")))
(unless (find-package "POSTBOT::BOXINFO")
  (make-package "POSTBOT::BOXINFO"))

(in-package "ROS")
;;//! \htmlinclude BoxInfo.msg.html


(defclass postbot::BoxInfo
  :super ros::object
  :slots (_colors _x _y _status ))

(defmethod postbot::BoxInfo
  (:init
   (&key
    ((:colors __colors) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:x __x) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:y __y) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:status __status) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _colors __colors)
   (setq _x __x)
   (setq _y __y)
   (setq _status __status)
   self)
  (:colors
   (&optional __colors)
   (if __colors (setq _colors __colors)) _colors)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; string[] _colors
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _colors)) 4
    ;; float32[] _x
    (* 4    (length _x)) 4
    ;; float32[] _y
    (* 4    (length _y)) 4
    ;; int32[] _status
    (* 4    (length _status)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _colors
     (write-long (length _colors) s)
     (dolist (elem _colors)
       (write-long (length elem) s) (princ elem s)
       )
     ;; float32[] _x
     (write-long (length _x) s)
     (dotimes (i (length _x))
       (sys::poke (elt _x i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _y
     (write-long (length _y) s)
     (dotimes (i (length _y))
       (sys::poke (elt _y i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; int32[] _status
     (write-long (length _status) s)
     (dotimes (i (length _status))
       (write-long (elt _status i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _colors
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _colors (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _colors i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; float32[] _x
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _x (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _x i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _y
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _y (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _y i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; int32[] _status
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _status (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _status i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get postbot::BoxInfo :md5sum-) "0d695438bc8e2030477534c7599a981b")
(setf (get postbot::BoxInfo :datatype-) "postbot/BoxInfo")
(setf (get postbot::BoxInfo :definition-)
      "string[] colors
float32[] x
float32[] y
int32[] status
")



(provide :postbot/BoxInfo "0d695438bc8e2030477534c7599a981b")


