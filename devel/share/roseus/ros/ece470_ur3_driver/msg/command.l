;; Auto-generated. Do not edit!


(when (boundp 'ece470_ur3_driver::command)
  (if (not (find-package "ECE470_UR3_DRIVER"))
    (make-package "ECE470_UR3_DRIVER"))
  (shadow 'command (find-package "ECE470_UR3_DRIVER")))
(unless (find-package "ECE470_UR3_DRIVER::COMMAND")
  (make-package "ECE470_UR3_DRIVER::COMMAND"))

(in-package "ROS")
;;//! \htmlinclude command.msg.html


(defclass ece470_ur3_driver::command
  :super ros::object
  :slots (_destination _grip _softhome _duration ))

(defmethod ece470_ur3_driver::command
  (:init
   (&key
    ((:destination __destination) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:grip __grip) 0)
    ((:softhome __softhome) 0)
    ((:duration __duration) 0.0)
    )
   (send-super :init)
   (setq _destination __destination)
   (setq _grip (round __grip))
   (setq _softhome (round __softhome))
   (setq _duration (float __duration))
   self)
  (:destination
   (&optional __destination)
   (if __destination (setq _destination __destination)) _destination)
  (:grip
   (&optional __grip)
   (if __grip (setq _grip __grip)) _grip)
  (:softhome
   (&optional __softhome)
   (if __softhome (setq _softhome __softhome)) _softhome)
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:serialization-length
   ()
   (+
    ;; float64[] _destination
    (* 8    (length _destination)) 4
    ;; int32 _grip
    4
    ;; int32 _softhome
    4
    ;; float64 _duration
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _destination
     (write-long (length _destination) s)
     (dotimes (i (length _destination))
       (sys::poke (elt _destination i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; int32 _grip
       (write-long _grip s)
     ;; int32 _softhome
       (write-long _softhome s)
     ;; float64 _duration
       (sys::poke _duration (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _destination
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _destination (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _destination i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; int32 _grip
     (setq _grip (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _softhome
     (setq _softhome (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _duration
     (setq _duration (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get ece470_ur3_driver::command :md5sum-) "2d2c2700daec81019d4338afb533bad6")
(setf (get ece470_ur3_driver::command :datatype-) "ece470_ur3_driver/command")
(setf (get ece470_ur3_driver::command :definition-)
      "float64[] destination
int32 grip
int32 softhome
float64 duration

")



(provide :ece470_ur3_driver/command "2d2c2700daec81019d4338afb533bad6")


