;; Auto-generated. Do not edit!


(when (boundp 'ece470_ur3_driver::positions)
  (if (not (find-package "ECE470_UR3_DRIVER"))
    (make-package "ECE470_UR3_DRIVER"))
  (shadow 'positions (find-package "ECE470_UR3_DRIVER")))
(unless (find-package "ECE470_UR3_DRIVER::POSITIONS")
  (make-package "ECE470_UR3_DRIVER::POSITIONS"))

(in-package "ROS")
;;//! \htmlinclude positions.msg.html


(defclass ece470_ur3_driver::positions
  :super ros::object
  :slots (_position _velocity _grip _isReady _state _pending ))

(defmethod ece470_ur3_driver::positions
  (:init
   (&key
    ((:position __position) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:velocity __velocity) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:grip __grip) 0)
    ((:isReady __isReady) nil)
    ((:state __state) "")
    ((:pending __pending) nil)
    )
   (send-super :init)
   (setq _position __position)
   (setq _velocity __velocity)
   (setq _grip (round __grip))
   (setq _isReady __isReady)
   (setq _state (string __state))
   (setq _pending __pending)
   self)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:grip
   (&optional __grip)
   (if __grip (setq _grip __grip)) _grip)
  (:isReady
   (&optional __isReady)
   (if __isReady (setq _isReady __isReady)) _isReady)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:pending
   (&optional __pending)
   (if __pending (setq _pending __pending)) _pending)
  (:serialization-length
   ()
   (+
    ;; float64[] _position
    (* 8    (length _position)) 4
    ;; float64[] _velocity
    (* 8    (length _velocity)) 4
    ;; int32 _grip
    4
    ;; bool _isReady
    1
    ;; string _state
    4 (length _state)
    ;; bool _pending
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _position
     (write-long (length _position) s)
     (dotimes (i (length _position))
       (sys::poke (elt _position i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _velocity
     (write-long (length _velocity) s)
     (dotimes (i (length _velocity))
       (sys::poke (elt _velocity i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; int32 _grip
       (write-long _grip s)
     ;; bool _isReady
       (if _isReady (write-byte -1 s) (write-byte 0 s))
     ;; string _state
       (write-long (length _state) s) (princ _state s)
     ;; bool _pending
       (if _pending (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _position
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _position (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _position i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _velocity
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _velocity (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _velocity i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; int32 _grip
     (setq _grip (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _isReady
     (setq _isReady (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _state
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _state (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _pending
     (setq _pending (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get ece470_ur3_driver::positions :md5sum-) "bf2455dc51ac9b75c8e422304ce3ab30")
(setf (get ece470_ur3_driver::positions :datatype-) "ece470_ur3_driver/positions")
(setf (get ece470_ur3_driver::positions :definition-)
      "float64[] position
float64[] velocity
int32 grip
bool isReady
string state
bool pending

")



(provide :ece470_ur3_driver/positions "bf2455dc51ac9b75c8e422304ce3ab30")


