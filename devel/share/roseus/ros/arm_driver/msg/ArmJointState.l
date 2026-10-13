;; Auto-generated. Do not edit!


(when (boundp 'arm_driver::ArmJointState)
  (if (not (find-package "ARM_DRIVER"))
    (make-package "ARM_DRIVER"))
  (shadow 'ArmJointState (find-package "ARM_DRIVER")))
(unless (find-package "ARM_DRIVER::ARMJOINTSTATE")
  (make-package "ARM_DRIVER::ARMJOINTSTATE"))

(in-package "ROS")
;;//! \htmlinclude ArmJointState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass arm_driver::ArmJointState
  :super ros::object
  :slots (_header _position _velocity _effort ))

(defmethod arm_driver::ArmJointState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:position __position) (make-array 5 :initial-element 0.0 :element-type :float))
    ((:velocity __velocity) (make-array 5 :initial-element 0.0 :element-type :float))
    ((:effort __effort) (make-array 5 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _position __position)
   (setq _velocity __velocity)
   (setq _effort __effort)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:effort
   (&optional __effort)
   (if __effort (setq _effort __effort)) _effort)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64[5] _position
    (* 8    5)
    ;; float64[5] _velocity
    (* 8    5)
    ;; float64[5] _effort
    (* 8    5)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64[5] _position
     (dotimes (i 5)
       (sys::poke (elt _position i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[5] _velocity
     (dotimes (i 5)
       (sys::poke (elt _velocity i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[5] _effort
     (dotimes (i 5)
       (sys::poke (elt _effort i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64[5] _position
   (dotimes (i (length _position))
     (setf (elt _position i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[5] _velocity
   (dotimes (i (length _velocity))
     (setf (elt _velocity i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[5] _effort
   (dotimes (i (length _effort))
     (setf (elt _effort i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get arm_driver::ArmJointState :md5sum-) "d8bb599502f81f86694a218d54e62d9e")
(setf (get arm_driver::ArmJointState :datatype-) "arm_driver/ArmJointState")
(setf (get arm_driver::ArmJointState :definition-)
      "Header header
float64[5] position
float64[5] velocity
float64[5] effort

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :arm_driver/ArmJointState "d8bb599502f81f86694a218d54e62d9e")


