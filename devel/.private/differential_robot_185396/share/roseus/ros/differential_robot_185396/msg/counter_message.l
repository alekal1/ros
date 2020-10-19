;; Auto-generated. Do not edit!


(when (boundp 'differential_robot_185396::counter_message)
  (if (not (find-package "DIFFERENTIAL_ROBOT_185396"))
    (make-package "DIFFERENTIAL_ROBOT_185396"))
  (shadow 'counter_message (find-package "DIFFERENTIAL_ROBOT_185396")))
(unless (find-package "DIFFERENTIAL_ROBOT_185396::COUNTER_MESSAGE")
  (make-package "DIFFERENTIAL_ROBOT_185396::COUNTER_MESSAGE"))

(in-package "ROS")
;;//! \htmlinclude counter_message.msg.html


(defclass differential_robot_185396::counter_message
  :super ros::object
  :slots (_count_left _count_right ))

(defmethod differential_robot_185396::counter_message
  (:init
   (&key
    ((:count_left __count_left) 0)
    ((:count_right __count_right) 0)
    )
   (send-super :init)
   (setq _count_left (round __count_left))
   (setq _count_right (round __count_right))
   self)
  (:count_left
   (&optional __count_left)
   (if __count_left (setq _count_left __count_left)) _count_left)
  (:count_right
   (&optional __count_right)
   (if __count_right (setq _count_right __count_right)) _count_right)
  (:serialization-length
   ()
   (+
    ;; int32 _count_left
    4
    ;; int32 _count_right
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _count_left
       (write-long _count_left s)
     ;; int32 _count_right
       (write-long _count_right s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _count_left
     (setq _count_left (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _count_right
     (setq _count_right (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get differential_robot_185396::counter_message :md5sum-) "9acad0024d496a45d7194e5310734a3c")
(setf (get differential_robot_185396::counter_message :datatype-) "differential_robot_185396/counter_message")
(setf (get differential_robot_185396::counter_message :definition-)
      "
int32 count_left
int32 count_right


")



(provide :differential_robot_185396/counter_message "9acad0024d496a45d7194e5310734a3c")


