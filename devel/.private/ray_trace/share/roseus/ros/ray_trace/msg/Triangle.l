;; Auto-generated. Do not edit!


(when (boundp 'ray_trace::Triangle)
  (if (not (find-package "RAY_TRACE"))
    (make-package "RAY_TRACE"))
  (shadow 'Triangle (find-package "RAY_TRACE")))
(unless (find-package "RAY_TRACE::TRIANGLE")
  (make-package "RAY_TRACE::TRIANGLE"))

(in-package "ROS")
;;//! \htmlinclude Triangle.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass ray_trace::Triangle
  :super ros::object
  :slots (_p1 _p2 _p3 ))

(defmethod ray_trace::Triangle
  (:init
   (&key
    ((:p1 __p1) (instance geometry_msgs::Point :init))
    ((:p2 __p2) (instance geometry_msgs::Point :init))
    ((:p3 __p3) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _p1 __p1)
   (setq _p2 __p2)
   (setq _p3 __p3)
   self)
  (:p1
   (&rest __p1)
   (if (keywordp (car __p1))
       (send* _p1 __p1)
     (progn
       (if __p1 (setq _p1 (car __p1)))
       _p1)))
  (:p2
   (&rest __p2)
   (if (keywordp (car __p2))
       (send* _p2 __p2)
     (progn
       (if __p2 (setq _p2 (car __p2)))
       _p2)))
  (:p3
   (&rest __p3)
   (if (keywordp (car __p3))
       (send* _p3 __p3)
     (progn
       (if __p3 (setq _p3 (car __p3)))
       _p3)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _p1
    (send _p1 :serialization-length)
    ;; geometry_msgs/Point _p2
    (send _p2 :serialization-length)
    ;; geometry_msgs/Point _p3
    (send _p3 :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _p1
       (send _p1 :serialize s)
     ;; geometry_msgs/Point _p2
       (send _p2 :serialize s)
     ;; geometry_msgs/Point _p3
       (send _p3 :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _p1
     (send _p1 :deserialize buf ptr-) (incf ptr- (send _p1 :serialization-length))
   ;; geometry_msgs/Point _p2
     (send _p2 :deserialize buf ptr-) (incf ptr- (send _p2 :serialization-length))
   ;; geometry_msgs/Point _p3
     (send _p3 :deserialize buf ptr-) (incf ptr- (send _p3 :serialization-length))
   ;;
   self)
  )

(setf (get ray_trace::Triangle :md5sum-) "480a47a04a0e0681cafbb5fbe734f2d4")
(setf (get ray_trace::Triangle :datatype-) "ray_trace/Triangle")
(setf (get ray_trace::Triangle :definition-)
      "geometry_msgs/Point p1
geometry_msgs/Point p2
geometry_msgs/Point p3
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :ray_trace/Triangle "480a47a04a0e0681cafbb5fbe734f2d4")


