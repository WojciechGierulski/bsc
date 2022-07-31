;; Auto-generated. Do not edit!


(when (boundp 'ray_trace::ray_trace)
  (if (not (find-package "RAY_TRACE"))
    (make-package "RAY_TRACE"))
  (shadow 'ray_trace (find-package "RAY_TRACE")))
(unless (find-package "RAY_TRACE::RAY_TRACE")
  (make-package "RAY_TRACE::RAY_TRACE"))
(unless (find-package "RAY_TRACE::RAY_TRACEREQUEST")
  (make-package "RAY_TRACE::RAY_TRACEREQUEST"))
(unless (find-package "RAY_TRACE::RAY_TRACERESPONSE")
  (make-package "RAY_TRACE::RAY_TRACERESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass ray_trace::ray_traceRequest
  :super ros::object
  :slots (_vertices _faces _rays _origins ))

(defmethod ray_trace::ray_traceRequest
  (:init
   (&key
    ((:vertices __vertices) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Point :init) r)) r))
    ((:faces __faces) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Point :init) r)) r))
    ((:rays __rays) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Point :init) r)) r))
    ((:origins __origins) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Point :init) r)) r))
    )
   (send-super :init)
   (setq _vertices __vertices)
   (setq _faces __faces)
   (setq _rays __rays)
   (setq _origins __origins)
   self)
  (:vertices
   (&rest __vertices)
   (if (keywordp (car __vertices))
       (send* _vertices __vertices)
     (progn
       (if __vertices (setq _vertices (car __vertices)))
       _vertices)))
  (:faces
   (&rest __faces)
   (if (keywordp (car __faces))
       (send* _faces __faces)
     (progn
       (if __faces (setq _faces (car __faces)))
       _faces)))
  (:rays
   (&rest __rays)
   (if (keywordp (car __rays))
       (send* _rays __rays)
     (progn
       (if __rays (setq _rays (car __rays)))
       _rays)))
  (:origins
   (&rest __origins)
   (if (keywordp (car __origins))
       (send* _origins __origins)
     (progn
       (if __origins (setq _origins (car __origins)))
       _origins)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point[] _vertices
    (apply #'+ (send-all _vertices :serialization-length)) 4
    ;; geometry_msgs/Point[] _faces
    (apply #'+ (send-all _faces :serialization-length)) 4
    ;; geometry_msgs/Point[] _rays
    (apply #'+ (send-all _rays :serialization-length)) 4
    ;; geometry_msgs/Point[] _origins
    (apply #'+ (send-all _origins :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point[] _vertices
     (write-long (length _vertices) s)
     (dolist (elem _vertices)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Point[] _faces
     (write-long (length _faces) s)
     (dolist (elem _faces)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Point[] _rays
     (write-long (length _rays) s)
     (dolist (elem _rays)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Point[] _origins
     (write-long (length _origins) s)
     (dolist (elem _origins)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point[] _vertices
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _vertices (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _vertices)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Point[] _faces
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _faces (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _faces)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Point[] _rays
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _rays (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _rays)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Point[] _origins
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _origins (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _origins)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass ray_trace::ray_traceResponse
  :super ros::object
  :slots (_intersections ))

(defmethod ray_trace::ray_traceResponse
  (:init
   (&key
    ((:intersections __intersections) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Point :init) r)) r))
    )
   (send-super :init)
   (setq _intersections __intersections)
   self)
  (:intersections
   (&rest __intersections)
   (if (keywordp (car __intersections))
       (send* _intersections __intersections)
     (progn
       (if __intersections (setq _intersections (car __intersections)))
       _intersections)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point[] _intersections
    (apply #'+ (send-all _intersections :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point[] _intersections
     (write-long (length _intersections) s)
     (dolist (elem _intersections)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point[] _intersections
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _intersections (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _intersections)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass ray_trace::ray_trace
  :super ros::object
  :slots ())

(setf (get ray_trace::ray_trace :md5sum-) "81c2826f4b1acbbe34aaa616c1b2d0b8")
(setf (get ray_trace::ray_trace :datatype-) "ray_trace/ray_trace")
(setf (get ray_trace::ray_trace :request) ray_trace::ray_traceRequest)
(setf (get ray_trace::ray_trace :response) ray_trace::ray_traceResponse)

(defmethod ray_trace::ray_traceRequest
  (:response () (instance ray_trace::ray_traceResponse :init)))

(setf (get ray_trace::ray_traceRequest :md5sum-) "81c2826f4b1acbbe34aaa616c1b2d0b8")
(setf (get ray_trace::ray_traceRequest :datatype-) "ray_trace/ray_traceRequest")
(setf (get ray_trace::ray_traceRequest :definition-)
      "geometry_msgs/Point[] vertices
geometry_msgs/Point[] faces
geometry_msgs/Point[] rays
geometry_msgs/Point[] origins

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
geometry_msgs/Point[] intersections

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
")

(setf (get ray_trace::ray_traceResponse :md5sum-) "81c2826f4b1acbbe34aaa616c1b2d0b8")
(setf (get ray_trace::ray_traceResponse :datatype-) "ray_trace/ray_traceResponse")
(setf (get ray_trace::ray_traceResponse :definition-)
      "geometry_msgs/Point[] vertices
geometry_msgs/Point[] faces
geometry_msgs/Point[] rays
geometry_msgs/Point[] origins

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
geometry_msgs/Point[] intersections

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
")



(provide :ray_trace/ray_trace "81c2826f4b1acbbe34aaa616c1b2d0b8")


