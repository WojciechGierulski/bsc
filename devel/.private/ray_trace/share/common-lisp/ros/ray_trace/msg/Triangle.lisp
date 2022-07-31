; Auto-generated. Do not edit!


(cl:in-package ray_trace-msg)


;//! \htmlinclude Triangle.msg.html

(cl:defclass <Triangle> (roslisp-msg-protocol:ros-message)
  ((p1
    :reader p1
    :initarg :p1
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (p2
    :reader p2
    :initarg :p2
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (p3
    :reader p3
    :initarg :p3
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass Triangle (<Triangle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Triangle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Triangle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ray_trace-msg:<Triangle> is deprecated: use ray_trace-msg:Triangle instead.")))

(cl:ensure-generic-function 'p1-val :lambda-list '(m))
(cl:defmethod p1-val ((m <Triangle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ray_trace-msg:p1-val is deprecated.  Use ray_trace-msg:p1 instead.")
  (p1 m))

(cl:ensure-generic-function 'p2-val :lambda-list '(m))
(cl:defmethod p2-val ((m <Triangle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ray_trace-msg:p2-val is deprecated.  Use ray_trace-msg:p2 instead.")
  (p2 m))

(cl:ensure-generic-function 'p3-val :lambda-list '(m))
(cl:defmethod p3-val ((m <Triangle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ray_trace-msg:p3-val is deprecated.  Use ray_trace-msg:p3 instead.")
  (p3 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Triangle>) ostream)
  "Serializes a message object of type '<Triangle>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'p1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'p2) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'p3) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Triangle>) istream)
  "Deserializes a message object of type '<Triangle>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'p1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'p2) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'p3) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Triangle>)))
  "Returns string type for a message object of type '<Triangle>"
  "ray_trace/Triangle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Triangle)))
  "Returns string type for a message object of type 'Triangle"
  "ray_trace/Triangle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Triangle>)))
  "Returns md5sum for a message object of type '<Triangle>"
  "480a47a04a0e0681cafbb5fbe734f2d4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Triangle)))
  "Returns md5sum for a message object of type 'Triangle"
  "480a47a04a0e0681cafbb5fbe734f2d4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Triangle>)))
  "Returns full string definition for message of type '<Triangle>"
  (cl:format cl:nil "geometry_msgs/Point p1~%geometry_msgs/Point p2~%geometry_msgs/Point p3~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Triangle)))
  "Returns full string definition for message of type 'Triangle"
  (cl:format cl:nil "geometry_msgs/Point p1~%geometry_msgs/Point p2~%geometry_msgs/Point p3~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Triangle>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'p1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'p2))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'p3))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Triangle>))
  "Converts a ROS message object to a list"
  (cl:list 'Triangle
    (cl:cons ':p1 (p1 msg))
    (cl:cons ':p2 (p2 msg))
    (cl:cons ':p3 (p3 msg))
))
