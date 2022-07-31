; Auto-generated. Do not edit!


(cl:in-package ray_trace-srv)


;//! \htmlinclude ray_trace-request.msg.html

(cl:defclass <ray_trace-request> (roslisp-msg-protocol:ros-message)
  ((vertices
    :reader vertices
    :initarg :vertices
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (faces
    :reader faces
    :initarg :faces
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (rays
    :reader rays
    :initarg :rays
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (origins
    :reader origins
    :initarg :origins
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point))))
)

(cl:defclass ray_trace-request (<ray_trace-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ray_trace-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ray_trace-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ray_trace-srv:<ray_trace-request> is deprecated: use ray_trace-srv:ray_trace-request instead.")))

(cl:ensure-generic-function 'vertices-val :lambda-list '(m))
(cl:defmethod vertices-val ((m <ray_trace-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ray_trace-srv:vertices-val is deprecated.  Use ray_trace-srv:vertices instead.")
  (vertices m))

(cl:ensure-generic-function 'faces-val :lambda-list '(m))
(cl:defmethod faces-val ((m <ray_trace-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ray_trace-srv:faces-val is deprecated.  Use ray_trace-srv:faces instead.")
  (faces m))

(cl:ensure-generic-function 'rays-val :lambda-list '(m))
(cl:defmethod rays-val ((m <ray_trace-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ray_trace-srv:rays-val is deprecated.  Use ray_trace-srv:rays instead.")
  (rays m))

(cl:ensure-generic-function 'origins-val :lambda-list '(m))
(cl:defmethod origins-val ((m <ray_trace-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ray_trace-srv:origins-val is deprecated.  Use ray_trace-srv:origins instead.")
  (origins m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ray_trace-request>) ostream)
  "Serializes a message object of type '<ray_trace-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'vertices))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'vertices))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'faces))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'faces))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'rays))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'rays))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'origins))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'origins))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ray_trace-request>) istream)
  "Deserializes a message object of type '<ray_trace-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'vertices) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'vertices)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'faces) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'faces)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'rays) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'rays)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'origins) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'origins)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ray_trace-request>)))
  "Returns string type for a service object of type '<ray_trace-request>"
  "ray_trace/ray_traceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ray_trace-request)))
  "Returns string type for a service object of type 'ray_trace-request"
  "ray_trace/ray_traceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ray_trace-request>)))
  "Returns md5sum for a message object of type '<ray_trace-request>"
  "81c2826f4b1acbbe34aaa616c1b2d0b8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ray_trace-request)))
  "Returns md5sum for a message object of type 'ray_trace-request"
  "81c2826f4b1acbbe34aaa616c1b2d0b8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ray_trace-request>)))
  "Returns full string definition for message of type '<ray_trace-request>"
  (cl:format cl:nil "geometry_msgs/Point[] vertices~%geometry_msgs/Point[] faces~%geometry_msgs/Point[] rays~%geometry_msgs/Point[] origins~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ray_trace-request)))
  "Returns full string definition for message of type 'ray_trace-request"
  (cl:format cl:nil "geometry_msgs/Point[] vertices~%geometry_msgs/Point[] faces~%geometry_msgs/Point[] rays~%geometry_msgs/Point[] origins~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ray_trace-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'vertices) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'faces) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'rays) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'origins) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ray_trace-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ray_trace-request
    (cl:cons ':vertices (vertices msg))
    (cl:cons ':faces (faces msg))
    (cl:cons ':rays (rays msg))
    (cl:cons ':origins (origins msg))
))
;//! \htmlinclude ray_trace-response.msg.html

(cl:defclass <ray_trace-response> (roslisp-msg-protocol:ros-message)
  ((intersections
    :reader intersections
    :initarg :intersections
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point))))
)

(cl:defclass ray_trace-response (<ray_trace-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ray_trace-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ray_trace-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ray_trace-srv:<ray_trace-response> is deprecated: use ray_trace-srv:ray_trace-response instead.")))

(cl:ensure-generic-function 'intersections-val :lambda-list '(m))
(cl:defmethod intersections-val ((m <ray_trace-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ray_trace-srv:intersections-val is deprecated.  Use ray_trace-srv:intersections instead.")
  (intersections m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ray_trace-response>) ostream)
  "Serializes a message object of type '<ray_trace-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'intersections))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'intersections))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ray_trace-response>) istream)
  "Deserializes a message object of type '<ray_trace-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'intersections) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'intersections)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ray_trace-response>)))
  "Returns string type for a service object of type '<ray_trace-response>"
  "ray_trace/ray_traceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ray_trace-response)))
  "Returns string type for a service object of type 'ray_trace-response"
  "ray_trace/ray_traceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ray_trace-response>)))
  "Returns md5sum for a message object of type '<ray_trace-response>"
  "81c2826f4b1acbbe34aaa616c1b2d0b8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ray_trace-response)))
  "Returns md5sum for a message object of type 'ray_trace-response"
  "81c2826f4b1acbbe34aaa616c1b2d0b8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ray_trace-response>)))
  "Returns full string definition for message of type '<ray_trace-response>"
  (cl:format cl:nil "geometry_msgs/Point[] intersections~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ray_trace-response)))
  "Returns full string definition for message of type 'ray_trace-response"
  (cl:format cl:nil "geometry_msgs/Point[] intersections~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ray_trace-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'intersections) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ray_trace-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ray_trace-response
    (cl:cons ':intersections (intersections msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ray_trace)))
  'ray_trace-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ray_trace)))
  'ray_trace-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ray_trace)))
  "Returns string type for a service object of type '<ray_trace>"
  "ray_trace/ray_trace")