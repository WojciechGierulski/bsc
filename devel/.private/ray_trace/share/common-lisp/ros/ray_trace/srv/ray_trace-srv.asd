
(cl:in-package :asdf)

(defsystem "ray_trace-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "ray_trace" :depends-on ("_package_ray_trace"))
    (:file "_package_ray_trace" :depends-on ("_package"))
  ))