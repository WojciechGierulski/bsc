
(cl:in-package :asdf)

(defsystem "ray_trace-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "Triangle" :depends-on ("_package_Triangle"))
    (:file "_package_Triangle" :depends-on ("_package"))
  ))