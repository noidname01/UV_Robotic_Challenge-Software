
(cl:in-package :asdf)

(defsystem "uv_robot_ros-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "cmdToRpi" :depends-on ("_package_cmdToRpi"))
    (:file "_package_cmdToRpi" :depends-on ("_package"))
  ))