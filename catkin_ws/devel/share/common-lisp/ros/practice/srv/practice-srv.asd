
(cl:in-package :asdf)

(defsystem "practice-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "my_srv" :depends-on ("_package_my_srv"))
    (:file "_package_my_srv" :depends-on ("_package"))
  ))