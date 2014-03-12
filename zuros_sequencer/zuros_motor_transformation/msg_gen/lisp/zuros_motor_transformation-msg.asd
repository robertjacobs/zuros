
(cl:in-package :asdf)

(defsystem "zuros_motor_transformation-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "differential" :depends-on ("_package_differential"))
    (:file "_package_differential" :depends-on ("_package"))
  ))