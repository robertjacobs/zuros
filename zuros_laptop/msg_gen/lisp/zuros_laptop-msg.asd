
(cl:in-package :asdf)

(defsystem "zuros_laptop-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "MSG_LAPTOP_BATTERY" :depends-on ("_package_MSG_LAPTOP_BATTERY"))
    (:file "_package_MSG_LAPTOP_BATTERY" :depends-on ("_package"))
  ))