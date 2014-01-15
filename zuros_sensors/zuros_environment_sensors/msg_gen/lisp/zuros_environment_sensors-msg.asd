
(cl:in-package :asdf)

(defsystem "zuros_environment_sensors-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "MSG_ZWAVE_STATUS" :depends-on ("_package_MSG_ZWAVE_STATUS"))
    (:file "_package_MSG_ZWAVE_STATUS" :depends-on ("_package"))
    (:file "MSG_ZWAVE_SENSORS" :depends-on ("_package_MSG_ZWAVE_SENSORS"))
    (:file "_package_MSG_ZWAVE_SENSORS" :depends-on ("_package"))
  ))