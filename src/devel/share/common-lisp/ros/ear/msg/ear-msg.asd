
(cl:in-package :asdf)

(defsystem "ear-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "sensor_imu" :depends-on ("_package_sensor_imu"))
    (:file "_package_sensor_imu" :depends-on ("_package"))
    (:file "usonic" :depends-on ("_package_usonic"))
    (:file "_package_usonic" :depends-on ("_package"))
  ))