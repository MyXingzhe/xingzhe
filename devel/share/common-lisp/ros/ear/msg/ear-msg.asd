
(cl:in-package :asdf)

(defsystem "ear-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "usonic" :depends-on ("_package_usonic"))
    (:file "_package_usonic" :depends-on ("_package"))
  ))