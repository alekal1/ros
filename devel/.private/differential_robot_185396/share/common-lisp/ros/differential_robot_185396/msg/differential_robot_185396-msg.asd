
(cl:in-package :asdf)

(defsystem "differential_robot_185396-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "counter_message" :depends-on ("_package_counter_message"))
    (:file "_package_counter_message" :depends-on ("_package"))
  ))