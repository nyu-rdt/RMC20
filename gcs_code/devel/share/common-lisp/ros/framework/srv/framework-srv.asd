
(cl:in-package :asdf)

(defsystem "framework-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "IntStr" :depends-on ("_package_IntStr"))
    (:file "_package_IntStr" :depends-on ("_package"))
    (:file "StrInt" :depends-on ("_package_StrInt"))
    (:file "_package_StrInt" :depends-on ("_package"))
  ))