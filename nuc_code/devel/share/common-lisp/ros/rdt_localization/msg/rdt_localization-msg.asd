
(cl:in-package :asdf)

(defsystem "rdt_localization-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Drive_Vector" :depends-on ("_package_Drive_Vector"))
    (:file "_package_Drive_Vector" :depends-on ("_package"))
    (:file "Location" :depends-on ("_package_Location"))
    (:file "_package_Location" :depends-on ("_package"))
    (:file "Orientation_Vector" :depends-on ("_package_Orientation_Vector"))
    (:file "_package_Orientation_Vector" :depends-on ("_package"))
    (:file "Pose" :depends-on ("_package_Pose"))
    (:file "_package_Pose" :depends-on ("_package"))
  ))