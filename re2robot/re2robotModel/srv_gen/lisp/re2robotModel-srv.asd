
(cl:in-package :asdf)

(defsystem "re2robotModel-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :re2robotModel-msg
)
  :components ((:file "_package")
    (:file "RemoveArbiter" :depends-on ("_package_RemoveArbiter"))
    (:file "_package_RemoveArbiter" :depends-on ("_package"))
    (:file "AddTransmission" :depends-on ("_package_AddTransmission"))
    (:file "_package_AddTransmission" :depends-on ("_package"))
    (:file "RemoveTransmission" :depends-on ("_package_RemoveTransmission"))
    (:file "_package_RemoveTransmission" :depends-on ("_package"))
    (:file "AddArbiter" :depends-on ("_package_AddArbiter"))
    (:file "_package_AddArbiter" :depends-on ("_package"))
  ))