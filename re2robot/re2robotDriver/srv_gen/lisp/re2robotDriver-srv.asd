
(cl:in-package :asdf)

(defsystem "re2robotDriver-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :re2robotDriver-msg
)
  :components ((:file "_package")
    (:file "AddDrive" :depends-on ("_package_AddDrive"))
    (:file "_package_AddDrive" :depends-on ("_package"))
    (:file "RemoveDrive" :depends-on ("_package_RemoveDrive"))
    (:file "_package_RemoveDrive" :depends-on ("_package"))
  ))