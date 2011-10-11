
(cl:in-package :asdf)

(defsystem "re2robotModel-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ModelConfig" :depends-on ("_package_ModelConfig"))
    (:file "_package_ModelConfig" :depends-on ("_package"))
    (:file "JointState" :depends-on ("_package_JointState"))
    (:file "_package_JointState" :depends-on ("_package"))
    (:file "TransmissionConfig" :depends-on ("_package_TransmissionConfig"))
    (:file "_package_TransmissionConfig" :depends-on ("_package"))
    (:file "ArbiterConfig" :depends-on ("_package_ArbiterConfig"))
    (:file "_package_ArbiterConfig" :depends-on ("_package"))
    (:file "JointCmd" :depends-on ("_package_JointCmd"))
    (:file "_package_JointCmd" :depends-on ("_package"))
    (:file "ControlArbiterState" :depends-on ("_package_ControlArbiterState"))
    (:file "_package_ControlArbiterState" :depends-on ("_package"))
    (:file "ControlArbiterCmd" :depends-on ("_package_ControlArbiterCmd"))
    (:file "_package_ControlArbiterCmd" :depends-on ("_package"))
  ))