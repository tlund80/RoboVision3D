
(cl:in-package :asdf)

(defsystem "vrm3dvision-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SurfaceModelParams" :depends-on ("_package_SurfaceModelParams"))
    (:file "_package_SurfaceModelParams" :depends-on ("_package"))
    (:file "AlignmentPrerejectiveParams" :depends-on ("_package_AlignmentPrerejectiveParams"))
    (:file "_package_AlignmentPrerejectiveParams" :depends-on ("_package"))
  ))