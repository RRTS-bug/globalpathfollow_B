
(cl:in-package :asdf)

(defsystem "pathfollow-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Traj" :depends-on ("_package_Traj"))
    (:file "_package_Traj" :depends-on ("_package"))
  ))