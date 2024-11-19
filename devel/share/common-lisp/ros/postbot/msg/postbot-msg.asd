
(cl:in-package :asdf)

(defsystem "postbot-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "BallInfo" :depends-on ("_package_BallInfo"))
    (:file "_package_BallInfo" :depends-on ("_package"))
    (:file "BoxGoal" :depends-on ("_package_BoxGoal"))
    (:file "_package_BoxGoal" :depends-on ("_package"))
    (:file "BoxInfo" :depends-on ("_package_BoxInfo"))
    (:file "_package_BoxInfo" :depends-on ("_package"))
  ))