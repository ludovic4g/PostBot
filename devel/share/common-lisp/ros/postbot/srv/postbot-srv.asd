
(cl:in-package :asdf)

(defsystem "postbot-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "reset_boxes" :depends-on ("_package_reset_boxes"))
    (:file "_package_reset_boxes" :depends-on ("_package"))
    (:file "spawn_ball" :depends-on ("_package_spawn_ball"))
    (:file "_package_spawn_ball" :depends-on ("_package"))
  ))