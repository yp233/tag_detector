
(cl:in-package :asdf)

(defsystem "tag_detector-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ServiceMsg" :depends-on ("_package_ServiceMsg"))
    (:file "_package_ServiceMsg" :depends-on ("_package"))
  ))