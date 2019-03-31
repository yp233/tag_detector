;; Auto-generated. Do not edit!


(when (boundp 'tag_detector::ServiceMsg)
  (if (not (find-package "TAG_DETECTOR"))
    (make-package "TAG_DETECTOR"))
  (shadow 'ServiceMsg (find-package "TAG_DETECTOR")))
(unless (find-package "TAG_DETECTOR::SERVICEMSG")
  (make-package "TAG_DETECTOR::SERVICEMSG"))
(unless (find-package "TAG_DETECTOR::SERVICEMSGREQUEST")
  (make-package "TAG_DETECTOR::SERVICEMSGREQUEST"))
(unless (find-package "TAG_DETECTOR::SERVICEMSGRESPONSE")
  (make-package "TAG_DETECTOR::SERVICEMSGRESPONSE"))

(in-package "ROS")





(defclass tag_detector::ServiceMsgRequest
  :super ros::object
  :slots (_size1 _size2 ))

(defmethod tag_detector::ServiceMsgRequest
  (:init
   (&key
    ((:size1 __size1) 0)
    ((:size2 __size2) 0)
    )
   (send-super :init)
   (setq _size1 (round __size1))
   (setq _size2 (round __size2))
   self)
  (:size1
   (&optional __size1)
   (if __size1 (setq _size1 __size1)) _size1)
  (:size2
   (&optional __size2)
   (if __size2 (setq _size2 __size2)) _size2)
  (:serialization-length
   ()
   (+
    ;; int32 _size1
    4
    ;; int32 _size2
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _size1
       (write-long _size1 s)
     ;; int32 _size2
       (write-long _size2 s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _size1
     (setq _size1 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _size2
     (setq _size2 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass tag_detector::ServiceMsgResponse
  :super ros::object
  :slots (_isSetOK ))

(defmethod tag_detector::ServiceMsgResponse
  (:init
   (&key
    ((:isSetOK __isSetOK) nil)
    )
   (send-super :init)
   (setq _isSetOK __isSetOK)
   self)
  (:isSetOK
   (&optional __isSetOK)
   (if __isSetOK (setq _isSetOK __isSetOK)) _isSetOK)
  (:serialization-length
   ()
   (+
    ;; bool _isSetOK
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _isSetOK
       (if _isSetOK (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _isSetOK
     (setq _isSetOK (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass tag_detector::ServiceMsg
  :super ros::object
  :slots ())

(setf (get tag_detector::ServiceMsg :md5sum-) "b8e47ef395f4ff61960de951a76c82dd")
(setf (get tag_detector::ServiceMsg :datatype-) "tag_detector/ServiceMsg")
(setf (get tag_detector::ServiceMsg :request) tag_detector::ServiceMsgRequest)
(setf (get tag_detector::ServiceMsg :response) tag_detector::ServiceMsgResponse)

(defmethod tag_detector::ServiceMsgRequest
  (:response () (instance tag_detector::ServiceMsgResponse :init)))

(setf (get tag_detector::ServiceMsgRequest :md5sum-) "b8e47ef395f4ff61960de951a76c82dd")
(setf (get tag_detector::ServiceMsgRequest :datatype-) "tag_detector/ServiceMsgRequest")
(setf (get tag_detector::ServiceMsgRequest :definition-)
      "int32 size1
int32 size2
---
bool isSetOK

")

(setf (get tag_detector::ServiceMsgResponse :md5sum-) "b8e47ef395f4ff61960de951a76c82dd")
(setf (get tag_detector::ServiceMsgResponse :datatype-) "tag_detector/ServiceMsgResponse")
(setf (get tag_detector::ServiceMsgResponse :definition-)
      "int32 size1
int32 size2
---
bool isSetOK

")



(provide :tag_detector/ServiceMsg "b8e47ef395f4ff61960de951a76c82dd")


