; Auto-generated. Do not edit!


(cl:in-package tag_detector-srv)


;//! \htmlinclude ServiceMsg-request.msg.html

(cl:defclass <ServiceMsg-request> (roslisp-msg-protocol:ros-message)
  ((size1
    :reader size1
    :initarg :size1
    :type cl:integer
    :initform 0)
   (size2
    :reader size2
    :initarg :size2
    :type cl:integer
    :initform 0))
)

(cl:defclass ServiceMsg-request (<ServiceMsg-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ServiceMsg-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ServiceMsg-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tag_detector-srv:<ServiceMsg-request> is deprecated: use tag_detector-srv:ServiceMsg-request instead.")))

(cl:ensure-generic-function 'size1-val :lambda-list '(m))
(cl:defmethod size1-val ((m <ServiceMsg-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tag_detector-srv:size1-val is deprecated.  Use tag_detector-srv:size1 instead.")
  (size1 m))

(cl:ensure-generic-function 'size2-val :lambda-list '(m))
(cl:defmethod size2-val ((m <ServiceMsg-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tag_detector-srv:size2-val is deprecated.  Use tag_detector-srv:size2 instead.")
  (size2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ServiceMsg-request>) ostream)
  "Serializes a message object of type '<ServiceMsg-request>"
  (cl:let* ((signed (cl:slot-value msg 'size1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'size2)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ServiceMsg-request>) istream)
  "Deserializes a message object of type '<ServiceMsg-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'size1) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'size2) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ServiceMsg-request>)))
  "Returns string type for a service object of type '<ServiceMsg-request>"
  "tag_detector/ServiceMsgRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ServiceMsg-request)))
  "Returns string type for a service object of type 'ServiceMsg-request"
  "tag_detector/ServiceMsgRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ServiceMsg-request>)))
  "Returns md5sum for a message object of type '<ServiceMsg-request>"
  "b8e47ef395f4ff61960de951a76c82dd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ServiceMsg-request)))
  "Returns md5sum for a message object of type 'ServiceMsg-request"
  "b8e47ef395f4ff61960de951a76c82dd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ServiceMsg-request>)))
  "Returns full string definition for message of type '<ServiceMsg-request>"
  (cl:format cl:nil "int32 size1~%int32 size2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ServiceMsg-request)))
  "Returns full string definition for message of type 'ServiceMsg-request"
  (cl:format cl:nil "int32 size1~%int32 size2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ServiceMsg-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ServiceMsg-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ServiceMsg-request
    (cl:cons ':size1 (size1 msg))
    (cl:cons ':size2 (size2 msg))
))
;//! \htmlinclude ServiceMsg-response.msg.html

(cl:defclass <ServiceMsg-response> (roslisp-msg-protocol:ros-message)
  ((isSetOK
    :reader isSetOK
    :initarg :isSetOK
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ServiceMsg-response (<ServiceMsg-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ServiceMsg-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ServiceMsg-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tag_detector-srv:<ServiceMsg-response> is deprecated: use tag_detector-srv:ServiceMsg-response instead.")))

(cl:ensure-generic-function 'isSetOK-val :lambda-list '(m))
(cl:defmethod isSetOK-val ((m <ServiceMsg-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tag_detector-srv:isSetOK-val is deprecated.  Use tag_detector-srv:isSetOK instead.")
  (isSetOK m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ServiceMsg-response>) ostream)
  "Serializes a message object of type '<ServiceMsg-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isSetOK) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ServiceMsg-response>) istream)
  "Deserializes a message object of type '<ServiceMsg-response>"
    (cl:setf (cl:slot-value msg 'isSetOK) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ServiceMsg-response>)))
  "Returns string type for a service object of type '<ServiceMsg-response>"
  "tag_detector/ServiceMsgResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ServiceMsg-response)))
  "Returns string type for a service object of type 'ServiceMsg-response"
  "tag_detector/ServiceMsgResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ServiceMsg-response>)))
  "Returns md5sum for a message object of type '<ServiceMsg-response>"
  "b8e47ef395f4ff61960de951a76c82dd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ServiceMsg-response)))
  "Returns md5sum for a message object of type 'ServiceMsg-response"
  "b8e47ef395f4ff61960de951a76c82dd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ServiceMsg-response>)))
  "Returns full string definition for message of type '<ServiceMsg-response>"
  (cl:format cl:nil "bool isSetOK~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ServiceMsg-response)))
  "Returns full string definition for message of type 'ServiceMsg-response"
  (cl:format cl:nil "bool isSetOK~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ServiceMsg-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ServiceMsg-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ServiceMsg-response
    (cl:cons ':isSetOK (isSetOK msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ServiceMsg)))
  'ServiceMsg-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ServiceMsg)))
  'ServiceMsg-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ServiceMsg)))
  "Returns string type for a service object of type '<ServiceMsg>"
  "tag_detector/ServiceMsg")