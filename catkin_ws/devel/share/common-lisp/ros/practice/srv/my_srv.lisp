; Auto-generated. Do not edit!


(cl:in-package practice-srv)


;//! \htmlinclude my_srv-request.msg.html

(cl:defclass <my_srv-request> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0))
)

(cl:defclass my_srv-request (<my_srv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <my_srv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'my_srv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name practice-srv:<my_srv-request> is deprecated: use practice-srv:my_srv-request instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <my_srv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader practice-srv:id-val is deprecated.  Use practice-srv:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <my_srv-request>) ostream)
  "Serializes a message object of type '<my_srv-request>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <my_srv-request>) istream)
  "Deserializes a message object of type '<my_srv-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<my_srv-request>)))
  "Returns string type for a service object of type '<my_srv-request>"
  "practice/my_srvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'my_srv-request)))
  "Returns string type for a service object of type 'my_srv-request"
  "practice/my_srvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<my_srv-request>)))
  "Returns md5sum for a message object of type '<my_srv-request>"
  "b8a2aab5099fa54e0eb1247d552c20ed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'my_srv-request)))
  "Returns md5sum for a message object of type 'my_srv-request"
  "b8a2aab5099fa54e0eb1247d552c20ed")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<my_srv-request>)))
  "Returns full string definition for message of type '<my_srv-request>"
  (cl:format cl:nil "int64 id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'my_srv-request)))
  "Returns full string definition for message of type 'my_srv-request"
  (cl:format cl:nil "int64 id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <my_srv-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <my_srv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'my_srv-request
    (cl:cons ':id (id msg))
))
;//! \htmlinclude my_srv-response.msg.html

(cl:defclass <my_srv-response> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (gender
    :reader gender
    :initarg :gender
    :type cl:string
    :initform "")
   (age
    :reader age
    :initarg :age
    :type cl:integer
    :initform 0))
)

(cl:defclass my_srv-response (<my_srv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <my_srv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'my_srv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name practice-srv:<my_srv-response> is deprecated: use practice-srv:my_srv-response instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <my_srv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader practice-srv:name-val is deprecated.  Use practice-srv:name instead.")
  (name m))

(cl:ensure-generic-function 'gender-val :lambda-list '(m))
(cl:defmethod gender-val ((m <my_srv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader practice-srv:gender-val is deprecated.  Use practice-srv:gender instead.")
  (gender m))

(cl:ensure-generic-function 'age-val :lambda-list '(m))
(cl:defmethod age-val ((m <my_srv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader practice-srv:age-val is deprecated.  Use practice-srv:age instead.")
  (age m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <my_srv-response>) ostream)
  "Serializes a message object of type '<my_srv-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'gender))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'gender))
  (cl:let* ((signed (cl:slot-value msg 'age)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <my_srv-response>) istream)
  "Deserializes a message object of type '<my_srv-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'gender) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'gender) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'age) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<my_srv-response>)))
  "Returns string type for a service object of type '<my_srv-response>"
  "practice/my_srvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'my_srv-response)))
  "Returns string type for a service object of type 'my_srv-response"
  "practice/my_srvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<my_srv-response>)))
  "Returns md5sum for a message object of type '<my_srv-response>"
  "b8a2aab5099fa54e0eb1247d552c20ed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'my_srv-response)))
  "Returns md5sum for a message object of type 'my_srv-response"
  "b8a2aab5099fa54e0eb1247d552c20ed")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<my_srv-response>)))
  "Returns full string definition for message of type '<my_srv-response>"
  (cl:format cl:nil "string name~%string gender~%int64 age~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'my_srv-response)))
  "Returns full string definition for message of type 'my_srv-response"
  (cl:format cl:nil "string name~%string gender~%int64 age~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <my_srv-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4 (cl:length (cl:slot-value msg 'gender))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <my_srv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'my_srv-response
    (cl:cons ':name (name msg))
    (cl:cons ':gender (gender msg))
    (cl:cons ':age (age msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'my_srv)))
  'my_srv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'my_srv)))
  'my_srv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'my_srv)))
  "Returns string type for a service object of type '<my_srv>"
  "practice/my_srv")