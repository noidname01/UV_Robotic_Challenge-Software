; Auto-generated. Do not edit!


(cl:in-package practice-msg)


;//! \htmlinclude my_msg.msg.html

(cl:defclass <my_msg> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (title
    :reader title
    :initarg :title
    :type cl:string
    :initform "")
   (content
    :reader content
    :initarg :content
    :type cl:string
    :initform ""))
)

(cl:defclass my_msg (<my_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <my_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'my_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name practice-msg:<my_msg> is deprecated: use practice-msg:my_msg instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <my_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader practice-msg:id-val is deprecated.  Use practice-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'title-val :lambda-list '(m))
(cl:defmethod title-val ((m <my_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader practice-msg:title-val is deprecated.  Use practice-msg:title instead.")
  (title m))

(cl:ensure-generic-function 'content-val :lambda-list '(m))
(cl:defmethod content-val ((m <my_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader practice-msg:content-val is deprecated.  Use practice-msg:content instead.")
  (content m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <my_msg>) ostream)
  "Serializes a message object of type '<my_msg>"
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
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'title))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'title))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'content))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'content))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <my_msg>) istream)
  "Deserializes a message object of type '<my_msg>"
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
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'title) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'title) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'content) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'content) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<my_msg>)))
  "Returns string type for a message object of type '<my_msg>"
  "practice/my_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'my_msg)))
  "Returns string type for a message object of type 'my_msg"
  "practice/my_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<my_msg>)))
  "Returns md5sum for a message object of type '<my_msg>"
  "1e2cb8cf2c48e86d2e59515dd5faeb6a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'my_msg)))
  "Returns md5sum for a message object of type 'my_msg"
  "1e2cb8cf2c48e86d2e59515dd5faeb6a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<my_msg>)))
  "Returns full string definition for message of type '<my_msg>"
  (cl:format cl:nil "int64 id~%string title~%string content~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'my_msg)))
  "Returns full string definition for message of type 'my_msg"
  (cl:format cl:nil "int64 id~%string title~%string content~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <my_msg>))
  (cl:+ 0
     8
     4 (cl:length (cl:slot-value msg 'title))
     4 (cl:length (cl:slot-value msg 'content))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <my_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'my_msg
    (cl:cons ':id (id msg))
    (cl:cons ':title (title msg))
    (cl:cons ':content (content msg))
))
