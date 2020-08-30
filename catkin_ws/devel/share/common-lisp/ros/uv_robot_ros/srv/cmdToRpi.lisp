; Auto-generated. Do not edit!


(cl:in-package uv_robot_ros-srv)


;//! \htmlinclude cmdToRpi-request.msg.html

(cl:defclass <cmdToRpi-request> (roslisp-msg-protocol:ros-message)
  ((cmdType
    :reader cmdType
    :initarg :cmdType
    :type cl:string
    :initform "")
   (dist_or_deg
    :reader dist_or_deg
    :initarg :dist_or_deg
    :type cl:string
    :initform ""))
)

(cl:defclass cmdToRpi-request (<cmdToRpi-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <cmdToRpi-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'cmdToRpi-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uv_robot_ros-srv:<cmdToRpi-request> is deprecated: use uv_robot_ros-srv:cmdToRpi-request instead.")))

(cl:ensure-generic-function 'cmdType-val :lambda-list '(m))
(cl:defmethod cmdType-val ((m <cmdToRpi-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uv_robot_ros-srv:cmdType-val is deprecated.  Use uv_robot_ros-srv:cmdType instead.")
  (cmdType m))

(cl:ensure-generic-function 'dist_or_deg-val :lambda-list '(m))
(cl:defmethod dist_or_deg-val ((m <cmdToRpi-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uv_robot_ros-srv:dist_or_deg-val is deprecated.  Use uv_robot_ros-srv:dist_or_deg instead.")
  (dist_or_deg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <cmdToRpi-request>) ostream)
  "Serializes a message object of type '<cmdToRpi-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'cmdType))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'cmdType))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'dist_or_deg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'dist_or_deg))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <cmdToRpi-request>) istream)
  "Deserializes a message object of type '<cmdToRpi-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cmdType) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'cmdType) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'dist_or_deg) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'dist_or_deg) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<cmdToRpi-request>)))
  "Returns string type for a service object of type '<cmdToRpi-request>"
  "uv_robot_ros/cmdToRpiRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cmdToRpi-request)))
  "Returns string type for a service object of type 'cmdToRpi-request"
  "uv_robot_ros/cmdToRpiRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<cmdToRpi-request>)))
  "Returns md5sum for a message object of type '<cmdToRpi-request>"
  "ab56ca80f4b06b7d4d8f78fb733348e9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'cmdToRpi-request)))
  "Returns md5sum for a message object of type 'cmdToRpi-request"
  "ab56ca80f4b06b7d4d8f78fb733348e9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<cmdToRpi-request>)))
  "Returns full string definition for message of type '<cmdToRpi-request>"
  (cl:format cl:nil "string cmdType~%string dist_or_deg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'cmdToRpi-request)))
  "Returns full string definition for message of type 'cmdToRpi-request"
  (cl:format cl:nil "string cmdType~%string dist_or_deg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <cmdToRpi-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'cmdType))
     4 (cl:length (cl:slot-value msg 'dist_or_deg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <cmdToRpi-request>))
  "Converts a ROS message object to a list"
  (cl:list 'cmdToRpi-request
    (cl:cons ':cmdType (cmdType msg))
    (cl:cons ':dist_or_deg (dist_or_deg msg))
))
;//! \htmlinclude cmdToRpi-response.msg.html

(cl:defclass <cmdToRpi-response> (roslisp-msg-protocol:ros-message)
  ((isComplete
    :reader isComplete
    :initarg :isComplete
    :type cl:boolean
    :initform cl:nil)
   (errorMsg
    :reader errorMsg
    :initarg :errorMsg
    :type cl:string
    :initform ""))
)

(cl:defclass cmdToRpi-response (<cmdToRpi-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <cmdToRpi-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'cmdToRpi-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uv_robot_ros-srv:<cmdToRpi-response> is deprecated: use uv_robot_ros-srv:cmdToRpi-response instead.")))

(cl:ensure-generic-function 'isComplete-val :lambda-list '(m))
(cl:defmethod isComplete-val ((m <cmdToRpi-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uv_robot_ros-srv:isComplete-val is deprecated.  Use uv_robot_ros-srv:isComplete instead.")
  (isComplete m))

(cl:ensure-generic-function 'errorMsg-val :lambda-list '(m))
(cl:defmethod errorMsg-val ((m <cmdToRpi-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uv_robot_ros-srv:errorMsg-val is deprecated.  Use uv_robot_ros-srv:errorMsg instead.")
  (errorMsg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <cmdToRpi-response>) ostream)
  "Serializes a message object of type '<cmdToRpi-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isComplete) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'errorMsg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'errorMsg))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <cmdToRpi-response>) istream)
  "Deserializes a message object of type '<cmdToRpi-response>"
    (cl:setf (cl:slot-value msg 'isComplete) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'errorMsg) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'errorMsg) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<cmdToRpi-response>)))
  "Returns string type for a service object of type '<cmdToRpi-response>"
  "uv_robot_ros/cmdToRpiResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cmdToRpi-response)))
  "Returns string type for a service object of type 'cmdToRpi-response"
  "uv_robot_ros/cmdToRpiResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<cmdToRpi-response>)))
  "Returns md5sum for a message object of type '<cmdToRpi-response>"
  "ab56ca80f4b06b7d4d8f78fb733348e9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'cmdToRpi-response)))
  "Returns md5sum for a message object of type 'cmdToRpi-response"
  "ab56ca80f4b06b7d4d8f78fb733348e9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<cmdToRpi-response>)))
  "Returns full string definition for message of type '<cmdToRpi-response>"
  (cl:format cl:nil "bool isComplete~%string errorMsg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'cmdToRpi-response)))
  "Returns full string definition for message of type 'cmdToRpi-response"
  (cl:format cl:nil "bool isComplete~%string errorMsg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <cmdToRpi-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'errorMsg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <cmdToRpi-response>))
  "Converts a ROS message object to a list"
  (cl:list 'cmdToRpi-response
    (cl:cons ':isComplete (isComplete msg))
    (cl:cons ':errorMsg (errorMsg msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'cmdToRpi)))
  'cmdToRpi-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'cmdToRpi)))
  'cmdToRpi-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cmdToRpi)))
  "Returns string type for a service object of type '<cmdToRpi>"
  "uv_robot_ros/cmdToRpi")