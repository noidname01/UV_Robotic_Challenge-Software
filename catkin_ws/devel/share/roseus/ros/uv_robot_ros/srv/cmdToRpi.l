;; Auto-generated. Do not edit!


(when (boundp 'uv_robot_ros::cmdToRpi)
  (if (not (find-package "UV_ROBOT_ROS"))
    (make-package "UV_ROBOT_ROS"))
  (shadow 'cmdToRpi (find-package "UV_ROBOT_ROS")))
(unless (find-package "UV_ROBOT_ROS::CMDTORPI")
  (make-package "UV_ROBOT_ROS::CMDTORPI"))
(unless (find-package "UV_ROBOT_ROS::CMDTORPIREQUEST")
  (make-package "UV_ROBOT_ROS::CMDTORPIREQUEST"))
(unless (find-package "UV_ROBOT_ROS::CMDTORPIRESPONSE")
  (make-package "UV_ROBOT_ROS::CMDTORPIRESPONSE"))

(in-package "ROS")





(defclass uv_robot_ros::cmdToRpiRequest
  :super ros::object
  :slots (_cmdType _dist_or_deg ))

(defmethod uv_robot_ros::cmdToRpiRequest
  (:init
   (&key
    ((:cmdType __cmdType) "")
    ((:dist_or_deg __dist_or_deg) "")
    )
   (send-super :init)
   (setq _cmdType (string __cmdType))
   (setq _dist_or_deg (string __dist_or_deg))
   self)
  (:cmdType
   (&optional __cmdType)
   (if __cmdType (setq _cmdType __cmdType)) _cmdType)
  (:dist_or_deg
   (&optional __dist_or_deg)
   (if __dist_or_deg (setq _dist_or_deg __dist_or_deg)) _dist_or_deg)
  (:serialization-length
   ()
   (+
    ;; string _cmdType
    4 (length _cmdType)
    ;; string _dist_or_deg
    4 (length _dist_or_deg)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _cmdType
       (write-long (length _cmdType) s) (princ _cmdType s)
     ;; string _dist_or_deg
       (write-long (length _dist_or_deg) s) (princ _dist_or_deg s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _cmdType
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _cmdType (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _dist_or_deg
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _dist_or_deg (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass uv_robot_ros::cmdToRpiResponse
  :super ros::object
  :slots (_isComplete _errorMsg ))

(defmethod uv_robot_ros::cmdToRpiResponse
  (:init
   (&key
    ((:isComplete __isComplete) nil)
    ((:errorMsg __errorMsg) "")
    )
   (send-super :init)
   (setq _isComplete __isComplete)
   (setq _errorMsg (string __errorMsg))
   self)
  (:isComplete
   (&optional __isComplete)
   (if __isComplete (setq _isComplete __isComplete)) _isComplete)
  (:errorMsg
   (&optional __errorMsg)
   (if __errorMsg (setq _errorMsg __errorMsg)) _errorMsg)
  (:serialization-length
   ()
   (+
    ;; bool _isComplete
    1
    ;; string _errorMsg
    4 (length _errorMsg)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _isComplete
       (if _isComplete (write-byte -1 s) (write-byte 0 s))
     ;; string _errorMsg
       (write-long (length _errorMsg) s) (princ _errorMsg s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _isComplete
     (setq _isComplete (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _errorMsg
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _errorMsg (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass uv_robot_ros::cmdToRpi
  :super ros::object
  :slots ())

(setf (get uv_robot_ros::cmdToRpi :md5sum-) "ab56ca80f4b06b7d4d8f78fb733348e9")
(setf (get uv_robot_ros::cmdToRpi :datatype-) "uv_robot_ros/cmdToRpi")
(setf (get uv_robot_ros::cmdToRpi :request) uv_robot_ros::cmdToRpiRequest)
(setf (get uv_robot_ros::cmdToRpi :response) uv_robot_ros::cmdToRpiResponse)

(defmethod uv_robot_ros::cmdToRpiRequest
  (:response () (instance uv_robot_ros::cmdToRpiResponse :init)))

(setf (get uv_robot_ros::cmdToRpiRequest :md5sum-) "ab56ca80f4b06b7d4d8f78fb733348e9")
(setf (get uv_robot_ros::cmdToRpiRequest :datatype-) "uv_robot_ros/cmdToRpiRequest")
(setf (get uv_robot_ros::cmdToRpiRequest :definition-)
      "string cmdType
string dist_or_deg
---
bool isComplete
string errorMsg

")

(setf (get uv_robot_ros::cmdToRpiResponse :md5sum-) "ab56ca80f4b06b7d4d8f78fb733348e9")
(setf (get uv_robot_ros::cmdToRpiResponse :datatype-) "uv_robot_ros/cmdToRpiResponse")
(setf (get uv_robot_ros::cmdToRpiResponse :definition-)
      "string cmdType
string dist_or_deg
---
bool isComplete
string errorMsg

")



(provide :uv_robot_ros/cmdToRpi "ab56ca80f4b06b7d4d8f78fb733348e9")


