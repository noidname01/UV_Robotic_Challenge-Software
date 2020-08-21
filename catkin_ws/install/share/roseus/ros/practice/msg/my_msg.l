;; Auto-generated. Do not edit!


(when (boundp 'practice::my_msg)
  (if (not (find-package "PRACTICE"))
    (make-package "PRACTICE"))
  (shadow 'my_msg (find-package "PRACTICE")))
(unless (find-package "PRACTICE::MY_MSG")
  (make-package "PRACTICE::MY_MSG"))

(in-package "ROS")
;;//! \htmlinclude my_msg.msg.html


(defclass practice::my_msg
  :super ros::object
  :slots (_id _title _content ))

(defmethod practice::my_msg
  (:init
   (&key
    ((:id __id) 0)
    ((:title __title) "")
    ((:content __content) "")
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _title (string __title))
   (setq _content (string __content))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:title
   (&optional __title)
   (if __title (setq _title __title)) _title)
  (:content
   (&optional __content)
   (if __content (setq _content __content)) _content)
  (:serialization-length
   ()
   (+
    ;; int64 _id
    8
    ;; string _title
    4 (length _title)
    ;; string _content
    4 (length _content)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _id
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _id (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _id) (= (length (_id . bv)) 2)) ;; bignum
              (write-long (ash (elt (_id . bv) 0) 0) s)
              (write-long (ash (elt (_id . bv) 1) -1) s))
             ((and (class _id) (= (length (_id . bv)) 1)) ;; big1
              (write-long (elt (_id . bv) 0) s)
              (write-long (if (>= _id 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _id s)(write-long (if (>= _id 0) 0 #xffffffff) s)))
     ;; string _title
       (write-long (length _title) s) (princ _title s)
     ;; string _content
       (write-long (length _content) s) (princ _content s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _id
#+(or :alpha :irix6 :x86_64)
      (setf _id (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _id (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; string _title
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _title (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _content
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _content (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get practice::my_msg :md5sum-) "1e2cb8cf2c48e86d2e59515dd5faeb6a")
(setf (get practice::my_msg :datatype-) "practice/my_msg")
(setf (get practice::my_msg :definition-)
      "int64 id
string title
string content

")



(provide :practice/my_msg "1e2cb8cf2c48e86d2e59515dd5faeb6a")


