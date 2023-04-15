;; Auto-generated. Do not edit!


(when (boundp 'beginner_tutorials::BallLocation)
  (if (not (find-package "BEGINNER_TUTORIALS"))
    (make-package "BEGINNER_TUTORIALS"))
  (shadow 'BallLocation (find-package "BEGINNER_TUTORIALS")))
(unless (find-package "BEGINNER_TUTORIALS::BALLLOCATION")
  (make-package "BEGINNER_TUTORIALS::BALLLOCATION"))

(in-package "ROS")
;;//! \htmlinclude BallLocation.msg.html


(defclass beginner_tutorials::BallLocation
  :super ros::object
  :slots (_bearing _distance ))

(defmethod beginner_tutorials::BallLocation
  (:init
   (&key
    ((:bearing __bearing) 0.0)
    ((:distance __distance) 0.0)
    )
   (send-super :init)
   (setq _bearing (float __bearing))
   (setq _distance (float __distance))
   self)
  (:bearing
   (&optional __bearing)
   (if __bearing (setq _bearing __bearing)) _bearing)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:serialization-length
   ()
   (+
    ;; float64 _bearing
    8
    ;; float64 _distance
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _bearing
       (sys::poke _bearing (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _distance
       (sys::poke _distance (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _bearing
     (setq _bearing (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _distance
     (setq _distance (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get beginner_tutorials::BallLocation :md5sum-) "e6b9899956f1eacd3991a6d711941249")
(setf (get beginner_tutorials::BallLocation :datatype-) "beginner_tutorials/BallLocation")
(setf (get beginner_tutorials::BallLocation :definition-)
      "float64 bearing
float64 distance

")



(provide :beginner_tutorials/BallLocation "e6b9899956f1eacd3991a6d711941249")


