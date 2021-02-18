;; Auto-generated. Do not edit!


(when (boundp 'pathfollow::Traj)
  (if (not (find-package "PATHFOLLOW"))
    (make-package "PATHFOLLOW"))
  (shadow 'Traj (find-package "PATHFOLLOW")))
(unless (find-package "PATHFOLLOW::TRAJ")
  (make-package "PATHFOLLOW::TRAJ"))

(in-package "ROS")
;;//! \htmlinclude Traj.msg.html


(defclass pathfollow::Traj
  :super ros::object
  :slots (_trajectoryX _trajectoryY ))

(defmethod pathfollow::Traj
  (:init
   (&key
    ((:trajectoryX __trajectoryX) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:trajectoryY __trajectoryY) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _trajectoryX __trajectoryX)
   (setq _trajectoryY __trajectoryY)
   self)
  (:trajectoryX
   (&optional __trajectoryX)
   (if __trajectoryX (setq _trajectoryX __trajectoryX)) _trajectoryX)
  (:trajectoryY
   (&optional __trajectoryY)
   (if __trajectoryY (setq _trajectoryY __trajectoryY)) _trajectoryY)
  (:serialization-length
   ()
   (+
    ;; float32[] _trajectoryX
    (* 4    (length _trajectoryX)) 4
    ;; float32[] _trajectoryY
    (* 4    (length _trajectoryY)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _trajectoryX
     (write-long (length _trajectoryX) s)
     (dotimes (i (length _trajectoryX))
       (sys::poke (elt _trajectoryX i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _trajectoryY
     (write-long (length _trajectoryY) s)
     (dotimes (i (length _trajectoryY))
       (sys::poke (elt _trajectoryY i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _trajectoryX
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _trajectoryX (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _trajectoryX i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _trajectoryY
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _trajectoryY (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _trajectoryY i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get pathfollow::Traj :md5sum-) "2343a9eebc0c84e51eef389e14d15ade")
(setf (get pathfollow::Traj :datatype-) "pathfollow/Traj")
(setf (get pathfollow::Traj :definition-)
      "float32[] trajectoryX
float32[] trajectoryY
")



(provide :pathfollow/Traj "2343a9eebc0c84e51eef389e14d15ade")


