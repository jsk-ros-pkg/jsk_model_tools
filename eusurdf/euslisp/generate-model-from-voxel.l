#!/usr/bin/env roseus

(ros::roseus "generate_model_from_voxel")

(ros::roseus-add-srvs "jsk_pcl_ros")

(defun generate-eus-model
  (req)
  (let* ((res (send req :response))
         (marker (send req :voxel))
         (points (send marker :points))
         (marker-scale (send marker :scale))
         (marker-color (send marker :color))
         cube-list
         tmp-cube
         )
      ;; make cube-list
      (dolist (p points)
        (setq tmp-cube (make-cube (send marker-scale :x) (send marker-scale :y) (send marker-scale :z)))
        (send tmp-cube :locate (float-vector (send p :x) (send p :y) (send p :z)))
        (send tmp-cube :set-color (float-vector (send marker-color :r) (send marker-color :g) (send marker-color :b)))
        (push tmp-cube cube-list)
        )
      ;; assoc each cube to root cube
      (dolist (c (cdr cube-list))
        (send (car cube-list) :assoc c)
        )
      ;;
      (objects cube-list)
      (x::window-main-one)
    res))

(defun main
  ()
  (ros::advertise-service "/generate_voxel_eus_model" jsk_pcl_ros::VoxelModelGenerate #'generate-eus-model)
  (ros::spin)
  )

(main)
