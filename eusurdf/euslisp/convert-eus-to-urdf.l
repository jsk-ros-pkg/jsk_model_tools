(defun irteus2urdf-for-gazebo
  (model
   &key
   name
   eusurdf-package-path
   collada-to-urdf-exe-path
   (tmp-dir "/tmp")
   )
  (cond
   ((not (subclassp (class model) cascaded-link))
    (format t "invalid class, not childclass of cascaded-link~%")
    (return-from irteus2urdf-for-gazebo nil))
   ((null (send model :links))
    (format t "please setq links~%")
    (return-from irteus2urdf-for-gazebo nil))
   (t
    (unless name
      (if (or (not (stringp (send model :name)))
              (zerop (length (send model :name))))
          (setq name (remove #\: (format nil "~A" (send model :name)))))
      )
    (let* ((tmp-obj (instance (class model) :init)))
      (send tmp-obj :name name)
      (collada::eus2collada tmp-obj tmp-dir))
    ;;(collada::eus2collada model tmp-dir) ;; I don't know why, but this line does not work correctly.
    (cond
     ((probe-file (format nil "~a/~a.dae" tmp-dir name))
      (format t "[eus2urdf] generate ~A.dae~%" name)
      (unix:system (format nil "~a/scripts/eus2urdf_for_gazebo.py ~a ~a/~a.dae ~a ~a"
                           eusurdf-package-path name tmp-dir name eusurdf-package-path
                           (cond (collada-to-urdf-exe-path collada-to-urdf-exe-path) (t "")))))
     (t
      (format nil "dae error~%")
      (return-from irteus2urdf-for-gazebo nil)))
    ))
  t)
(warn "~%(irteus2urdf-for-gazebo (make-national-laundry-machine) \"laundry\")~%")

(defun irteus2urdf-room-for-gazebo
  (room
   &key
   make-static make-fixed
   world-file-path
   eusurdf-package-path
   collada-to-urdf-exe-path
   )
  (let* ((package-dir-path
          (cond (eusurdf-package-path eusurdf-package-path)
                (t (ros::resolve-ros-path "package://eusurdf")))))
    (dolist (obj (send room :objects))
      (let* ((obj-name)
             (obj-cnt 0))
        ;; deal with the objects with no name. (objects with no name cause an error in converting)
        (cond ((null (send obj :name)) (send obj :name "no-name"))
              ((symbolp (send obj :name)) (send obj :name (string-downcase (send obj :name)))))
        ;; set obj-name
        (while t
          (setq obj-name (string-downcase (format nil "~a-~a" (send obj :name) obj-cnt)))
          (let* ((model-file-path (format nil "~a/models/~a" package-dir-path obj-name)))
            (unless (probe-file model-file-path) (return)))
          (incf obj-cnt))
        (let* ((pos (scale 0.001 (send obj :worldpos)))
               (pos-string (format nil "~a ~a ~a" (elt pos 0) (elt pos 1) (elt pos 2)))
               (rpy (car (rpy-angle (send obj :worldrot))))
               (rpy-string (format nil "~a ~a ~a" (elt rpy 2) (elt rpy 1) (elt rpy 0))))
          ;; convert the model
          (when (irteus2urdf-for-gazebo obj :name obj-name :eusurdf-package-path package-dir-path :collada-to-urdf-exe-path collada-to-urdf-exe-path)
            ;; add the obects to the world file
            (unix:system (format nil "sed -i -e \"s@    <!-- Objects -->@    <!-- Objects -->\\n    <include>\\n      <uri>model://~a_static</uri>\\n      <pose>~a ~a</pose>\\n    </include>@g\" ~a" obj-name pos-string rpy-string world-file-path))
            ;; ;; make the static model
            (when make-static
              (unix:system (format nil "~a/scripts/make_static_model.py ~a ~a" package-dir-path obj-name package-dir-path)))
            ;; ;; make the fixed model
            (when make-fixed
              (unix:system (format nil "~a/scripts/make_fixed_model.py ~a ~a" package-dir-path obj-name package-dir-path)))
            )
          )
        ))
    ;; ;; generate the model database file
    ;; (unix:system (format nil "~a/scripts/generate_model_database.py" package-dir-path ))
    ))
(warn "(irteus2urdf-room-for-gazebo (room73b2))~%")


(defun generate-room-models
  (name
   &key
   eusurdf-package-path
   collada-to-urdf-exe-path
   )
  ;; check the function making room exits.
  (load (format nil "models/~a-scene.l" name))
  (unless (functionp (read-from-string name))
    (warn "[ERROR] room (~a) is not found.~%" name)
    (return-from generate-room-models))
  ;; check the model is derived from the room class
  (let* ((room (funcall (read-from-string name))))
    (unless (derivedp room scene-model)
      (warn "[ERROR] room (~a) is not the room model.~%" name)
      (return-from generate-room-models))
    ;; generate the world file
    (let* ((package-dir-path
            (cond (eusurdf-package-path eusurdf-package-path)
                  (t (ros::resolve-ros-path "package://eusurdf"))))
           (world-src-file-path (format nil "~a/worlds/world_source.world" package-dir-path))
           (world-new-file-path (format nil "~a/worlds/~a.world" package-dir-path name)))
      (unix:system (format nil "rm -rf ~a" world-new-file-path))
      (unix:system (format nil "cp ~a ~a" world-src-file-path world-new-file-path))
      ;; convert models
      (irteus2urdf-room-for-gazebo room :make-static t :make-fixed t :world-file-path world-new-file-path :eusurdf-package-path eusurdf-package-path :collada-to-urdf-exe-path collada-to-urdf-exe-path)
      )
    ))
(warn "(generate-room-models \"room73b2\")~%")
