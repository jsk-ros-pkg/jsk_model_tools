(defclass euscollada-robot
;; This euscollada-robot class is for robots converted from collada files.
;; This class provides :init-ending override.
  :super robot-model
  :slots ()
  )
(defmethod euscollada-robot
  (:init-ending
   ()
   ;; fix all links' mass properties ;; root link relative (collada) -> link local (euslisp)
   (dolist (l links)
     (setq (l . acentroid) (send l :inverse-transform-vector
                                 (send (send (car links) :worldcoords) :transform-vector
                                       (l . acentroid))))
     (setq (l . inertia-tensor) (m* (transpose (send l :worldrot))
                                    (m* (send (car links) :worldrot)
                                        (l . inertia-tensor))))
     )
   (send-super :init-ending)
   (dolist (l (send self :links)) (send self :make-detail-collision-model-from-glvertices-for-one-link l))

   ;;
   (dolist (j (mapcan #'(lambda (x) (if (and (derivedp (cdr x) joint)
					     (not (memq (cdr x) (send self :joint-list))))
					(list (cdr x)))) (send self :slots)))
     (send (send j :child-link) :add-joint j)
     (send (send j :child-link) :add-parent-link (send j :parent-link))
     (send (send j :parent-link) :add-child-links (send j :child-link)))
   ;; add sensor method ;; e.g., (send self :camera 0), (send self :force-sensor :rasensor), ... etc
   (dolist (sensor-name '(:force-sensor :imu-sensor))
     (eval `(defmethod ,(send (class self) :name)
              (,sensor-name (&rest args)
                            (cond ((integerp (car args))
                                   (forward-message-to (elt (send self ,(read-from-string (format nil "~As" sensor-name)) ) (car args)) (cdr args)))
                                  ;; enable to access sensors by limb name
                                  ((memq (car args) '(:larm :rarm :lleg :rleg :head :torso))
                                   (find-if
                                    #'(lambda (x) (member (send x :parent) (send self (car args) :links)))
                                    (send self ,(read-from-string (format nil "~As" sensor-name)))))
                                  ((and (keywordp (car args))
                                        (derivedp (send self (car args)) cascaded-coords))
                                   (send* self args))
                                  ((keywordp (car args))
                                   ;;(warn ";; no such sensor ~A~%" (car args))
                                   nil)
                                  (t
                                   (forward-message-to (car (send self ,(read-from-string (format nil "~As" sensor-name)))) args)
                                   )))
              )))
   )
  ;; fullbody-inverse-kinematics overwrite
  ;;  reduce root-link's weight based on leg's joint limit
  ;;  increase stop and cog-gain
  (:fullbody-inverse-kinematics
   (target-coords &rest args &key (stop 250) (cog-gain 2.0) &allow-other-keys)
   (let* ((leg-ul (send self :calc-union-link-list (send self :legs :links))))
     (send self :reset-joint-angle-limit-weight-old leg-ul)
     (send-message* self robot-model :fullbody-inverse-kinematics
                    target-coords
                    :stop stop :cog-gain cog-gain
                    :weight #'(lambda (ul)
                                (let ((tw (fill (instantiate float-vector (send self :calc-target-joint-dimension ul)) 1.0)))
                                  ;; set root-link 6dof-joint's weight based on legs' joint limit
                                  (let* ((min-weight
                                          (reduce #'(lambda (x y) (min x y))
                                                  (coerce (send self :calc-inverse-kinematics-weight-from-link-list
                                                                leg-ul :union-link-list leg-ul) cons))))
                                    (dotimes (i 6) (setf (elt tw i) (* min-weight (elt tw i)))))
                                  ;; additional weight setting
                                  (if (memq :weight args)
                                      (let ((tmp-weight2 (funcall (cadr (memq :weight args)) ul)))
                                        (dotimes (i (length tmp-weight2)) (setf (elt tw i) (* (elt tw i) (elt tmp-weight2 i))))))
                                  tw))
                    args)
     ))
  ;; make collision model from gl-vertices
  (:make-detail-collision-model-from-glvertices-for-one-link
   (ll &key (fat 0) (collision-func 'pqp-collision-check))
   (unless (send ll :get (read-from-string (format nil ":~Amodel"
                                                   (string-right-trim "-COLLISION-CHECK" (string collision-func)))))
     (send-message ll cascaded-coords
                   (read-from-string
                    (format nil ":make-~Amodel"
                            (string-right-trim "-COLLISION-CHECK" (string collision-func))))
                   :fat fat
                   :faces (flatten (mapcar #'(lambda (x)
                                               (cond
                                                ((derivedp x collada-body)
                                                 (send (x . glvertices) :convert-to-faces :wrt :world))
                                                (t
                                                 (send x :faces))))
                                           (send ll :bodies)))))
   )
  )

;; copy euscollada-body class definition from euscollada/src/euscollada-robot.l
(defclass collada-body
  :super body
  :slots (glvertices)
  )
(defmethod collada-body
  (:draw (vwr)
   (when glvertices
     (send glvertices :draw vwr)))
  (:set-color (&rest args)
   (send-super* :set-color args)
   (when glvertices (send* glvertices :set-color args)))
  )
