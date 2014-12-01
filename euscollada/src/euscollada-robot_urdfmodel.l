(defclass euscollada-robot
;; This euscollada-robot class is for robots converted from collada files.
;; This class provides :init-ending override.
  :super robot-model
  :slots ()
  )
(defmethod euscollada-robot
  (:init-ending ()
   (send-super :init-ending)
   (send self :make-collision-model-for-links)
   ;;
   (dolist (j (mapcan #'(lambda (x) (if (and (derivedp (cdr x) joint)
                                             (not (memq (cdr x) (send self :joint-list))))
                                        (list (cdr x)))) (send self :slots)))
     (send (send j :child-link) :add-joint j)
     (send (send j :child-link) :add-parent-link (send j :parent-link))
     (send (send j :parent-link) :add-child-links (send j :child-link)))
   )
  (:change-visual
   (&optional (visual :convex))
   (dolist (lk (send self :links))
     (dolist (bd (send lk :bodies))
       (when (derivedp bd collada-body)
         (send bd :change-visual visual)))))
  ;; fullbody-inverse-kinematics overwrite
  ;;  reduce root-link's weight based on leg's joint limit
  ;;  increase stop and cog-gain
  (:fullbody-inverse-kinematics
   (target-coords &rest args &key (stop 250) (cog-gain 2.0) (additional-weight-list) &allow-other-keys)
   (let* ((leg-ul (send self :calc-union-link-list (send self :legs :links))))
     (send self :reset-joint-angle-limit-weight-old leg-ul)
     (send-message* self robot-model :fullbody-inverse-kinematics
                    target-coords
                    :stop stop :cog-gain cog-gain
                    :additional-weight-list
                    (append
                     additional-weight-list
                     (list
                      (list (car (send self :links))
                            #'(lambda ()
                                ;; set root-link 6dof-joint's weight based on legs' joint limit
                                (let* ((min-weight
                                        (reduce #'(lambda (x y) (min x y))
                                                (coerce (send self :calc-inverse-kinematics-weight-from-link-list
                                                              leg-ul :union-link-list leg-ul) cons))))
                                  (fill (instantiate float-vector 6) min-weight)))
                            )
                      ))
                    args)
     ))
  ;; make collision model from faces or gl-vertices
  (:make-collision-model-for-links
   (&key (fat 0) (collision-func 'pqp-collision-check) ((:links ls) (send self :links)))
   (dolist (ll ls)
     (unless (send ll :get (read-from-string (format nil ":~Amodel"
                                                     (string-right-trim "-COLLISION-CHECK" (string collision-func)))))
       (send ll
             (read-from-string
              (format nil ":make-~Amodel"
                      (string-right-trim "-COLLISION-CHECK" (string collision-func))))
             :fat fat
             :faces (flatten (mapcar #'(lambda (x)
                                         (cond
                                          ((find-method x :def-gl-vertices)
                                           (send (x . glvertices) :convert-to-faces :wrt :world))
                                          (t
                                           (send x :faces))))
                                     (send ll :bodies)))))
     )
   )
  )

(defclass collada-body
;; This euscollada-body class is for bodies in robot model converted from collada files.
;; Class definition is in euscollada/src/euscollada-robot.l
;; This class provides :draw override for displaying glvertices.
  :super body
  :slots (glvertices)
  )
(defmethod collada-body
  (:init (&key (name) (replace-obj))
    (if replace-obj
        (replace-object self replace-obj)
      (replace-object self (make-cube 10 10 10)))
    (if name (send self :name name))
    self)
  (:draw (vwr)
   (when glvertices
     (send glvertices :draw vwr)))
  (:set-color (&rest args)
   (send-super* :set-color args)
   (when glvertices (send* glvertices :set-color args)))
  (:change-visual
   (&optional (visual :convex))
   (case visual
     ((list :convex :hull :convex-hull)
      (unless (send self :get :original-mesh)
        (send self :put :original-mesh glvertices)) ;; store original mesh
      (let* ((org-mesh (send self :get :original-mesh))
             (org-cds (send org-mesh :copy-worldcoords))
             (glv (gl::make-glvertices-from-faces (send self :faces))))
        (send glv :set-offset (send org-cds :inverse-transformation))
        (send glv :transform org-cds)
        (send org-mesh :assoc glv)
        (setq glvertices glv)
        glv))
     ((list :original :mesh :glvertices :original-mesh)
      (let ((org-mesh (send self :get :original-mesh)))
        (when org-mesh
          (dolist (d (send org-mesh :descendants)) ;; dissoc all discendants
            (send org-mesh :dissoc d))
          (setq glvertices org-mesh)
          org-mesh)))
     ))
  )
