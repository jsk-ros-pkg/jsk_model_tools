;; This euscollada-robot class is for robots converted from collada files.
;; This class provides :init-ending override.
(defclass euscollada-robot
  :super robot-model
  :slots ()
  )
(defmethod euscollada-robot
  (:init-ending
   ()
   (dolist (l links)
     (setq (l . acentroid) (send l :inverse-transform-vector
                                 (send (send (car links) :worldcoords) :transform-vector
                                       (l . acentroid))))
     (setq (l . inertia-tensor) (m* (transpose (send l :worldrot))
                                    (m* (send (car links) :worldrot)
                                        (l . inertia-tensor))))
     )
   (send-super :init-ending)

   ;;
   (dolist (j (mapcan #'(lambda (x) (if (and (derivedp (cdr x) joint)
					     (not (memq (cdr x) (send self :joint-list))))
					(list (cdr x)))) (send self :slots)))
     (send (send j :child-link) :add-joint j)
     (send (send j :child-link) :add-parent-link (send j :parent-link))
     (send (send j :parent-link) :add-child-links (send j :child-link)))
   )
  )
