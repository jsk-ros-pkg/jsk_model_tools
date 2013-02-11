;; This euscollada-robot class is for robots converted from collada files.
;; This class provides :init-ending override.
(defclass euscollada-robot
  :super robot-model
  :slots ()
  )
(defmethod euscollada-robot
  (:init-ending
   ()
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
