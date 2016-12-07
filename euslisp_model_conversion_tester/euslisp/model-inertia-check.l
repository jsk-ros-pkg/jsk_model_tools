(defun confirm-link-parameters (lk) ;; lk is link
  (let* ((iner (send lk :inertia-tensor))
         (mass (send lk :weight))
         (cent (lk . acentroid))
         (cds  (send lk :copy-worldcoords))
         (vts  (mapcar #'(lambda (v) (send cds :inverse-transform-vector v))
                       (flatten (send-all (send lk :bodies) :vertices))))
         (bbox (make-bounding-box vts 0.0))
         (jt (send lk :joint))
         iner-box
         )
    (let ((v (send bbox :diagonal))
          l eigen)
      (setq l (list
               (/ (* mass (+ (expt (elt v 1) 2)
                             (expt (elt v 2) 2))) 12)
               (/ (* mass (+ (expt (elt v 0) 2)
                             (expt (elt v 2) 2))) 12)
               (/ (* mass (+ (expt (elt v 0) 2)
                             (expt (elt v 1) 2))) 12)))

      (format t ";; density: ~A (kg/m3)~%" (* 1000 1000 (/ mass (* (elt v 0) (elt v 1) (elt v 2)))))
      (format t ";; center of mass ~A / ~A (center of bounding box)~%" cent (send bbox :center))
      (format t ";; inertia matrix ~A~%" iner)
      (format t ";; inertia bounding box ~A~%" l)
      ;;
      (setq l (sort l #'>=))
      (let ((ret (sv-decompose iner)))
        (setq eigen (cadr ret))
        ;;(pprint (list (matrix-determinant (car ret)) (matrix-determinant (caddr ret))))
        (unless
            (eps= (matrix-determinant (car ret))
                  (matrix-determinant (caddr ret)))
          (warning-message 3 ";;;;;; wrong inertia matrix ;;;;;;~%"))

        (let ((mat (car ret)))
          (if (<= (matrix-determinant mat) 0)
              (scale -1 (array-entity mat) (array-entity mat)))
          #|(pprint
           (list (sqrt (/ (elt eigen 0) mass))
                 (sqrt (/ (elt eigen 1) mass))
                 (sqrt (/ (elt eigen 2) mass))
                 mat
                 bbox
                 ))|#
          (setq iner-box (make-cube (sqrt (/ (* 6 (- (+ (elt eigen 2) (elt eigen 1)) (elt eigen 0))) mass))
                                    (sqrt (/ (* 6 (- (+ (elt eigen 2) (elt eigen 0)) (elt eigen 1))) mass))
                                    (sqrt (/ (* 6 (- (+ (elt eigen 0) (elt eigen 1)) (elt eigen 2))) mass))))
          (send iner-box :newcoords (make-coords :pos (copy-seq cent) :rot mat))
          ;;(pprint iner-box)
          (send iner-box :transform (send lk :worldcoords) :world)
          (send iner-box :set-color (float-vector 1 0 1) 0.5)
          )
        )
      (format t ";; inertia bbox / model   -> bbox/model ratio~%")
      ;; if ratio >> 1  high density nearby centroid

      (format t ";; ~A/~A ~A/~A ~A/~A -> ~A ~A ~A~%"
              (elt l 0) (elt eigen 0)
              (elt l 1) (elt eigen 1)
              (elt l 2) (elt eigen 2)
              (/ (elt l 0) (elt eigen 0))
              (/ (elt l 1) (elt eigen 1))
              (/ (elt l 2) (elt eigen 2)))
      ;; g/mm3 -> 10^6 kg/m3
      )
    ;;;; inertia ratio between links on joint axis
    (when jt
      (let* ((pt (send jt :parent-link))
             jiner
             ch-inertia pt-inertia)
        (setq jiner (translate-inertia iner mass (v- cent)))
        (setq ch-inertia (v. (jt . axis) (transform (jt . axis) jiner)))

        (format t ";; link coordinates~%")
        (format t ";; ~A ~A~%"
                (transform (jt . axis) jiner)
                ch-inertia)

        (let* ((pcent (pt . acentroid))
               (dc-pos (send (jt . default-coords) :worldpos))
               pjiner)
          (setq pjiner (translate-inertia (send pt :inertia-tensor)
                                          (send pt :weight)
                                          (v- dc-pos pcent)))
          (setq pt-inertia (v. (jt . axis) (transform (jt . axis) pjiner)))
          (format t ";; parent coordinates~%")
          (format t ";; ~A ~A~%"
                  (transform (jt . axis) pjiner)
                  pt-inertia)
          (format t ";; ratio ~A~%" (/ ch-inertia pt-inertia))
          )))
    iner-box
    ))
(defun translate-inertia (I mass v)
  ;; I(inertia-matrix) on center of mass
  (let ((ax (elt v 0))
        (ay (elt v 1))
        (az (elt v 2))
        m)
    (setq m
          (matrix (float-vector (+ (* ay ay) (* az az)) (* (- ay) ax)            (* (- az) ax))
                  (float-vector (* (- ay) ax)           (+ (* ax ax) (* az az))  (* (- az) ay))
                  (float-vector (* (- az) ax)           (* (- az) ay)            (+ (* ax ax) (* ay ay)))))
    (scale mass (array-entity m) (array-entity m))
    (m+ I m m)
    m))
(defun rotate-inertia (I trans)
  (m* (m* (transpose trans) I) trans))

;;
(defun create-inertia-model (robot)
  (let ((bds
         (mapcar #'(lambda (lk)
                     (format t "~%;;; ~A ;;;~%" (send lk :name))
                     (confirm-link-parameters lk))
                 (append (list (car (send robot :links)))
                         (all-child-links (car (send robot :links))))
                 )))
    (objects (append (list robot) bds))
    ))

(warn "
(load \"package://hrpsys_ros_bridge_tutorials/models/samplerobot.l\")
(samplerobot)
(create-inertia-model *samplerobot*)
")
