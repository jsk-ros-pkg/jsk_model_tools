;;
;; assimp mesh reader for euslisp
;;
(require :eus-assimp "package://eus_assimp/lib/eus_assimp.so")
(defun load-mesh-file (fname &key ((:scale scl) 1.0) (gen-normal) (smooth-normal)
                             (split-large-mesh) (optimize-mesh) (identical-vertices) (fix-normal)
                             (direction :z_up) (dump-filename) &allow-other-keys)
  (case direction
    ((:x_up :x-up) (setq direction 0))
    ((:y_up :y-up) (setq direction 1))
    ((:z_up :z-up) (setq direction 2))
    ((:-x_up :-x-up) (setq direction 3))
    ((:-y_up :-y-up) (setq direction 4))
    ((numberp direction) );; do-nothing
    (t
     (warn ";; unknown direction ~A~%" direction)))

  (let ((ret
         (c-assimp-get-glvertices fname scl gen-normal smooth-normal split-large-mesh
                                  optimize-mesh identical-vertices fix-normal direction dump-filename))
        glv)
    (when ret
      (let ((fp (pathname fname)))
        (setq glv (instance gl::glvertices :init ret))
        (send glv :filename fname)
        (send glv :load-texture-image
              (pathname-directory (truename fp))))
      glv)
    ))

(defun store-glvertices (fname glvtcs
                               &key ((:scale scl) 1.0) (gen-normal) (smooth-normal)
                               (split-large-mesh) (optimize-mesh) (identical-vertices)
                               &allow-other-keys)
  (let ((mlst (glvtcs . gl::mesh-list))
        name-lst type-lst
        material-lst
        vertices-lst
        normals-lst
        indices-lst)
    (dolist (m mlst)
      (push (case (cadr (assoc :type m))
              (:triangles 3)
              (:quads 4)
              (t 0)) type-lst)
      (let ((mat (cadr (assoc :material m))))
        (push
         (mapcar #'(lambda (x)
                     (cadr (assoc x mat)))
                 (list :ambient :diffuse :specular :emission :shininess :transparency))
         material-lst))
      (push (cadr (assoc :vertices m)) vertices-lst)
      (push (cadr (assoc :normals m)) normals-lst)
      (push (cadr (assoc :indices m)) indices-lst)
      (push (cadr (assoc :name m)) name-lst)
      )
    ;;(pprint (list (nreverse type-lst) (nreverse material-lst)
    ;;(nreverse vertices-lst) (nreverse normals-lst) (nreverse indices-lst)))
    (c-assimp-dump-glvertices fname
                              (nreverse type-lst) (nreverse material-lst)
                              (nreverse vertices-lst) (nreverse normals-lst) (nreverse indices-lst)
                              scl gen-normal smooth-normal
                              split-large-mesh optimize-mesh identical-vertices)
    ))
;;
(defun append-glvertices (glv-lst)
  (let (ret)
    (dolist (glv glv-lst)
      (let ((mlst (glv . gl::mesh-list))
            (cds (send glv :worldcoords)))
        ;; not concerning coordinates
        (setq ret (append ret mlst))
        ))
    (instance gl::glvertices :init ret)))

(defun convex-decomposition-glvertices (glvtcs
                                        &key ((:scale scl) 1.0)
                                        (skinwidth 0.0) (decomposition-depth 4) (max-hull-vertices 64)
                                        (concavity-threshold 0.1) (merge-threshold 20.0) (volume-split-threshold 2.0))
  (let ((mlst (glvtcs . gl::mesh-list))
        ;;name-lst ;; type-lst
        ;;material-lst ;;normals-lst
        vertices-lst
        indices-lst
        ret glv)
    (dolist (m mlst)
      (when (eq :triangles (cadr (assoc :type m)))
        (push (cadr (assoc :vertices m)) vertices-lst)
        (push (cadr (assoc :indices m)) indices-lst)))
    (setq ret (c-convex-decomposition-glvertices
               (nreverse vertices-lst) (nreverse indices-lst)
               skinwidth decomposition-depth max-hull-vertices
               concavity-threshold merge-threshold volume-split-threshold))
    (when ret
      (setq glv (instance gl::glvertices :init ret))
      glv)
    ))

(when (and (fboundp 'c-assimp-load-image)
           (not (fboundp 'read-image-file-org)))
  (defun assimp-read-image-file (fname)
    (let* ((ret (c-assimp-load-image fname))
           (w (cadr (assoc :width ret)))
           (h (cadr (assoc :height ret)))
           (byte/pixel (cadr (assoc :type ret)))
           (buf (cadr (assoc :vertices ret))))
      (when ret
        (case byte/pixel
          (1
           (instance grayscale-image :init w h buf))
          (3
           (instance color-image24 :init w h buf))
          (4
           (instance color-image32 :init w h buf))
          (t
           (warn ";; not implemented byte/pixel ~d~%" byte/pixel)
           )))))
  (setf (symbol-function 'read-image-file-org)
        (symbol-function 'read-image-file))
  ;; override read-image-file
  (defun read-image-file (fname)
    (let ((r (read-image-file-org fname)))
      (unless r
        (setq r (assimp-read-image-file fname)))
      r))
  )

(defun make-glvertices-from-faces (flst &key (material))
  (setq flst (flatten (mapcar #'(lambda (f) (geometry::face-to-triangle-aux f)) flst)))
  (let ((mat (make-matrix (* 3 (length flst)) 3))
        (nom (make-matrix (* 3 (length flst)) 3))
        (idx (instantiate integer-vector (* 3 (length flst))))
        (cntr 0))
    (dolist (f flst)
      (let ((nm (normalize-vector (send f :normal)))
            (vsl (send f :vertices)))
        (c-matrix-row mat cntr (car vsl) t)
        (c-matrix-row nom cntr nm t)
        (incf cntr)

        (c-matrix-row mat cntr (cadr vsl) t)
        (c-matrix-row nom cntr nm t)
        (incf cntr)

        (c-matrix-row mat cntr (caddr vsl) t)
        (c-matrix-row nom cntr nm t)
        (incf cntr)
        ))
    (dotimes (i (length idx)) (setf (elt idx i) i))

    (let ((msh
           (list (list :type :triangles)
                 (list :vertices mat)
                 (list :normals nom)
                 (list :indices idx))))
      (when material
        (push (list :material material) msh))
      (instance gl::glvertices :init (list msh)))
    ))

(defun dump-to-meshfile (fname obj &rest args &key (material) &allow-other-keys)
  (let (faces)
    (cond
     ((find-method obj :bodies)
      (let (glv-lst)
        (dolist (bd (send obj :bodies))
          (let (mat)
            (if material (setq mat material))
            ;; TODO: get color and set matterial
            (push (apply #'make-glvertices-from-faces faces :material mat args)
                  glv-lst)))
        (store-glvertices fname (append-glvertices glv-lst))
        ))
     ((find-method obj :faces)
      (setq faces (send obj :faces))
      (store-glvertices
       fname
       (apply #'make-glvertices-from-faces faces :material material args)))
     (t
      (warn ";; face not found~%")
      ))
    fname))

(defun make-cube-from-bounding-box (bbox &rest args)
  (let ((cds (make-coords :pos (send bbox :center)))
        (dsize (send bbox :diagonal)) ret)
    (setq ret (apply #'make-cube (elt dsize 0) (elt dsize 1) (elt dsize 2) args))
    (send ret :translate-vertices (send cds :pos))
    ret))

;; urdf_to_collada -> collada2eus ..
(defun update-to-original-meshfile (robot &key (scale 1000.0) (direction :-y_up))
  (dolist (bd (send robot :bodies))
    (let ((str (send bd :name)))
      (when (and (stringp str)
                 (substringp ".dae" str))
        (let ((fn (ros::resolve-ros-path str))
              (old-glv (bd . glvertices))
              glv)
          (setq glv (load-mesh-file fn :scale scale :direction direction))
          (send glv :transform (send old-glv :worldcoords))
          (let ((p (send old-glv :parent)))
            (send p :dissoc old-glv)
            (send p :assoc glv))
          (setq (bd . glvertices) glv))
        )))
  robot)

(in-package "GL")
(defmethod glvertices
  (:load-texture-image (&optional dir)
   (dolist (minfo mesh-list)
     (let ((matinfo (cadr (assoc :material minfo))))
       (let ((fname (cadr (assoc :filename matinfo)))
             (img   (cadr (assoc :teximage matinfo))))
         ;;(pprint (list matinfo fname))
         (when (and fname (null img))
           (when (substringp ".tif" fname)
             (setq fname (format nil "~A.png" (subseq fname 0 (- (length fname) 4)))))
           ;; relative path -> absolute path
           (let ((pf (pathname fname)))
             (unless (or (null dir) (eq (car (pathname-directory pf)) :root)) ;; absolute path
               (setq (pf . directory) (append dir (pf . directory)))
               (setq fname (namestring pf))
               ))
           (setq img (image::read-image-file fname))
           ;; TODO: check color-image24 and packed to RGB24bit
           ;;(pprint (list fname img))
           (when img
             (nconc matinfo (list (list :teximage img))))
           ;;; load other method
           )
         )))
   )
  )
(in-package "USER")
