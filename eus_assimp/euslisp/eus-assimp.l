;;
;; assimp mesh reader for euslisp
;;
(require :eus-assimp "package://eus_assimp/lib/eus_assimp.so")
(defun load-mesh-file (fname &key ((:scale scl) 1.0) (gen-normal) (smooth-normal)
                             (split-large-mesh) (optimize-mesh) (identical-vertices) (fix-normal)
                             (dump-filename))
  (let ((ret
         (assimp-get-glvertices fname scl gen-normal smooth-normal split-large-mesh
                                optimize-mesh identical-vertices fix-normal dump-filename))
        glv)
    (when ret
      (setq glv (instance gl::glvertices :init ret))
      (send glv :filename fname)
      glv)
    ))

(defun store-glvertices (fname glvtcs
                               &key ((:scale scl) 1.0) (gen-normal) (smooth-normal)
                               (split-large-mesh) (optimize-mesh) (identical-vertices))
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
    (assimp-dump-glvertices fname
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
                                        &key ((:scale scl) 1.0) (gen-normal) (smooth-normal)
                                        (split-large-mesh) (optimize-mesh) (identical-vertices))
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
               (nreverse vertices-lst) (nreverse indices-lst)))
    (when ret
      (setq glv (instance gl::glvertices :init ret))
      glv)
    ))
