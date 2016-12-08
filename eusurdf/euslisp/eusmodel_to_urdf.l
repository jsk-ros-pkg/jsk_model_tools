;; eusmodel_to_urdf.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

;; assumed executed at PROJECT_SOURCE_DIR
(require :conversion-utils "euslisp/conversion-utils.l")

(defun make-model-conf (name fpath &optional desc)
  (unless desc
    (setq desc (format nil "This is automatically generated eus model from ~A" name)))
  (with-open-file (f fpath :direction :output :if-exists :overwrite)
    (format f "<?xml version='1.0'?>~%")
    (format f "<model>~%")
    (format f "  <name>~A</name>~%" name)
    (format f "  <version>0.1.0</version>~%")
    (format f "  <sdf>model.urdf</sdf>~%")
    (format f "  <description>~%")
    (format f "    ~A~%" desc)
    (format f "  </description>~%")
    (format f "</model>~%")))

(defun eusmodel2urdf (fname outdir &key (tmpdir "/tmp/"))
  (unless (probe-file fname)
    (errorf "file ~A not exists" fname))
  (load fname)
  (let* ((objname (string-join "-" (butlast (string-split (send (pathname fname) :name) #\-))))
         (urdf-out-path (send (concatenate-pathnames outdir "model.urdf") :namestring))
         (mesh-out-dir (send (concatenate-pathnames outdir "meshes/") :namestring))
         (obj (funcall (read-from-string objname))))
    (cond ((null (send obj :name)) (send obj :name objname))
          ((symbolp (send obj :name)) (send obj :name (string-downcase (send obj :name)))))

    (collada::eus2collada obj tmpdir)
    (make-dirs mesh-out-dir)
    (unix:system (format nil "rosrun collada_urdf_jsk_patch collada_to_urdf ~A -G -A --mesh_output_dir ~A --mesh_prefix \"model://~A/meshes\" -O ~A"
                         (send (concatenate-pathnames tmpdir objname ".dae") :namestring)
                         mesh-out-dir
                         objname
                         urdf-out-path))
    (unix:system (format nil "sed -i -e \"s@continuous@revolute@g\" ~A" urdf-out-path))
    (unix:system (format nil "sed -i -e \"s@<robot name=\\\"inst_kinsystem\\\"@<robot name=\\\"~A\\\"@g\" ~A" objname urdf-out-path))
    (unix:system (format nil "sed -i -e \"1,/  <link /s/  <link /  <gazebo>\\\n    <static>false<\\/static>\\\n  <\\/gazebo>\\\n  <link /\" ~A" urdf-out-path))
    (unix:system (format nil "sed -i -e \"s@      <inertia ixx=\\\"1e-09\\\" ixy=\\\"0\\\" ixz=\\\"0\\\" iyy=\\\"1e-09\\\" iyz=\\\"0\\\" izz=\\\"1e-09\\\"/>@      <inertia ixx=\\\"1e-03\\\" ixy=\\\"0\\\" ixz=\\\"0\\\" iyy=\\\"1e-03\\\" iyz=\\\"0\\\" izz=\\\"1e-03\\\"/>@g\" ~A" urdf-out-path))
    (make-model-conf objname (format nil "~A/model.config" outdir))))

(setq model-file-path (car (last (butlast lisp::*eustop-argument*))))
(setq output-directory (car (last lisp::*eustop-argument*)))
(format t "converting eus object ~A -> ~A~%" model-file-path output-directory)
(eusmodel2urdf model-file-path output-directory)
(exit)
