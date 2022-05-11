(require :unittest "lib/llib/unittest.l")

(init-unit-test)

;; copy from euscollada-model-conversion-test.l
(defun generate-eusmodel-from-collada (col-file eus-file &key (yaml-file))
  (let ((strm (piped-fork
               (if yaml-file
                   (format nil "rosrun euscollada collada2eus ~A ~A ~A && echo finished"
                           col-file yaml-file eus-file)
                 (format nil "rosrun euscollada collada2eus ~A ~A && echo finished"
                         col-file eus-file)))))
    ;; (assert (select-stream (list strm) 240) (format nil ";; faild to generate ~A within 240 second" eus-file)) ;; why return nil immidiately
    (dotimes (i 240)
      (when
	  (while (and (select-stream (list strm) 1)
		      (not (eq :eof (read-line strm nil :eof))))
	    (return t))
	(return))
      (assert (/= i  239) (format nil ";; faild to generate ~A within 240 second" eus-file))
      (unix::sleep 1)
      )
    (unix::sleep 1)
    (close strm)
    ))

(load "package://euscollada/src/euscollada-robot.l")
(defvar *loop* 0)
(defmethod euscollada-robot
  (:inverse-kinematics-loop (&rest args)
     (incf *loop*)
     (send-super* :inverse-kinematics-loop args)))

(defun test-mimic-joint (name)
  (let ((urdf-file (ros::resolve-ros-path (format nil "package://euscollada/test/robots/test-~A.urdf" name)))
        (yaml-file (ros::resolve-ros-path (format nil "package://euscollada/test/robots/test-~A.yaml" name)))
        (eus-file (format nil "/tmp/test-~A.l" name))
        param len av robot end-coords goal ret)
    (generate-eusmodel-from-collada urdf-file eus-file :yaml-file yaml-file)
    (load eus-file)
    (setq robot (funcall (intern (string-upcase (substitute #\_ #\- name)))))
    (format t "create robot model ~A~%" robot)
    (setq param (list (cons (send robot :joint1)  45)
                      (cons (send robot :joint2)  45)
                      (cons (send robot :joint3)  90)
                      (cons (send robot :joint4)  45)
                      (cons (send robot :joint5)   0)
                      (cons (send robot :joint6)  45)))
    (setq len (length (send robot :angle-vector)))
    (setq av (instantiate float-vector len))
    (objects (list robot))
    (dotimes (i len)
      (let* ((j (elt (send robot :joint-list) i))
             (p (assoc j param))
             (joint (car p))
             (angle (cdr p)))
        (if (and (string= (send j :name) "joint3") ;; for test-revolute-2
                 (eps= (elt (car (send (j . default-coords) :rpy-angle)) 1) 0.785398))
            (setq angle 45))
        (if (derivedp j linear-joint) ;; for test-linear
            (setq angle (* 10 angle)))
        (if (or (minusp (elt (j . axis) 0))
                (minusp (elt (j . axis) 1))
                (minusp (elt (j . axis) 2)))
            (setq angle (- angle)))
        (setf (elt av i) angle)
        (format t "moving ~A to ~A~%" joint angle)
        (dotimes (i 10)
          (send joint :joint-angle (* (+ i 1) 0.1 angle))
          (send *irtviewer* :draw-objects))
        (send joint :joint-angle angle)
        (send *irtviewer* :draw-objects)))
    ;; check target vector is equal to av
    (format t "target angle-vector is ~A~%" (send robot :angle-vector))
    (assert (v= (send robot :angle-vector) av))

    ;; get goal coordinates
    (setq end-coords (send robot :rarm :end-coords))
    (send end-coords :draw-on :size 100 :flush t)
    (setq goal (send end-coords :copy-worldcoords))

    (assert (eps= (elt (send goal :worldpos) 2) 0.0 0.1) "goal should be on the ground ~A" goal)
    ;; set initial posture
    (send robot :angle-vector (scale 0.6 av))
    ;; (send robot :angle-vector (scale 0.0 av))

    ;; start ik
    (setq *loop* 0)
    (setq ret (send robot :inverse-kinematics goal :move-target end-coords :stop 20
                    :debug-view t))
    (assert ret "ik solved")
    (format t "ik loop was ~A~%" *loop*)
    (assert (< *loop* 20) "check loop count ()" *loop*)
    ))

(deftest test-revolute
  (test-mimic-joint "revolute-1"))

;; joint3: 45 deg offset
(deftest test-revolute
  (test-mimic-joint "revolute-2"))

;; mimic joint2 -> joint3, :y->:y multiply: 2, offset: 0
(deftest test-revolute-mimic-1
  (test-mimic-joint "revolute-mimic-1"))

;; mimic joint2 -> joint3, :-y->:y multiply: -2, offset: 0
(deftest test-revolute-mimic-2
  (test-mimic-joint "revolute-mimic-2"))

;; mimic joint2 -> joint3, :y->:y multiply: 1, offset: 45
(deftest test-revolute-mimic-3
  (test-mimic-joint "revolute-mimic-3"))

;; mimic joint2 -> joint3, :-y->:y multiply: -1, offset: -45
(deftest test-revolute-mimic-4
  (test-mimic-joint "revolute-mimic-4"))

;; mimic joint1 -> joint3, :y->:y multiply: 2, offset: 0
(deftest test-revolute-mimic-5
  (test-mimic-joint "revolute-mimic-5"))

(deftest test-linear
  (test-mimic-joint "linear-1"))

(deftest test-linear-mimic-1
  (test-mimic-joint "linear-mimic-1"))

(deftest test-linear-mimic-2
  (test-mimic-joint "linear-mimic-2"))

(if (sys:list-all-catchers) ;; called from interpreter (progn (load "test-mimic.l") (test-mimic-joint "linear-mimic-1"))
    (setq lisp::*exit-on-fatal-error* nil)
  (progn ;; called as roseus argument `roseus test-mimic.l`
    (run-all-tests)
    (exit)))

