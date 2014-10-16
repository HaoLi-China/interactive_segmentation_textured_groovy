
(cl:in-package :asdf)

(defsystem "interactive_segmentation_textured-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "cornerPokePoseFind" :depends-on ("_package_cornerPokePoseFind"))
    (:file "_package_cornerPokePoseFind" :depends-on ("_package"))
    (:file "estimateRigid" :depends-on ("_package_estimateRigid"))
    (:file "_package_estimateRigid" :depends-on ("_package"))
    (:file "depthImage" :depends-on ("_package_depthImage"))
    (:file "_package_depthImage" :depends-on ("_package"))
    (:file "cornerFind" :depends-on ("_package_cornerFind"))
    (:file "_package_cornerFind" :depends-on ("_package"))
    (:file "computeICP" :depends-on ("_package_computeICP"))
    (:file "_package_computeICP" :depends-on ("_package"))
  ))