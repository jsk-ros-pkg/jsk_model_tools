#!/bin/bash

if [ $# -ge 1 ]; then
    ROBOTFILE=$1;
else
    ROBOTFILE=$(rospack find hrpsys_ros_bridge)/models/HRP4C.dae;
fi
LINKNAMES=$(sed -n -e 's@.*<link.*name="\([^ ]*\)".*@\1@p' $ROBOTFILE)
JOINTNAMES=$(sed -n -e 's@.*<joint name="\([^ ]*\)".*@\1@p' $ROBOTFILE)

for lname in ${LINKNAMES}; do
cat <<EOF
  <gazebo reference="${lname}">
    <material>Gazebo/Gray</material>
    <turnGravityOff>false</turnGravityOff>
  </gazebo>
EOF
done

TRSNO=0
for jname in ${JOINTNAMES}; do
cat <<EOF
 <transmission type="pr2_mechanism_model/SimpleTransmission" name="link${TRSNO}_trans">
    <actuator name="link${TRSNO}_motor" />
   <joint name="${jname}" />
   <mechanicalReduction>1</mechanicalReduction>
   <motorTorqueConstant>1</motorTorqueConstant>
   <pulsesPerRevolution>90000</pulsesPerRevolution>
 </transmission>
EOF
TRSNO=$(expr $TRSNO + 1);
done

cat <<EOF
joint_traj_controller:
  type: robot_mechanism_controllers/JointTrajectoryActionController
  joints:
EOF

TRSNO=0
for jname in ${JOINTNAMES}; do
    echo "    - ${jname}"
TRSNO=$(expr $TRSNO + 1);
done
echo "  gains:"
TRSNO=0
for jname in ${JOINTNAMES}; do
echo "    ${jname}: {p: 1000.0, d: 6.0, i: 600.0, i_clamp: 4.0}"
TRSNO=$(expr $TRSNO + 1);
done

cat <<EOF
  joint_trajectory_action_node:
    joints:
EOF

TRSNO=0
for jname in ${JOINTNAMES}; do
echo "      - ${jname}"
TRSNO=$(expr $TRSNO + 1);
done

cat <<EOF
    constraints:
      goal_time: 0.6
EOF

TRSNO=0
for jname in ${JOINTNAMES}; do
echo "      ${jname}:"
echo "        goal: 0.02"
TRSNO=$(expr $TRSNO + 1);
done
