#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib; roslib.load_manifest("hrpsys_gazebo")
import rospy
from sensor_msgs.msg import JointState
from optparse import OptionParser

## trajectory_action
import actionlib
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *
from trajectory_msgs.msg import *

import time

if __name__ == "__main__":
    parser = OptionParser(description='pub_joint_trajectory_action')

    parser.add_option('--joints', action='store', type='string', dest='joints', default=None,
                      help='')
    parser.add_option('--time', action='store', type='float', dest='time', default=1.0,
                      help='')
    parser.add_option('--start_time', action='store', type='float', dest='start_after', default=0.2,
                      help='')
    parser.add_option('--position', action='store', type='string', dest='position', default=None,
                      help='')
    parser.add_option('--velocity', action='store', type='string', dest='velocity', default=None,
                      help='')
    parser.add_option('--effort', action='store', type='string', dest='effort', default=None,
                      help='')
    parser.add_option('--action', action='store', type='string', dest='action', default='joint_states',
                      help='')

    (options, args) = parser.parse_args()

    if options.joints is not None:
        jlist = options.joints.split(',')
    else:
        print('There is no joint inputs')
        exit(0)

    plist = None
    vlist = None
    elist = None
    if options.position is not None:
        plist = options.position.split(',')
    if options.velocity is not None:
        vlist = options.velocity.split(',')
    if options.effort is not None:
        elist = options.effort.split(',')

    rospy.init_node("joint_state_publisher")

    client = actionlib.SimpleActionClient(options.action, JointTrajectoryAction)
    rospy.loginfo("wait server ( %s ) ... "%options.action)
    client.wait_for_server()
    rospy.loginfo("server ( %s ) found "%options.action)

    g = JointTrajectoryGoal()

    for i in range(len(jlist)):
        g.trajectory.joint_names.append(jlist[i])

    pt = JointTrajectoryPoint()
    for i in range(len(jlist)):
        if plist is not None and len(plist) > i:
            pt.positions.append(float(plist[i]))
        if vlist is not None and len(vlist) > i:
            pt.velocities.append(float(vlist[i]))
        if elist is not None and len(elist) > i:
            pt.accelerations.append(float(elist[i]))
        if (plist is None) and (vlist is None) and (elist is None):
            pt.positions.append(0)

    pt.time_from_start = rospy.Duration( options.time )
    g.trajectory.points.append(pt);

    g.trajectory.header.seq = 1
    g.trajectory.header.stamp = ( rospy.Time.now() + rospy.Duration( options.start_after) )

    client.send_goal(g)
    rospy.loginfo("send goal")
    client.wait_for_result( )
    #client.wait_for_result( rospy.Duration.from_sec(10.0) )

    print(client.get_state())

    if client.get_state() == GoalStatus.SUCCEEDED:
        print("action succeeded")
        exit(0)
    else:
        print("action failed")
        exit(1)
