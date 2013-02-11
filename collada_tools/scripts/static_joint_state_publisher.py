#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest("hrpsys_gazebo")
import rospy
from sensor_msgs.msg import JointState
from optparse import OptionParser

if __name__ == "__main__":
    parser = OptionParser(description='static_joint_state_publisher')

    parser.add_option('--joints', action='store', type='string', dest='joints', default=None,
                      help='')
    parser.add_option('--rate', action='store', type='float', dest='rate', default=100.0,
                      help='')
    parser.add_option('--position', action='store', type='string', dest='position', default=None,
                      help='')
    parser.add_option('--velocity', action='store', type='string', dest='velocity', default=None,
                      help='')
    parser.add_option('--effort', action='store', type='string', dest='effort', default=None,
                      help='')
    parser.add_option('--topic', action='store', type='string', dest='topicname', default='joint_states',
                      help='')

    (options, args) = parser.parse_args()

    if options.joints is not None:
        jlist = options.joints.split(',')
    else:
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

    # ROSノードの準備
    rospy.init_node("joint_state_publisher")
    publisher = rospy.Publisher(options.topicname, JointState)
    rate = rospy.Rate(options.rate)

    # JointStateの準備
    joint_state = JointState()

    for i in range(len(jlist)):
        joint_state.name.append(jlist[i])
        if plist is not None and len(plist) > i:
            joint_state.position.append(float(plist[i]))
        if vlist is not None and len(vlist) > i:
            joint_state.velocity.append(float(vlist[i]))
        if elist is not None and len(elist) > i:
            joint_state.effort.append(float(elist[i]))
        if (plist is None) and (vlist is None) and (elist is None):
            joint_state.position.append(0)

    # ROSループ
    while not rospy.is_shutdown():
        # 現在時刻をstampに入れる
        joint_state.header.stamp = rospy.Time.now()
        publisher.publish(joint_state)
        rate.sleep()
