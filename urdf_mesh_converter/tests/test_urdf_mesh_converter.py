#!/usr/bin/env python

import os.path as osp
import subprocess
import unittest

import rospkg
import rospy


PKG = 'urdf_mesh_converter'
NAME = 'test_urdf_mesh_converter'


class TestURDFMeshConverter(unittest.TestCase):

    def _check_command(self, cmd):
        proc = subprocess.Popen(cmd, shell=True,
                                stdout=subprocess.PIPE)
        with proc.stdout:
            for line in iter(proc.stdout.readline, b''):
                rospy.loginfo('{}'.format(line.decode('utf-8').strip()))
        returncode = proc.wait()

        if returncode != 0:
            raise RuntimeError('command {} failed.'.format(cmd))

    def test_merge(self):
        rospack = rospkg.RosPack()
        path = rospack.get_path('urdf_mesh_converter')

        output_path = osp.join(
            path, 'tests', 'output',
            'pitch_yaw_2axis_camera_module_modified.urdf')
        input_urdf_path = osp.join(path, 'samples',
                                  'pitch_yaw_2axis_camera_module.urdf')
        cmd = 'rosrun urdf_mesh_converter convert_urdf_mesh.py {} -o {}'\
            .format(input_urdf_path, output_path)
        self._check_command(cmd)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestURDFMeshConverter)
