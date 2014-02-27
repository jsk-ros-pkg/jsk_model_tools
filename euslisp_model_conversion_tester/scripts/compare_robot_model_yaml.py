#!/usr/bin/python

import yaml
import types

#EPSILON = 1e-6
EPSILON = 1e-5

def get_yaml_data (fname):
    return yaml.load(open(fname).read());

def eps_equal (v1, v2, eps=EPSILON):
    return abs(v1 - v2) < eps;

def list_eps_equal (v1, v2, eps=EPSILON):
    return all(map (lambda x, y : eps_equal (x,y, eps), v1, v2))

def check_var (t1, t2, prop_name, eps=EPSILON):
    if type(t1.get(prop_name)) == types.StringType:
        v1 = map (lambda x : float(x), t1.get(prop_name).split(','))
        func = list_eps_equal
    else:
        v1 = t1.get(prop_name)
        func = eps_equal
    if type(t2.get(prop_name)) == types.StringType:
        v2 = map (lambda x : float(x), t2.get(prop_name).split(','))
    else:
        v2 = t2.get(prop_name)
    if v1 == None and v2 == None:
        print "Neglected"
        return True
    elif v1 != None and v2 != None:
        if func(v1, v2, eps):
            print "OK"
            return True;
        else:
            print "NG, check fail ", v1, " vs ", v2
            return False;
    else:
        print "NG, data missing among ", v1, " ", v2
        return False;

def check_str_var (t1, t2, prop_name):
    v1 = t1.get(prop_name)
    v2 = t2.get(prop_name)
    if v1 and v2:
        if v1 == v2:
            print "OK"
            return True;
        else:
            print "NG", v1, v2
            return False;
    else:
        print None
        return True

import unittest
class TestRobotModelYaml(unittest.TestCase):
    def check_var(self, prop_name, param_name='Links', check_func=check_var):
        print "  check", param_name
        ret = []
        for f in test2.get(param_name).keys():
            t2 = test2.get(param_name).get(f)
            t1 = test1.get(param_name).get(f)
            print "    ", f, prop_name,
            if t1 and t2:
                ret.append(check_func(t1, t2, prop_name))
            else:
                print None
        return all(ret)

    # def test_total_joints (self):
    #     self.assertTrue(check_str_var(test1.get('total'), test2.get('total'), 'joints'))
    # def test_total_actjoints (self):
    #     self.assertTrue(check_str_var(test1.get('total'), test2.get('total'), 'actjoints'))
    def test_Total_Mass (self):
        print 'check Total'
        print '  Mass ',
        self.assertTrue(check_var(test1.get('Total'), test2.get('Total'), 'Mass', 1e-3))
    def test_Total_GlobalCom (self):
        print 'check Total'
        print '  GlobalCom ',
        self.assertTrue(check_var(test1.get('Total'), test2.get('Total'), 'GlobalCom'))
    def test_Total_GlobalInertia (self):
        print 'check Total'
        print '  GlobalInertia ',
        self.assertTrue(check_var(test1.get('Total'), test2.get('Total'), 'GlobalInertia', 1e-4))
    def test_Link_Mass (self):
        self.assertTrue(self.check_var('Mass'))
    def test_Link_LocalCom (self):
        self.assertTrue(self.check_var('LocalCom'))
    def test_Link_GlobalCom (self):
        self.assertTrue(self.check_var('GlobalCom'))
    def test_Link_ComNorm (self):
        self.assertTrue(self.check_var('ComNorm'))
    def test_Link_LocalInertia (self):
        self.assertTrue(self.check_var('LocalInertia'))
    def test_Link_GlobalInertia (self):
        self.assertTrue(self.check_var('GlobalInertia'))
    def test_Link_InertiaNorm (self):
        self.assertTrue(self.check_var('InertiaNorm'))
    def test_Link_LocalPos (self):
        self.assertTrue(self.check_var('LocalPos'))
    def test_Link_LocalRot (self):
        self.assertTrue(self.check_var('LocalRot'))
    def test_Link_GlobalPos (self):
        self.assertTrue(self.check_var('GlobalPos'))
    def test_Link_GlobalRot (self):
        self.assertTrue(self.check_var('GlobalRot'))
    def test_Joint_LowerLimit (self):
        self.assertTrue(self.check_var('LowerLimit', 'Joints'))
    def test_Joint_UpperLimit (self):
        self.assertTrue(self.check_var('UpperLimit', 'Joints'))
    def test_Joint_VelocityLimit (self):
        self.assertTrue(self.check_var('VelocityLimit', 'Joints'))
    def test_Sensor_LocalPos (self):
        self.assertTrue(self.check_var('LocalPos', 'Sensors'))
    def test_Sensor_LocalRot (self):
        self.assertTrue(self.check_var('LocalRot', 'Sensors'))
    def test_Sensor_GlobalPos (self):
        self.assertTrue(self.check_var('GlobalPos', 'Sensors'))
    def test_Sensor_GlobalRot (self):
        self.assertTrue(self.check_var('GlobalRot', 'Sensors'))

if __name__ == '__main__':
    import sys
    import rostest
    global test1, test2
    test1=get_yaml_data(sys.argv[1])
    test2=get_yaml_data(sys.argv[2])
    str1=sys.argv[1].split('/')[-1].split('.')[0].replace('_robot_model', '')
    str2=sys.argv[2].split('/')[-1].split('.')[0].replace('_robot_model', '')
    rostest.unitrun('euslisp_model_conversion_tester', 'test_robot_model_yaml_'+str1+"_"+str2, TestRobotModelYaml)

