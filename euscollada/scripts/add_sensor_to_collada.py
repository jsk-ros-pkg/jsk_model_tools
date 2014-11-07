#!/usr/bin/env python

import sys, os
from xml.dom.minidom import parse, parseString
import xml.dom
import yaml
import argparse

reload(sys)
sys.setdefaultencoding('utf-8')

from parseColladaBase import parseColladaSensor

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='add_sensor_to_collada')
    parser.add_argument('filename', nargs=1)
    parser.add_argument('-O', '--output', help='output filename')
    parser.add_argument('-C', '--config', help='config filename (yaml file)')
    args = parser.parse_args()

    obj = parseColladaSensor()
    if obj.init(args.filename[0]):
        if args.config:
            data = yaml.load(open(args.config).read())
            for sensor in data['sensors']:
                obj.add_sensor(sensor['sensor_name'],
                               sensor['parent_link'],
                               sensor['sensor_type'],
                               translate = sensor['translate'] if sensor.has_key('translate') else None,
                               rotate = sensor['rotate'] if sensor.has_key('rotate') else None)
        else:
            ## sample
            obj.add_sensor('lhsensor', 'l_hand', 'force')
            obj.add_sensor('rhsensor', 'r_hand', 'force')
            obj.add_sensor('lfsensor', 'l_foot', 'force')
            obj.add_sensor('rfsensor', 'r_foot', 'force')
            obj.add_sensor('gsensor', 'imu_link', 'acceleration')
            obj.add_sensor('gyrometer', 'imu_link', 'gyro')

        if args.output:
            f = open(args.output, 'wb')
            obj.writeDocument(f)
            f.close()
        else:
            obj.writeDocument(sys.stdout) ## wirting to standard output

## sample yaml for sensors
#sensors:
#  - {sensor_name: 'lhsensor',  sensor_type: 'force', parent_link: 'LARM_LINK6', translate: '0 0 0', rotate: '1 0 0 0'}
#  - {sensor_name: 'gsensor',   sensor_type: 'acceleration', parent_link: 'BODY'}
#  - {sensor_name: 'gyrometer', sensor_type: 'gyro', parent_link: 'BODY'}
