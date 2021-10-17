#!/usr/bin/python
# -*- coding:utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan # tipo da mensagem do sensor

# Inicia o node
rospy.init_node("rplidar_plot")

# Configura um Subscriber do tópico /laser/scan
scan = LaserScan()

def scan_cb(msg):
    global scan
    scan = msg

scan_sub = rospy.Subscriber("/laser/scan", LaserScan, scan_cb)

# Após a primeira leitura do sensor, printar a medidas imediatamente
# à frente a uma frequência de 10 Hz.
rospy.wait_for_message("/laser/scan", LaserScan)
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    # scan.ranges é um array de 360 medidas de distância
    print(scan.ranges[0], len(scan.ranges))
    rate.sleep()