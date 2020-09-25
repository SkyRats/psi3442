#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
from aula_pkg.msg import control
import cv2
import numpy as np
class Follower(object):
    def __init__(self):
        # Node subscreve no topico em que sao recebidas as imagens fa camera frontal do drone
        self.image_sub = rospy.Subscriber("/iris/usb_cam/image_raw", Image, self.camera_callback, queue_size=10)
        
        
        self.bridge_object = CvBridge()

        # Node passa a publicar no topico "control_topic" com o tipo de mensagem control
        self.pub = rospy.Publisher('control_topic', control, queue_size=10)
    def camera_callback(self,data):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        print("Imagem Carregada")
        height,width,channels = cv_image.shape
        descentre = 160
        rows_to_watch = 60
        #crop_image = cv_image[(height)/2+descentre:(height)/2+(descentre+rows_to_watch)][1:width]
        hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
        lowerb = np.array([0, 70, 50])
        upperb = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lowerb, upperb)
        #cv2.imshow('mask',mask)
        #cv2.waitKey(0)
        m = cv2.moments(mask, False)
        try:
            cx,cy = m["m10"]/m["m00"],m["m01"]/m["m00"]
        except ZeroDivisionError:
            cy,cx = height/2,width/2

        print(cx)
        
        # Cria a mensagem e seta os valores
        public = control()
        public.cx = cx
        public.width = width

        # Publica a mensagem
        self.pub.publish(public)

def main():
    # Inicia no node
    rospy.init_node('detection_node', anonymous = True)
    follower_object = Follower()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")



if __name__=="__main__":
    main()
