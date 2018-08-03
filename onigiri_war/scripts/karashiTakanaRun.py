#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import rospkg
import random
import time

from abstractCcr import *
from geometry_msgs.msg import Twist

import numpy as np

class KarashiTakanaBot(AbstractCcr):
    '''
    AbstractCcr を継承
    '''
    def __init__(self, use_lidar=False ,use_camera=False, use_bumper=False, use_opt=False, use_usonic=False, camera_preview=False):
        super(KarashiTakanaBot, self).__init__(use_lidar, use_camera, use_bumper, use_opt, use_usonic, camera_preview)
        self.mode = "choise"
        self.fps = 5
        self.rand_count = 0
        self.mode_count = 0

    def pubTwist(self, x, th):
        twist = Twist()
        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
        self.vel_pub.publish(twist)

    def modeChange(self):
        '''
        choise -> rand
        rand -> choise
        mode_count = 25
        '''
        if self.mode == "choise":
            self.mode = "rand"
        else:
            self.mode = "choise"
        self.mode_count = 20

    def randWalk(self):
        # keep rand value 1sec(5frames)
        if self.rand_count == 0:
            self.value = random.randint(0,3)
            self.rand_count = 3
        else:
            self.rand_count -= 1

        x_th_table = [[0.3,0],[-0.3,0],[0,-1],[0,1]]
        x,th = x_th_table[self.value]
        return x, th

    def markerChoise(self):
        # resize 32*32
        resized = cv2.resize(self.img, (32,32))

        # convert hsv color space
        hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)

        # HSV空間で青色の範囲を定義
        lower_blue = np.array([110,50,50])
        upper_blue = np.array([130,255,255])

        # HSV イメージから青い物体だけを取り出すための閾値
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        sum_mask =  mask.sum()/255
        raw_sum_mask = (mask/255).sum(axis=0)
        raw_sum_conv = np.convolve(raw_sum_mask, np.ones(9), mode='valid')

        # no blue
        if sum_mask < 10:
            x, th = self.randWalk()
        # too much blue
        elif sum_mask > 180:
            x = -0.3
            th = 0
            self.modeChange()
        else:
            max_idx = raw_sum_conv.argmax()
            # at right
            if max_idx < 10:
                th = 0.5
                x = 0.3 
            # at center
            elif max_idx < 14:
                th = 0
                x = 0.3
            # at left
            else:
                th = -0.5
                x = 0.3

        # debug view
        print sum_mask
        print raw_sum_conv
        print x, th
        cv2.imshow("karashi takana view", mask)
        cv2.waitKey(1)

        return x, th

    def strategy(self):
        '''
        main loop
        '''
        r = rospy.Rate(self.fps)
        while not rospy.is_shutdown():
            # check self.img exist
            if self.img is None:
                continue
            # get x, th
            if self.mode == "choise":
                x, th = self.markerChoise()
            else:
                x, th = self.randWalk()

            # count down mode count and change mode
            self.mode_count -= 1
            if self.mode_count < 0:
                self.modeChange()


            # pub teist
            self.pubTwist(x,th)
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('karashitakana')
    bot = KarashiTakanaBot(use_camera=True)
    bot.strategy()

