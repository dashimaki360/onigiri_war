#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import rospkg
import random
import time

from abstractCcr import *
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import numpy as np

class KarashiTakanaBot(AbstractCcr):
    '''
    AbstractCcr を継承
    '''
    def __init__(self, use_lidar=True ,use_camera=True, use_bumper=False, use_opt=True, use_usonic=True, camera_preview=False):
        super(KarashiTakanaBot, self).__init__(use_lidar, use_camera, use_bumper, use_opt, use_usonic, camera_preview)
        self.target_id_sub = rospy.Subscriber('target_id', String, self.targetIdCallback)
        self.mode = "chaise"
        self.fps = 5
        self.rand_count = 0
        self.mode_count = 999

    def targetIdCallback(self, data):
        self.mode = "rand"
        self.mode_count = 15
        print "### Get target ID ###"

    def pubTwist(self, x, th):
        twist = Twist()
        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
        self.vel_pub.publish(twist)

    def modeChange(self):
        '''
        if get_target
            chaise -> rand
        if time_out and found target
          rand -> chaise
          mode_count = 15
        '''
        if self.mode == "chaise":
            self.mode = "rand"
            self.mode_count = 15
        else:
            self.mode = "chaise"
            self.mode_count = 999

    def randWalk(self):
        # if infront of space gostraight
        if min( self.scan[:20] + self.scan[-20:] ) > 0.7:
            return 0.3, 0

        # keep rand value 1sec(5frames)
        x_th_table = [ [0.3,-1], [0.3, 1], [0,1], [0,-1] ]
        if self.rand_count == 0:
            self.value = random.randint(0,len(x_th_table)-1 )
            self.rand_count = 5
        else:
            self.rand_count -= 1

        x,th = x_th_table[self.value]
        return x, th

    def markerchaise(self):
        # resize 32*32
        resized = cv2.resize(self.img, (64,64))

        # convert hsv color space
        hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)

        lower_blue = np.array([110,50,50])
        upper_blue = np.array([130,255,255])

        lower_green = np.array([50,50,50])
        upper_green = np.array([70,255,255])

        # blue
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        sum_mask =  mask.sum()/255
        raw_sum_mask = (mask/255).sum(axis=0)
        raw_sum_conv = np.convolve(raw_sum_mask, np.ones(9), mode='valid')

        # green
        g_mask = cv2.inRange(hsv, lower_green, upper_green)
        g_sum_mask =  g_mask.sum()/255
        g_raw_sum_mask = (g_mask/255).sum(axis=0)
        g_raw_sum_conv = np.convolve(g_raw_sum_mask, np.ones(9), mode='valid')

        # detect green
        if g_sum_mask > 10:
            max_idx = g_raw_sum_conv.argmax()
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
        # no blue
        elif sum_mask < 30:
            x, th = self.randWalk()
            #x  = 0
            #th = 0.5
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
        '''
        print sum_mask
        print raw_sum_conv
        print x, th
        '''
        mask_concat = cv2.vconcat([mask, g_mask])
        cv2.imshow("karashi takana view", mask_concat)
        cv2.waitKey(1)

        return x, th
    def nearWall(self):
        if min(self.usonic) < 0.07 or min(self.opt) < 0.07:
            return True
        if len(self.scan) == 360 and min( self.scan[:20] + self.scan[-20:] ) < 0.3:
            return True
        return False


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
            # near wall
            if self.nearWall():
                x, th = -0.3, 0.5
                self.rand_count = 0
            elif self.mode == "chaise":
                x, th = self.markerchaise()
            else:
                x, th = self.randWalk()

            # count down mode count and change mode
            self.mode_count -= 1
            if self.mode_count < 0:
                self.modeChange()

            # pub teist
            self.pubTwist(x,th)
            r.sleep()

            # debug
            print min(self.usonic) , min(self.opt), self.mode
            print self.nearWall(), min( self.scan[:20] + self.scan[-20:] )


if __name__ == '__main__':
    rospy.init_node('karashitakana')
    bot = KarashiTakanaBot(use_camera=True)
    bot.strategy()

