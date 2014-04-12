#!/usr/bin/env python

import rospy
import cv2
import cv2.cv as cv
import numpy as np

class Display:

    def setup(self, fullscreen):       
        cv2.namedWindow('proj_0', cv2.WINDOW_NORMAL)
        if fullscreen:
            cv2.setWindowProperty(self.proj_names[i], cv2.WND_PROP_FULLSCREEN, cv.CV_WINDOW_FULLSCREEN)
        
    def draw(self, image):
        cv2.imshow('proj_0', image)
        cv2.waitKey(1)
        
