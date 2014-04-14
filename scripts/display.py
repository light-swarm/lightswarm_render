#!/usr/bin/env python

import cv2
import cv2.cv as cv

class Display:

    def setup(self, fullscreen):       
        cv2.namedWindow('proj_0', cv2.WINDOW_OPENGL)
        if fullscreen:
            cv2.setWindowProperty('proj_0', cv2.WND_PROP_FULLSCREEN, cv.CV_WINDOW_FULLSCREEN)
        
    def draw(self, image):
        cv2.imshow('proj_0', image)
        cv2.waitKey(1)
        
