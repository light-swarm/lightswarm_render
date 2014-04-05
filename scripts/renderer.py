#!/usr/bin/env python

import rospy
import cv2
import cv2.cv as cv
import numpy as np

import yaml

from lightswarm_core.msg import Penumbras
from lightswarm_core.msg import Shadow
from lightswarm_core.msg import Polygon
from lightswarm_core.msg import Point
from lightswarm_core.msg import World
from lightswarm_core.msg import Boid


WORLD_LIMIT = 100
CONFIG_FILENAME = '/home/ch21133/catkin_ws/src/lightswarm_render/scripts/renderer_config.yaml'


class Renderer:
    def __init__(self):
        self.node_name = 'renderer'
        
        self.read_in_config(CONFIG_FILENAME)
        
        self.create_homograpy()
        
        self.create_displays()

        self.sub_pen = rospy.Subscriber('/penumbras', Penumbras, self.penumbras_callback)
        self.sub_world = rospy.Subscriber('/world', World, self.world_callback2)
 
 
    def read_in_config(self, filename):
        f = open(filename)
        config_map = yaml.safe_load(f)
        f.close()
        
        self.proj_coord = config_map.get('projector_coordinates')
        self.proj_res = config_map.get('projector_resolution')
        
        
    def create_homograpy(self):
        proj_pixels = np.float32([[0, 0], 
            [0, self.proj_res[0]], 
            [self.proj_res[1], self.proj_res[0]], 
            [self.proj_res[1], 0]])
            
        self.homog, mask = cv2.findHomography(np.float32(self.proj_coord), proj_pixels, 0)
        
        
    def create_displays(self):
        cv2.namedWindow('proj_1', cv2.WINDOW_NORMAL)
        #cv2.setWindowProperty('proj_1', cv2.WND_PROP_FULLSCREEN, cv.CV_WINDOW_FULLSCREEN)
        self.reset_image()
        cv2.imshow('proj_1', self.image)
        cv2.waitKey(30)
        
    def reset_image(self):
        self.image = np.zeros([self.proj_res[1], self.proj_res[0], 3], np.uint8)

        
    def penumbras_callback(self, penumbras):
        
        
        
    def world_callback2(self, world):
        rospy.loginfo('got world')
        boid_pix = []
        boid_colors = []
        
        for boid in world.boids:
            poly = self.create_boid_poly([boid.location.y, boid.location.x], boid.theta)
            boid_colors.append(boid.color)
            poly = np.float32([ poly ]).reshape(-1,1,2)
            boid_pix.append(cv2.perspectiveTransform(poly , self.homog))
            
        
        self.update_image2(boid_pix, boid_colors)
        
        self.draw()
        
        
    def create_boid_poly(self, loc, theta):
        theta *= np.pi/180
        y = np.cos(theta)
        x = np.sin(theta)
        
        poly = np.array([ [loc[1]+y, loc[0]+x], 
            [loc[1]-y+0.5*x, loc[0]-x-0.5*y],
            [loc[1]-y-0.5*x, loc[0]-x+0.5*y]], np.float32)
        return poly
        
        
    def update_image2(self, boid_pix, boid_colors):
        self.reset_image()
        for i in range(len(boid_pix)):
            #color = boid_colors[i]
            poly = np.int32(boid_pix[i]).reshape(1,-1,2)
            cv2.fillConvexPoly(self.image, np.fliplr(poly[0]), [255, 255, 255])


        
    def draw(self):
        #cv2.namedWindow('proj_1', cv2.WINDOW_NORMAL)
        #cv2.setWindowProperty('proj_1', cv2.WND_PROP_FULLSCREEN, cv.CV_WINDOW_FULLSCREEN)
        cv2.imshow('proj_1', self.image)
        a = cv2.waitKey(30)
        
        
    def world_callback(self, world):
        rospy.loginfo('got world')
        boid_loc = []
        boid_colors = []
        boid_thetas = []
        
        for boid in world.boids:
            boid_loc.append([boid.location.y, boid.location.x])
            boid_colors.append(boid.color)
            boid_thetas.append(boid.theta)
            
        loc_world = np.float32([ boid_loc ]).reshape(-1,1,2)
        loc_pix = cv2.perspectiveTransform(loc_world , self.homog)
        
        #cv2.namedWindow('proj_1', cv2.WINDOW_NORMAL)
        #cv2.imshow('proj_1', self.image)
        #cv2.waitKey(30)
        
        #cv.NamedWindow('proj_1', 1)
        #cv.ShowImage('proj_1', cv.fromarray(self.image))
        #cv.WaitKey(3)
        
        self.update_image(loc_pix, boid_colors, boid_thetas)
        
        self.draw()
        
        
    def update_image(self, loc_pix, boid_colors, boid_thetas):
        for i in range(len(boid_thetas)):
            loc = loc_pix[i][0]
            #color = boid_colors[i]
            #boid_thetas[i]
            poly = np.array([ [loc[1]+5, loc[0]+5], 
                [loc[1]-5, loc[0]+5],
                [loc[1]-5, loc[0]-5],
                [loc[1]+5, loc[0]-5] ], np.int32)
            print poly
            cv2.fillConvexPoly(self.image, poly, [255, 0, 0])

        


    def test(self):
        im_fname = '/home/ch21133/Downloads/OpenCV/opencv-2.4.8/samples/cpp/baboon.jpg'
        
        #im2 = cv2.imread(im_fname, cv.CV_LOAD_IMAGE_COLOR)
        
        #cv2.imshow('proj_1', im2)
        #cv2.waitKey()
        #cv2.destroyAllWindows()
        
        
        
        #im3 = cv2.perspectiveTransform(im2, self.homog)


def main():
    renderer = Renderer()
    
    rospy.init_node(renderer.node_name, anonymous=True)
    
    rospy.spin()
    

if __name__ == '__main__':
    main()
    
    
    
    
    
