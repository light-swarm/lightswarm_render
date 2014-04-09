#!/usr/bin/env python

import rospy
import cv2
import cv2.cv as cv
import numpy as np
import shapely

import yaml

from lightswarm_core.msg import Penumbras
from lightswarm_core.msg import Shadow
from lightswarm_core.msg import Polygon
from lightswarm_core.msg import Point
from lightswarm_core.msg import World
from lightswarm_core.msg import Boid

from display import Display


CONFIG_FILENAME = 'src/lightswarm_core/params/config.yaml'


class Renderer:
    def __init__(self):
        self.node_name = 'renderer'
        
        config_fname = rospy.get_param('config_file', CONFIG_FILENAME)
        self.read_in_config(config_fname)
        self.create_homograpy()
        self.create_displays2()

        self.sub_pen = rospy.Subscriber('/penumbras', Penumbras, self.penumbras_callback)
        self.sub_world = rospy.Subscriber('/world', World, self.world_callback2)
 
 
    def read_in_config(self, filename):
        f = open(filename)
        config_map = yaml.safe_load(f)
        f.close()
        
        self.num_proj = config_map.get('number_of_projectors')
        self.proj_coord = config_map.get('projector_coordinates')
        self.proj_res = config_map.get('projector_resolution')
        
        
    def create_homograpy(self):
        proj_pixels = np.float32([[0, 0], 
            [0, self.proj_res[0]], 
            [self.proj_res[1], self.proj_res[0]], 
            [self.proj_res[1], 0]])
        
        self.homog = []
        for i in range(self.num_proj):
            temp_homog, mask = cv2.findHomography(np.float32(self.proj_coord[i]), proj_pixels, 0)
            self.homog.append(temp_homog)


    def create_displays2(self):
        self.reset_image()
        self.shadows = []
        
        self.display = Display()
        self.display.setup()
        self.draw()
        
        
    def reset_image(self):
        self.image = []
        for i in range(self.num_proj):
            self.image.append(np.zeros([self.proj_res[1], self.proj_res[0], 3], np.uint8))

        
    def penumbras_callback(self, penumbras):
        
        self.shadows = []
        for shadow in penumbras.projector_shadows:
            poly = []
            for point in shadow.polygon.points:
                poly.append([point.y, point.x])
            self.shadows.append(poly)
        
        
    def world_callback2(self, world):
        rospy.loginfo('got world')
        boid_pix0 = []
        boid_pix1 = []
        boid_colors = []
        
        for boid in world.boids:
            poly = self.create_boid_poly([boid.location.y, boid.location.x], boid.theta)
            boid_colors.append(boid.color)
            poly = np.float32([ poly ]).reshape(-1,1,2)
            boid_pix0.append(cv2.perspectiveTransform(poly, self.homog[0]))
            boid_pix1.append(cv2.perspectiveTransform(poly, self.homog[1])) 
        
        self.reset_image()
        self.update_image2(boid_pix0, boid_colors, 0)
        self.update_image2(boid_pix1, boid_colors, 1)
        self.draw()
        
        
    def create_boid_poly(self, loc, theta):
        theta *= np.pi/180
        y = np.cos(theta)
        x = np.sin(theta)
        
        poly = np.array([ [loc[1]+y, loc[0]+x], 
            [loc[1]-y+0.5*x, loc[0]-x-0.5*y],
            [loc[1]-y-0.5*x, loc[0]-x+0.5*y]], np.float32)
        return poly
        
        
    def update_image2(self, boid_pix, boid_colors, proj_idx):
        
        for i in range(len(boid_pix)):
            poly = np.int32(boid_pix[i]).reshape(1,-1,2)
            cv2.fillConvexPoly(self.image[proj_idx], np.fliplr(poly[0]), boid_colors[i])
            
        for poly in self.shadows:
            poly = np.float32([ poly ]).reshape(-1,1,2)
            pix = cv2.perspectiveTransform(poly, self.homog[proj_idx])
            pix = np.int32(pix).reshape(1,-1,2)
            cv2.fillConvexPoly(self.image[proj_idx], np.fliplr(pix[0]), [0, 0, 0])

    def draw(self):
        combined_image = self.combine_image(self.image[0], self.image[1])
        combined_image = self.combine_image(combined_image, self.image[1])
        self.display.draw(combined_image)
            
    def combine_image(self, image1, image2):
        return np.hstack((image1, image2))

    def create_displays_old(self):
        cv2.namedWindow('proj_1', cv2.WINDOW_NORMAL)
        #cv2.setWindowProperty('proj_1', cv2.WND_PROP_FULLSCREEN, cv.CV_WINDOW_FULLSCREEN)
        self.reset_image()
        self.shadows = []
        self.draw()
        
    def draw_old(self):
        cv2.imshow('proj_1', self.image)
        a = cv2.waitKey(30)
        


def main():
    renderer = Renderer()
    
    rospy.init_node(renderer.node_name, anonymous=True)
    
    rospy.spin()
    

if __name__ == '__main__':
    main()
    
    
    
    
    
