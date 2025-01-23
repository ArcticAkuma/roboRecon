import cv2
import numpy as np
from numpy import mean
from simple_pid import PID
import logging
import time

logger = logging.getLogger(__name__)

class AutoDrive:
    def __init__(self, pid, cfg):
        self.overlay_image = cfg.OVERLAY_IMAGE
        self.target_pixel = cfg.TARGET_PIXEL  # center point of the cam img used to determine distance from objects/walls
        self.target_threshold = cfg.TARGET_THRESHOLD # minimum distance from target_pixels before a steering change is made
        self.steering = 0.0 # from -1 to 1
        self.throttle = cfg.THROTTLE_INITIAL # from -1 to 1
        self.pid_st = pid

    def run(self, cam_depth_img):
        '''
        main runloop of the CV controller
        input: depth_image, an numpy array.
        output: steering, throttle, and the image.
        If overlay_image is True, then the output image
        includes and overlay that shows how the
        algorithm is working; otherwise the image
        is just passed-through untouched.
        '''
        if cam_depth_img is None:
            return 0, 0, False, None

        def Image_slicing (cam_depth_img):
            '''
            Slicing the immage along the target_pixel to get a right and a left half,
            used to determine the direction of sterring, by determinig which side
            has a higher average distance.
            '''

            # left slice of the image
            left_img = cam_depth_img[:, :self.target_pixel[0]]
            # right slice of the image
            right_img = cam_depth_img [:, self.target_pixel[0]:]
            # which index of the range has the higher average distance?
            left_distance = mean(left_img)
            right_distance = mean(right_img)

            # return fuction
            if left_distance > right_distance :
                return -1.0 # = steering left
            else:
                return 1.0 # = steering right

        distance = mean (cam_depth_img [:, 404 : 444])
        distance_l = mean(cam_depth_img [:, 264 : 384])
        distance_r = mean(cam_depth_img [:, 484 : 564])
        if distance < 1000 or distance_l < 1000 or distance_r < 1000:
            self.throttle = -1
            time.sleep(1) # drive back for 2 seconds
        else:
            self.throttle = 0.7

        # Steering when distance is too low
        if distance < self.target_threshold and distance > 1000:
            self.throttle = 0.5
            self.steering = Image_slicing(cam_depth_img)

        else :
            #determine if the car can fit through gaps if center is free
            distance_l = mean(cam_depth_img [:, 284 : 404])
            distance_r = mean(cam_depth_img [:, 484 : 564])
            self.steering = 0.0
            if (distance_r < self.target_threshold or distance_l < self.target_threshold) and distance_l > 500 and distance_r > 500:
               self.throttle = 0.5
               self.steering = Image_slicing(cam_depth_img)


        # show some diagnostics
        if self.overlay_image:
            cam_depth_img = self.overlay_display(cam_depth_img,distance)

        return self.steering, self.throttle, cam_depth_img

    def overlay_display(self, cam_depth_img,distance):
        '''
        composite mask on top the original image.
        show some values we are using for control
        '''
        img = np.copy(cam_depth_img)
        # img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        display_str = []
        display_str.append("STEERING:{:.1f}".format(self.steering))
        display_str.append("THROTTLE:{:.2f}".format(self.throttle))
        display_str.append("I DISTANCE:{:.2f}".format(distance))

        y = 10
        x = 10

        for s in display_str:
            cv2.putText(img, s, color=(255, 0, 0), org=(x ,y), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.4)
            y += 10

        return img