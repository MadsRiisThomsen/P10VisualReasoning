#!/usr/bin/env python3
import rospy
from typing import List
import numpy as np
from PIL import Image
import cv2 as cv


class AssemblyAssistance:
    def __init__(self):
        var = 0


    def scale_img(self, image):
        scale_percent = 40 # percent of original size
        width = int(image.shape[1] * scale_percent / 100)
        height = int(image.shape[0] * scale_percent / 100)
        dim = (width, height)
  
        # resize image
        resized = cv.resize(image, dim, interpolation = cv.INTER_AREA)
        return resized


    def start_help(self, image):

        image_b = image
        image_g = image
        image_w = image

        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

        #ranges for blue
        lower_blue = np.array([110,50,50])
        upper_blue = np.array([130,255,255])

        #ranges for green
        lower_green = np.array([25,55,72])
        upper_green = np.array([102,255,255])

        #ranges for white, adjust sensitivity
        sensitivity = 70
        lower_white = np.array([0,0,255-sensitivity])
        upper_white = np.array([255,sensitivity,255])

        #create masks for all colors
        mask_blue = cv.inRange(hsv, lower_blue, upper_blue)
        mask_green = cv.inRange(hsv, lower_green, upper_green)
        mask_white = cv.inRange(hsv, lower_white, upper_white)

        # Remove unnecessary noise from mask
        kernel = np.ones((7,7),np.uint8)
        mask_blue = cv.morphologyEx(mask_blue, cv.MORPH_CLOSE, kernel)
        mask_blue = cv.morphologyEx(mask_blue, cv.MORPH_OPEN, kernel)
        mask_green = cv.morphologyEx(mask_green, cv.MORPH_CLOSE, kernel)
        mask_green = cv.morphologyEx(mask_green, cv.MORPH_OPEN, kernel)
        mask_white = cv.morphologyEx(mask_white, cv.MORPH_CLOSE, kernel)
        mask_white = cv.morphologyEx(mask_white, cv.MORPH_OPEN, kernel)

        # Segmentation
        segmented_img_green = cv.bitwise_and(image_g, image_g, mask=mask_green)
        segmented_img_blue = cv.bitwise_and(image_b, image_b, mask=mask_blue)
        segmented_img_white = cv.bitwise_and(image_w, image_w, mask=mask_white)

        #get the contours of each object
        contours_green, hierarchy = cv.findContours(mask_green.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        contours_blue, hierarchy = cv.findContours(mask_blue.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        contours_white, hierarchy = cv.findContours(mask_white.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)


        #find the biggest countour by the area
        try:
            area_blue = max(contours_blue, key = cv.contourArea)
            area_blue = cv.contourArea(area_blue)
        except ValueError:
            area_blue = 0
        
        try:
            area_green = max(contours_green, key = cv.contourArea)
            area_green = cv.contourArea(area_green)
        except ValueError:
            area_green = 0        

        try:
            area_white = max(contours_white, key = cv.contourArea)
            area_white = cv.contourArea(area_white)
        except ValueError:
            area_white = 0

        print("BLUE ARE")
        print(area_blue)
        print("GREEN AREA")
        print(area_green)
        print("WHITE AREA")
        print(area_white)


        if area_blue > area_green and area_blue > area_white:
            return "Blue is biggest, we are at step 1"
        elif area_green > area_blue and area_green > area_white:
            return "Green is biggest, we are at step 2"
        elif area_white > area_blue and area_white > area_green:
            return "White is biggest, the assembly is done. No assistance needed."
        else:
            return "No assembly needed, we are at step 0"

        """
        #output_green = cv.drawContours(segmented_img_green, area_green, -1, (0, 0, 255), 3)
        output_blue = cv.drawContours(segmented_img_blue, area_blue, -1, (0, 0, 255), 3)
        output_white = cv.drawContours(segmented_img_white, area_white, -1, (0, 0, 255), 3)

        # Showing the output
        #cv.imshow("Output_green", self.scale_img(output_green))
        #cv.waitKey(0)
        cv.imshow("Output_blue", self.scale_img(output_blue))
        cv.waitKey(0)
        cv.imshow("Output_white", self.scale_img(output_white))
        cv.waitKey(0)
        """

        




if __name__ == "__main__":
    ass = AssemblyAssistance()
    bgr_img = cv.imread('/home/mads/Desktop/yep_ws/src/P10VisualReasoning/assembly_assistance/scripts/blue.jpg', cv.IMREAD_COLOR)

    assembly_step = ass.start_help(bgr_img)
    print(assembly_step)

