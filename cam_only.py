
from __future__ import division
from __future__ import print_function
from __future__ import absolute_import

import os
import numpy as np
import cv2
import time
import picamera
import datetime as dt
import csv
from datetime import datetime
import math
import pygame
from pygame.locals import *
import sys
import RPi.GPIO as GPIO
# import imutils

pygame.init()
pygame.mouse.set_visible(False)
pygame.display.set_caption("OpenCV camera stream on Pygame")
screen = pygame.display.set_mode([320,240])

font = pygame.font.SysFont("comicsansms", 72)
text = font.render("Hello, World", True, (0, 128, 0))

os.putenv('SDL_FBDEV', '/dev/fb1')

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(22, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Create accel.csv file to write sensor data to
f = open("accel.csv", "w", newline="")
c=csv.writer(f)

# Set time at start of program
start = time.monotonic()
timestr = time.strftime("%Y%m%d-%H%M%S")
# Set declination based on location http://www.magnetic-declination.com/
declination = -0.106988683

# Set filename video.avi for recording camera output
filename = 'video' + timestr + '.avi' # .avi .mp4

# Set picamera standard frame rate
frames_per_seconds = 30.0

# Set recording resolution
my_res = (640, 480) #'480p' # 1080p

# Read from default (0) camera
cap = cv2.VideoCapture(0)

# Set video format
video_type_cv2 = cv2.VideoWriter_fourcc(*'XVID')

# Save video.avi to current directory
save_path = os.path.join(filename)

# Create video
out = cv2.VideoWriter(save_path, video_type_cv2,frames_per_seconds,  my_res)

# Read in transport image of thermostat icon
overlay_t = cv2.imread('therm.png', -1)


# Define method for overlaying trasnparent images
# copied from https://gist.github.com/clungzta/b4bbb3e2aa0490b0cfcbc042184b0b4e
def overlay_transparent(background_img, img_to_overlay_t, x, y, overlay_size=None):
        """
        @brief      Overlays a transparant PNG onto another image using CV2
        @param      background_img    The background image
        @param      img_to_overlay_t  The transparent image to overlay (has alpha channel)
        @param      x                 x location to place the top-left corner of our overlay
        @param      y                 y location to place the top-left corner of our overlay
        @param      overlay_size      The size to scale our overlay to (tuple), no scaling if None
        @return     Background image with overlay on top
        """
        bg_img = background_img.copy()
        if overlay_size is not None:
                img_to_overlay_t = cv2.resize(img_to_overlay_t.copy(), overlay_size)
	# Extract the alpha mask of the RGBA image, convert to RGB
        b,g,r,a = cv2.split(img_to_overlay_t)
        overlay_color = cv2.merge((b,g,r))

        # Apply some simple filtering to remove edge noise
        mask = cv2.medianBlur(a,5)
        h, w, _ = overlay_color.shape
        roi = bg_img[y:y+h, x:x+w]

        # Black-out the area behind the logo in our original ROI
        img1_bg = cv2.bitwise_and(roi.copy(),roi.copy(),mask = cv2.bitwise_not(mask))

        # Mask out the logo from the logo image.
        img2_fg = cv2.bitwise_and(overlay_color,overlay_color,mask = mask)

        # Update the original image with our new ROI
        bg_img[y:y+h, x:x+w] = cv2.add(img1_bg, img2_fg)
        return bg_img

tag_date = ""

show = True
cam = False
main = True
recording = False
# Video processing
while(True):
    if main == True:
        # set each frame from camera as 'frame'
        ret, frame = cap.read()
        frame1 = frame.copy()
        screen.fill([0,0,0])
        ov = overlay_transparent(frame1, overlay_t, 5, 80, (50,50))
        now = datetime.now()
        opacity = 0.8
        cv2.rectangle(ov,(0,0),(320,240),(51,51,0),cv2.FILLED)
        cv2.putText(ov,now.strftime("%H:%M:%S"),(20,50),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0),2,cv2.LINE_AA)
        cv2.putText(ov,"Exit",(265,220),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0),1,cv2.LINE_AA)
        cv2.putText(ov,"Cam",(255,175),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0),1,cv2.LINE_AA)
        cv2.addWeighted(ov, opacity, frame1, 1 - opacity, 0, frame1)
        ov = cv2.cvtColor(ov, cv2.COLOR_BGR2RGB)
        ov =  np.rot90(ov)
        ov = cv2.flip(ov, 0)
        #overlay = np.flipud(overlay)
        ov= pygame.surfarray.make_surface(ov)
        screen.blit(ov, (0,0))
        pygame.display.update()
        
        if GPIO.input(23) == False:
            main = False
            cam = True
        
        # Break loop
        if GPIO.input(27) == False:
            pygame.quit()
            break
    
    if cam == True:
        # set each frame from camera as 'frame'
        ret, frame = cap.read()
        frame1 = frame.copy()
        screen.fill([0,0,0])
        
        # Create overlay of frame add transparent image at screen coordinates (10, 80)
        #overlay = overlay_transparent(frame, overlay_t, 10, 80, (50,50))
        ov = overlay_transparent(frame1, overlay_t, 5, 80, (50,50))
        
        # Set var now to current date/time
        now = datetime.now()
        
        # Set opacity for overlay transparency, the closer to 0 the more transparent
        opacity = 0.8

        # Overlay date text
        #cv2.putText(overlay,now.strftime("%H:%M:%S"),(30,50),cv2.FONT_HERSHEY_SIMPLEX,0.8,(1, 1, 0),2,cv2.LINE_AA)
        cv2.putText(ov,now.strftime("%H:%M:%S"),(20,50),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0),2,cv2.LINE_AA)
        cv2.putText(ov,"Rec",(255,50),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0),1,cv2.LINE_AA)
        cv2.putText(ov,"Rec-Bio",(245,100),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0),1,cv2.LINE_AA)
        cv2.putText(ov,"Snap",(255,175),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0),1,cv2.LINE_AA)
        cv2.putText(ov,"Menu",(255,230),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0),1,cv2.LINE_AA)
        if recording == True:
	    cv2.circle(ov, (315,5), 2, (255,0,0), cv2.FILLED)

        # Combine overlay to frame, apply transparency
        #cv2.addWeighted(overlay, opacity, frame, 1 - opacity, 0, frame)
        cv2.addWeighted(ov, opacity, frame1, 1 - opacity, 0, frame1)
        #out.write(frame)

        ov = cv2.cvtColor(ov, cv2.COLOR_BGR2RGB)
        ov =  np.rot90(ov)
        ov = cv2.flip(ov, 0)
        ov = pygame.surfarray.make_surface(ov)
        screen.blit(ov, (0,0))
        pygame.display.update()
        
        if GPIO.input(17) == False:
            recording = True
            cv2.circle(ov, (315, 5), 2, (255,0,0), cv2.FILLED)

        if GPIO.input(23) == False:
            filename = "image_" + now.strftime("%H:%M:%S") + ".jpg"
            save_path = os.path.join(filename)
            cv2.imwrite(save_path, frame)
    
        if GPIO.input(27) == False:
            cam = False
            time.sleep(0.5)
            main = True

	# Break loop
    if cv2.waitKey(20) & 0xFF == ord('q'):
        break

