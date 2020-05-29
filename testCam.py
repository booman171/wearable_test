#!pip install sk-video
import cv2
from skvideo.io import vwrite
from skvideo.io import FFmpegWriter
cap = cv2.VideoCapture('Input.mp4')
fps=cap.get(cv2.CAP_PROP_FPS)
W=int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
H=int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
out = FFmpegWriter('Output.avi', 
            inputdict={'-r': str(fps), '-s':'{}x{}'.format(W,H)},
            outputdict={'-r': str(fps), '-c:v': 'libx264', '-preset': 'ultrafast', '-pix_fmt': 'yuv444p'})
while True:
  success , frame = cap.read()
  if success==True:
    out.writeFrame(frame)
  else:
    break
cap.release()
