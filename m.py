import cv2

capture = cv2.VideoCapture(0)
frame_width = int(capture.get(3))
frame_height = int(capture.get(4))

print("Height: " + str( frame_height) + ", Width: " + str(frame_width) + ", Frame Rate: " + str(capture.get(cv2.CAP_PROP_FPS)))
