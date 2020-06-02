from threading import Thread
import cv2
import time

class VideoGet:
	'''
	Class will continuosly get frames  within a dedicated thread
	'''

	def __init__(self, src=0):
		self.stream = cv2.VideoCapture(src)
		(self.grabbed, self.frame) = self.stream.read()
		self.stopped = False

	def start(self):
		Thread(target=self.get, args=()).start()
		return self

	def get(self):
		self.timestr = time.strftime("%Y%m%d-%H%M%S")
		self.filename = 'video' + self.timestr + '.avi' # .avi .mp4
		self.fps = 15.0
		self.video_writer = cv2.VideoWriter_fourcc('M','J','P','G')
		self.video_out = cv2.VideoWriter(self.filename, self.video_writer, self.fps, (640, 480))

		while not self.stopped:
			if not self.grabbed:
				self.stop()
			else:
				(self.grabbed, self.frame) = self.stream.read()
				self.video_out.write(self.frame)
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break

	def stop(self):
		self.stopped = True
