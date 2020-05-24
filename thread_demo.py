import argparse
import os
import cv2
from get_video import VideoGet
from flask import Flask, render_template, Response

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

ds_factor = 0.6

#global frame
def threadVideoGet(source=0):
	'''
	Thread for getting video from get_video
	'''
	video_getter = VideoGet(source).start()
	while True:
		if (cv2.waitKey(1) == ord("q")) or video_getter.stopped:
			video_getter.stop()
			break
		frame = video_getter.frame
		frame=cv2.resize(frame,None,fx=ds_factor,fy=ds_factor,interpolation=cv2.INTER_AREA)
		gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
		ret, jpeg = cv2.imencode('.jpg', frame)
		#cv2.imshow("Video", frame)
		jpg = jpeg.tobytes()
		yield (b'--frame\r\n'b'Content-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n\r\n')


@app.route('/video_feed')
def video_feed():
	return Response(threadVideoGet(), mimetype='multipart/x-mixed-replace; boundary=frame')



while True:
	app.run(host='192.168.1.4', debug=True)


