import os
import time
import sys
import serial
import cv2
import glob

#define countdown parameters
font = cv2.FONT_HERSHEY_SIMPLEX
scale = 5
color = (0, 0, 255)
thickness = 10

def setup():
	os.system('fbcp &')
	os.system('sudo fbi -T 2 -d /dev/fb0 -noverbose -a ~/white.jpeg')
	os.system('screen -d -m omxplayer --no-osd --loop ~/neu.mp4')

def happy():
#	os.system('killall omxplayer')
	os.system('omxplayer ~/happy.mp4')
#	os.system('screen -d -m omxplayer --no-osd --loop ~/neu.mp4')

def countdown():
	os.system('killall omxplayer')
	os.system('omxplayer ~/cd.mp4')
	os.system('screen -d -m omxplayer --no-osd --loop ~/neu.mp4')

def disconnect():
	os.system('killall omxplayer')
	os.system('killall fbcp')

def cameraview_setup():
	os.system('killall omxplayer')
	time.sleep(1)

def cameraview(image, j):
	resized = cv2.resize(img, (480,320), interpolation = cv2.INTER_AREA)

	#if j%3==0:
	cv2.putText(resized,str(j//3),(200,210),font, scale,color,thickness,cv2.LINE_AA)
	#cv2.namedWindow('test', cv2.WINDOW_NORMAL)
	#cv2.imshow('test', resized)
	cv2.imwrite('1m.jpg',resized)
	os.system('sudo fbi -T 2 -d /dev/fb0 -noverbose -a ~/1m.jpg')

Def cameraview_close():
	os.system('screen -d -m omxplayer --no-osd --loop ~/neu.mp4')

setup()
time.sleep(5)
#happy()
#time.sleep(10)
#countdown()

#currently using images from file
files = glob.glob("/home/pi/photos/*.jpg")
files = sorted(files)
cnt=11

cameraview_setup()

for i in files:
	img = cv2.imread(i)
	cameraview(img, cnt)
	cnt=cnt-1

cameraview_close()

time.sleep(10)
disconnect()
