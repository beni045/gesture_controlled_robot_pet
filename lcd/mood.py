import os
import time
import sys
import serial

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

setup()
time.sleep(5)
happy()
time.sleep(10)
countdown()
time.sleep(10)
disconnect()
