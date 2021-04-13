from hello_world import *
from Car_movement import *
import time
import RPi.GPIO as GPIO     
import socket


in1 = 23
in2 = 24
en = 25
in3 = 17
in4 = 27 
en2 = 22
temp1=1

GPIO.setmode(GPIO.BCM)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(en,GPIO.OUT)
GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)
GPIO.setup(en2,GPIO.OUT)
GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)
GPIO.output(in3,GPIO.LOW)
GPIO.output(in4,GPIO.LOW)
p=GPIO.PWM(en,1000)
p.start(75)
p2=GPIO.PWM(en2,1000)
p2.start(75)


UDP_IP = "0.0.0.0" # listen to everything
UDP_PORT = 12345 # port

sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

buffer = []

while True:
    data, addr = sock.recvfrom(512) # random buffer size, doesn't matter here..
    print("received message:", data)
    buffer.append(data)
    print(len(buffer))
    
    if len(buffer) ==8:
        same = all(elem == buffer[0] for elem in buffer)
        
        if same:
            command = buffer[0]
            print("same. command:", command)
            
            if command==b'forward\n':
                forward()
                print("Go Forward")
            elif command==b'backward\n':
                backward()
                print("Go Backward")
            elif command==b'left\n':
                left()
                print("Go Left")
            elif command==b'right\n':
                right()
                print("Go Right") 
            
            buffer.clear()
        else:
            print("not same")
            buffer.clear()       
    
    