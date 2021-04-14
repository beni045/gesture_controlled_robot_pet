import socket
import time

def send_servo_command(x, sock, UDP_IP, UDP_PORT):

 while True:

  if command != "nothing":

    if x =='u':
        print("Sending: Servo Up")
        sock.sendto(b'servo up\n', (UDP_IP, UDP_PORT))
        data, addr = sock.recvfrom(512)
        print(data)
        x = 'z'
    
    elif x =='d':
        print("Sending: Servo Down")
        sock.sendto(b'servo down\n', (UDP_IP, UDP_PORT))
        data, addr = sock.recvfrom(512)
        x = 'z'
    
    elif x =='l':
        print("Sending: Servo Left")
        sock.sendto(b'servo left\n', (UDP_IP, UDP_PORT))
        data, addr = sock.recvfrom(512)
        x = 'z'
    
    elif x =='r':
        print("Sending: Servo Right")
        sock.sendto(b'servo right\n', (UDP_IP, UDP_PORT))
        data, addr = sock.recvfrom(512)
        x = 'z'
    
    elif x =='e':
        print("Sending: Exit")
        sock.sendto(b'exit\n', (UDP_IP, UDP_PORT))
        data, addr = sock.recvfrom(512)
        x = 'z'
    
    else:
        print("wrong input")

  command = "nothing"
    
