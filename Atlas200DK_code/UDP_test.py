import socket
import time

UDP_IP = "192.168.0.4" # set it to destination IP.. RPi in this case
UDP_PORT = 12345

print("UDP target IP:", UDP_IP)
print("UDP target port:", UDP_PORT)

sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

# print("Sending: Go Forward")
# sock.sendto(b'forward\n', (UDP_IP, UDP_PORT))
# time.sleep(10)
# print("Sending: Go Backward")
# sock.sendto(b'backward\n', (UDP_IP, UDP_PORT))
# time.sleep(10)
# print("Sending: Exit")
# sock.sendto(b'exit\n', (UDP_IP, UDP_PORT))
print("f-forward b-backward l-left r-right e-exit \n")


while(1):
    
    x = raw_input("please enter the command: ")

    if x =='f':
        print("Sending: Go Forward")
        sock.sendto(b'forward\n', (UDP_IP, UDP_PORT))
        x = 'z'
    
    elif x =='b':
        print("Sending: Go Backward")
        sock.sendto(b'backward\n', (UDP_IP, UDP_PORT))
        x = 'z'
    
    elif x =='l':
        print("Sending: Go Left")
        sock.sendto(b'left\n', (UDP_IP, UDP_PORT))
        x = 'z'
    
    elif x =='r':
        print("Sending: Go Right")
        sock.sendto(b'right\n', (UDP_IP, UDP_PORT))
        x = 'z'
    
    elif x =='e':
        print("Sending: Exit")
        sock.sendto(b'exit\n', (UDP_IP, UDP_PORT))
        x = 'z'
        break
    
    else:
        print("wrong input")
    