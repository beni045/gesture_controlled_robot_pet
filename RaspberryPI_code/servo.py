# Import libraries
import RPi.GPIO as GPIO
import time

# Pins for respective motors
SERVO_X = 11
SERVO_Z = 12
START_X = 90
START_Z = 15
# Set GPIO numbering mode
GPIO.setmode(GPIO.BOARD)

# Global for angle value
x_position = START_X
z_position = START_Z

# Set pins 11 & 12 as outputs, and define as PWM servo1 & servo2
GPIO.setup(SERVO_X,GPIO.OUT)
servo_x = GPIO.PWM(SERVO_X,50) # pin 11 for servo_x, pulse 50Hz
GPIO.setup(SERVO_Z,GPIO.OUT)
servo_z = GPIO.PWM(SERVO_Z,50) # pin 12 for servo_z, pulse 50Hz

# Function for servo to go to specific angle
# Z: 90=straight up
def go_to_angle(servo, angle):
    servo.ChangeDutyCycle(2+(angle/18))
    time.sleep(0.5)
    servo.ChangeDutyCycle(0)
    
def move_up():
    global z_position
    z_position = z_position + 5
    go_to_angle(servo_z, z_position)
    
def move_down():
    global z_position
    z_position = z_position - 5
    go_to_angle(servo_z, z_position)

def move_left():
    global x_position
    x_position = x_position + 5
    go_to_angle(servo_x, x_position)

def move_right():
    global x_position
    x_position = x_position - 5
    go_to_angle(servo_x, x_position)


def main():
    # Start PWM running on both servos, value of 0 (pulse off)
    servo_x.start(0)
    servo_z.start(0)

    print ("Servo Control Started")

    go_to_angle(servo_x, START_X)
    go_to_angle(servo_z, START_Z)
    
    while True:
        data = input('Enter u,d,l,r ')
        #simplest way to react
        if data=="u":
            move_up()
            print("Look up " + str(z_position))
        
        elif data=="d":
            move_down()
            print("Look down " + str(z_position))
        
        elif data=="l":
            move_left()
            print("Look left " + str(x_position))
        
        elif data=="r":
            move_right()
            print("Look right " + str(x_position))
        
        elif data=="e":
            go_to_angle(servo_x, START_X)
            go_to_angle(servo_z, START_Z)
            print("Program Exit, Cleaning up")
            break
        
    #Clean things up at the end
    servo_x.stop()
    servo_z.stop()
    GPIO.cleanup()

    print ("Goodbye")

if __name__ == '__main__':
    main()