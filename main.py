from machine import Pin, I2C, PWM
from pico_i2c_lcd import I2cLcd   #https://github.com/T-622/RPI-PICO-I2C-LCD
from hcsr04 import HCSR04           #https://github.com/gamegine/HCSR04-ultrasonic-sensor-lib
import time

led = Pin("LED",Pin.OUT)

i2c=I2C(id=1,scl=Pin(15),sda=Pin(14),freq=400000)

lcd=I2cLcd(i2c, 0x3f, 2, 16)

#servo=PWM(Pin(13))
#servo.freq(50)
#distanceSensor= HCSR04(trigger_pin=17,echo_pin=16)
leftMotorIn1=Pin(3,Pin.OUT)
leftMotorIn2=Pin(4,Pin.OUT)
rightMotorIn1=Pin(5,Pin.OUT)
rightMotorIn2=Pin(6,Pin.OUT)
MotorPwm=PWM(Pin(2))

"""     Motor steering:
        IN1    IN2    PWM   
        1       0      x   -Max PWM CW
        0       1      x   -Max PWM-CCW
        0       0      1   -Fast stop
        1       1      1   -Soft stop   
"""
def moveBackward(power):
    MotorPwm.duty_u16(power)
    leftMotorIn1.value(0)
    leftMotorIn2.value(1)
    rightMotorIn1.value(0)
    rightMotorIn2.value(1)

def moveForward(power):
    MotorPwm.duty_u16(power)
    leftMotorIn1.value(1)
    leftMotorIn2.value(0)
    rightMotorIn1.value(1)
    rightMotorIn2.value(0)
    


def moveLeft(power):
    MotorPwm.duty_u16(power)
    leftMotorIn1.value(1)
    leftMotorIn2.value(0)


def moveRight(power):
    MotorPwm.duty_u16(power)
    rightMotorIn1.value(1)
    rightMotorIn2.value(0)


def moveStop():
    leftMotorIn1.value(0)
    leftMotorIn2.value(0)
    rightMotorIn1.value(0)
    rightMotorIn2.value(0)


def drivesTest(power):
    MotorPwm.duty_u16(power)
    moveForward(power)
    time.sleep(2)
    moveStop()
    moveBackward(power)
    time.sleep(2)
    moveStop()
    moveLeft(power)
    time.sleep(2)
    moveStop()
    moveRight(power)
    time.sleep(2)
    moveStop()
moveStop()
powerPercent=120           #200==max
powerValue=int(powerPercent/0.0028)
lcd.putstr("DRIVES TEST")
#drivesTest(powerValue)
moveForward(powerValue)
time.sleep(1)
moveStop()

