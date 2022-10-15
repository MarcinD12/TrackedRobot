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

def drivesTest():
    pwmPower=45534  #NO MORE THAN 35534 BECAUSE WITH 12V CURRENT IS WAY TOO MUCH (>700mA vs max 500mA)
    lcd.putstr("DRIVES TEST:")
    lcd.move_to(0,1)
    lcd.putstr("Power:"+str(pwmPower))
    MotorPwm.duty_u16(pwmPower)  
    leftMotorIn1.value(1)
    leftMotorIn2.value(0)
    rightMotorIn1.value(1)
    rightMotorIn2.value(0)
    time.sleep(4)
    leftMotorIn1.value(0)
    leftMotorIn2.value(0)
    rightMotorIn1.value(0)
    rightMotorIn2.value(0)
    MotorPwm.duty_u16(0)

drivesTest()




# lcd.clear()
# lcd.putstr("reeee")
# time.sleep(3)
# leftMotorIn1.value(0)
# leftMotorIn2.value(0)
# lcd.clear()
# lcd.putstr("ssss")