from machine import Pin, I2C, PWM
from pico_i2c_lcd import I2cLcd   #https://github.com/T-622/RPI-PICO-I2C-LCD
from hcsr04 import HCSR04    #https://github.com/gamegine/HCSR04-ultrasonic-sensor-lib
# import network                  #https://docs.micropython.org/en/latest/esp8266/tutorial/network_basics.html
# import socket
import time
from radar import *
#sta_if = network.WLAN(network.STA_IF)
#sta_if.connect('Starlink1','wifipass')
#print(sta_if.isconnected())
#print(sta_if.ifconfig())
#i2c config

led = Pin("LED",Pin.OUT)

i2c=I2C(id=1,scl=Pin(15),sda=Pin(14),freq=400000)       #display
lcd=I2cLcd(i2c, 0x3f, 2, 16)

leftMotorIn1=Pin(3,Pin.OUT)                         #motor driver
leftMotorIn2=Pin(4,Pin.OUT)
rightMotorIn1=Pin(5,Pin.OUT)
rightMotorIn2=Pin(6,Pin.OUT)
MotorPwm=PWM(Pin(2))
MotorPwm.freq(20)

distanceSensor= HCSR04(trigger_pin=11,echo_pin=12)      #radar distance sensor
radarMeasure= list()

def radar():
    servoPwm=PWM(Pin(13))                       #radar servo
    servoPwm.freq(50) 
   # i=2500  
    for i in range(2000,8500,500):              #mid = 5250
        radarDistance=distanceSensor.distance_cm()
        lcd.putstr(str(radarDistance))
        radarMeasure.append(radarDistance)
        servoPwm.duty_u16(i)
        time.sleep(0.5)
        lcd.clear()
    servoPwm.duty_u16(2000)
    time.sleep(1)
    servoPwm.deinit()
    lcd.clear()
    direction = radarMeasure.index(max(radarMeasure)) 
    print(direction)
    return(direction)
    



def servoTest():                #between 2500-8500
    servoPwm.duty_u16(1500)
    time.sleep(1)
    servoPwm.duty_u16(8500)
    time.sleep(1)
    servoPwm.deinit()





#servo=PWM(Pin(13))
#servo.freq(50)
#distanceSensor= HCSR04(trigger_pin=17,echo_pin=16)


"""     Motor steering:
        IN1    IN2    PWM   
        1       0      x   -Max PWM CW
        0       1      x   -Max PWM-CCW
        0       0      1   -Fast stop
        1       1      1   -Soft stop   
"""
def moveBackward(timeOfMove):
    MotorPwm.duty_u16(power)
    leftMotorIn1.value(1)
    leftMotorIn2.value(0)
    rightMotorIn1.value(1)
    rightMotorIn2.value(0)
    time.sleep(timeOfMove)
    moveStop()

def moveForward(timeOfMove):
    MotorPwm.duty_u16(power)
    leftMotorIn1.value(0)
    leftMotorIn2.value(1)
    rightMotorIn1.value(0)
    rightMotorIn2.value(1)
    time.sleep(timeOfMove)
    moveStop()
    


def turnLeft(timeOfMove):
    MotorPwm.duty_u16(power)
    leftMotorIn1.value(1)
    leftMotorIn2.value(0)
    rightMotorIn1(0)
    rightMotorIn2(1)
    time.sleep(timeOfMove)
    moveStop()

def turnRight(timeOfMove):
    MotorPwm.duty_u16(power)
    rightMotorIn1.value(1)
    rightMotorIn2.value(0)
    leftMotorIn1(0)
    leftMotorIn2(1)
    time.sleep(timeOfMove)
    moveStop()

def moveStop():
    leftMotorIn1.value(0)
    leftMotorIn2.value(0)
    rightMotorIn1.value(0)
    rightMotorIn2.value(0)


def drivesTest(timeOfMove):
    MotorPwm.duty_u16(power)
    moveForward(timeOfMove)
    moveBackward(timeOfMove)  
    turnLeft(timeOfMove)
    turnRight(timeOfMove)
    

    




#moveForward(20000)
power = 20000
#turnLeft(1.5)
#radarAngle=radar()
radarAngle=radar()
print(radarAngle)
if(radarAngle!=6):
    if(radarAngle>6):
        turnLeft(1.5/13*radarAngle)
    else:
        (turnRight(1.5/13*radarAngle))  
else:
    moveForward(2)            
      

#moveStop()
#moveBackward(1)



######################### WEB CONTROLLING


def connect():
    #Connect to WLAN
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect('Starlink1', 'wifipass')
    while wlan.isconnected() == False:
        print('Waiting for connection...')
        sleep(1)
    ip = wlan.ifconfig()[0]
    print(f'Connected on {ip}')
    return ip

def webpage():
    #Template HTML
    html = f"""
            <!DOCTYPE html>
            <html>
            <head>
            <title>Zumo Robot Control</title>
            </head>
            <center><b>
            <form action="./forward">
            <input type="submit" value="Forward" style="height:120px; width:120px" />
            </form>
            <table><tr>
            <td><form action="./left">
            <input type="submit" value="Left" style="height:120px; width:120px" />
            </form></td>
            <td><form action="./stop">
            <input type="submit" value="Stop" style="height:120px; width:120px" />
            </form></td>
            <td><form action="./right">
            <input type="submit" value="Right" style="height:120px; width:120px" />
            </form></td>
            </tr></table>
            <form action="./back">
            <input type="submit" value="Back" style="height:120px; width:120px" />
            </form>
            </body>
            </html>
            """
    return str(html)
def serve(connection):
    #Start web server
    while True:
        client = connection.accept()[0]
        request = client.recv(1024)
        request = str(request)
        try:
            request = request.split()[1]
        except IndexError:
            pass
        if request == '/forward?':
            moveForward(50000)
        elif request =='/left?':
            turnLeft(50000)
        elif request =='/stop?':
            moveStop()
        elif request =='/right?':
            turnRight(50000)
        elif request =='/back?':
            moveBackward(50000)
        html = webpage()
        client.send(html)
        client.close()


def open_socket(ip):
    # Open a socket
    address = (ip, 80)
    connection = socket.socket()
    connection.bind(address)
    connection.listen(1)
    return connection

    # try:
#     ip = connect()
#     connection = open_socket(ip)
#     serve(connection)
# except KeyboardInterrupt:
#     machine.reset()