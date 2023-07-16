# TrackedRobot
Project of Raspberry Pi Pico W tracked robot focused on remote steering, camera object detection, detecting and avoiding obstacles based on simple radar which uses servo and ultrasonic distance sensor.
##
![](https://github.com/MarcinD12/TrackedRobot/blob/master/IMG_20221013_184746.jpg)
# Hardware:
- Raspberry Pi Pico W
- ESP32cam
- L298N (motor driver&voltage converter)
- MPU6050 (gyroscope&accelerometer)
- 1602a (LCD)
- SG90 (servo)
- HC-SR04 (ultrasonic distance sensor)
 
# How it works
Robot is controlled via [frontend app](https://github.com/MarcinD12/RobotControl) that posts command to [api](https://github.com/MarcinD12/RobotApi) and robot repedately sends GET request to api to get command. Robot can scan environment in 180° radius and pick direction with furthest distance to obstacle. It is able to turn with desired angle (about 80% precision) thanks to gyroscope.
ESP32cam provides video stream in 320x240 resolution. In further update provided video will be used for detecting objects. Prototype of Machine Learning model was trained using ML.NET and Azure ML experiment.
##
![image](https://github.com/MarcinD12/TrackedRobot/assets/111440372/bfa57d74-4055-4e76-9df6-dcf26014f1f9)

 (sample of object detection)

![image](https://github.com/MarcinD12/TrackedRobot/assets/111440372/ba73de6e-e414-4005-958b-2d58e00ea18d)

 (UI and stream)

**Things to do:**
- Add gripper 
- Implement ML model
- Modes like 'autonomous drive', 'area scanning'
- Voltage monitoring
- Save data in database
- Replace prototype cables 
- Improve onboard voltage converter
- containerize app for easier startup
- Emergency reset button

**Done:**
- Embed video to front app (for now it works with workaround with tunneling stream using ngrok)
- Stream video
- POST data from radar
- Trained ML model
- Remote control via requests
- 3D printed Body-done (need improvement)
- Internal power source on 18650 cells


