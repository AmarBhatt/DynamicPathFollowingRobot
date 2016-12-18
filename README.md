# DynamicPathFollowingRobot
This project explores making a wireless automatic guided robot which does not require any kind of intrusive modifications to be made to the environment, apart from installing an overhead camera. Generally environments that make use of automatic guided vehicles (AGVs) have to plan the path(s) where the robots should go before installing the tracks, like magnetic strips or metal tracks; this is an investment even before using the robots. If any change to the path(s) is required to be made, then more cost is incurred. In this paper a four wheeled differential drive robot has been controlled wirelessly to follow paths drawn on a graphical user interface within a workspace of 1.8m by 1.4m. The robot is controlled by correcting its orientation through visual feedback from a camera. Error analysis was performed to investigate how well the robot followed the path drawn. The estimated error of the robot is within a few centimeters of the path and can be reduced by modifying various thresholds.

## View Video
https://youtu.be/SvSE9jIfbJU

## Details
This project was tested on MATLAB 2016b on Windows 10.  It requires:

- MATLAB
- OpenCV
- OpenCV Contrib
- Webcam
- Bluetooth module
- Arduino IDE
- Arduino Microcontroller
- Robot

1. MATLAB

  Can be retrieved at: https://www.mathworks.com/

2. OpenCV and Open CV Contrib

  Installed and downloaded from https://github.com/kyamagu/mexopencv
  Using ArUco library from OpenCV Contrib

3. Webcam

  Any webcam that interfaces with MATLAB (i.e. Logitech)

4. Bluetooth

  Any Bluetooth device module (i.e. HC-06)

5. Arduino IDE

  https://www.arduino.cc/en/Main/Software

6. Arduino

  Any Arduino compatible with Arduino IDE

7. Robot

  Any locomotion robot that can be controlled by Arduino (with motor shields, etc)
