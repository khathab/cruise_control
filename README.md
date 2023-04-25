# cruise_control

As roads become congested and distractions to drivers increase, accidents continue to occur due to human error.
An Advanced Driver Assistance System (ADAS) would help drivers avoid collisions, and improve safety on the road. 
The goal is to design a system that assists with basic tasks while keeping the driver in charge.

Our ADAS system allows a user to set the cruise control and dynamically adjust the speed in real-time based on the car in front of it.
The system interacts with the user through a set of controls on the steering wheel and dashboard display. It also interacts with the environment 
by using sensors such as radar and lidar to monitor the road and vehicles. Adjusting the system’s speed of the vehicle by managing stimuli such as 
throttle and brake inputs will help maintain a safe and comfortable driving experience. Furthermore, in order to allow the ADAS system to react quickly 
to changes in the environment and maintain a safe and comfortable driving experience for the user, the frequency at which the ADAS system should check the 
vehicle in front of it and update its speed will be 60 Hz.

The current systems makes use of a DE10-Soc Standard Microcontroller. The code uses the built in accelerometer, GPIO and ADC ports in the board to use the required components. 
To make use of the board's accelerometer, an I2C connection had to be manually made for the board. The sonar also had to be manually coded, thus making use of the built-in timer 
of the DE10. 

To execute the project, you will need an ultrasonic sensor, a potentiometer and  DE10-Soc Standard board
