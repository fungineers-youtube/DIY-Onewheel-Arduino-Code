# DIY-Onewheel-Arduino-Code
Arduino code for the DIY Onewheel (from the Fungineers Youtube channel).
Project Brief: 
The project is to build a replica of the Onewheel at 1/4 the cost. 
Parts: Hub motor wheel, Vesc, Arduino, MPU6050, 10s2p battery (or whatever you like). Rails and enclosures (3D printed or aluminium, or a mixture of both).
About the code: 
The Arduino code is a working one. 
It takes the angle measurements from the Gyro (MPU), and converts them to a pwm signal between 1000 and 2000 and sends to the VESC.
It uses the Kalman filter and the complemenary filter. The servo and the wire libraries are used as well.
Future plans/ collaboration ideas: 
Make the code UART instead of just PWM so that the UART port on the VESC can be used (to send and recevie data), and display the received date on a screen via bluetooth module.
