# line-follower-robot
Arduino Code for a line follower robot

Used Libraries:
NewPing (!)
Sparkfun_APDS9960

Implemented Functions:
Following a black line using 6 digital sensors;
Detecting an obstacle aprox. 30cm in front;
Detecting the color of the obstacle and playing an according sound to the detected colour (Available colours - Red, Green, Blue, White, Black);

IMPORTANT:
NewPing and Tone functions both use the same timer, the usage of the timer was disabled in the NewPing library. ADPS9960 library is stock, get it from Arduino library manager. Wire library there is for shows not actually used.
