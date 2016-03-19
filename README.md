# ClosedLoopDC
Closed Loop Control For DC Servos

   This program uses an Arduino for a closed-loop control of a DC-motor.
   You may have gotten a forked version but the original by Dan Hartman is here:
   https://github.com/dynamodan/ClosedLoopDC
   
   It was inspired by misan's dcservo.ino file which was inspired by ServoStrap,
   but looks less and less like it all the time.  Credit for these go here:
   
   https://github.com/misan/dcservo/blob/master/dcservo.ino
   
   https://github.com/danithebest91/ServoStrap
   
   specifically its PID stragegy is completely different (or non-existent).
   
   Motor motion is still detected by a quadrature encoder.
   Two inputs named STEP and DIR allow changing the target position.
   
   Pins used:
   Digital inputs 2 & 8 are connected to the two encoder signals (AB).
   Digital input 3 is the STEP input.
   Analog input 0 is the DIR input.
   Digital outputs 9 and 10 are PWM and DIR outputs for a mosfet driver board
   (I'm using the MD10C R3 H-bridge and MDD10A dual H-bridge from robotshop.com)

   Please note that you need to tune RMult, RDiv, PWMin, and DZone and possibly other
   values to the "acoustics" of your particular machine with variations on encoder
   resolution, motor gearing etc.  Its probably a good idea to read through and
   attempt to understand what is happening.  In the setup() function there is also
   a place to choose 4x or 2x quadrature encoder interrupt resolution.
