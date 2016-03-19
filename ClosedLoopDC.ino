/*

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
   
*/

#include <PinChangeInt.h>
#define encoder0PinA  2 // PD2; 
#define encoder0PinB  8  // PB0;
#define M1            9  // PWM output, this one gives 16KHz with an 8MHz crystal or the 8MHz internal clock
#define M2            10  // DIR output

#define SMax	512	// Speed Maximum, limit the target rate to this.  Never lower than 256, never higher than what your encoder can reliably drive the arduino's inputs
#define RMult	16 	// Rate multiplier, less with higher resolution
#define RDiv	16  // Rate divider, generally 256 / RMult.  Greater with higher resolution
#define PWMin	5 	// where PWM starts, if greater than zero. Increase this if motors sit there squealing with not enough torque to move
#define DZone 	5	// Dead Zone, the encoder can freely move this many clicks either direction before torque is called
#define PMarg	64	// Plugging margin, any margin above this will always give 100% power in the opposite direction when plugging (resisting movement away from target)
#define BLash	8	// Backlash compensation

double output = 0;
double margin = 0;

volatile long encoder0Pos = 0;

long messageMillis = 0;        // will store last time LED was updated
long delayMillis = 0;
double lastPos = 0;
volatile long spd = 0;
long rate = 0;
long targetRate = 0;
volatile long lastMicros;
volatile long target = 0;  // destination location at any moment

volatile bool cw = false; // motor spin direction, needed to compute forward or braking
volatile bool braking = false;
volatile bool plugging = false;

void setup() { 
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(encoder0PinA, INPUT); 
  pinMode(encoder0PinB, INPUT);
  digitalWrite(encoder0PinA, 1); // weak pullup (I add a 220 ohm pullup in some cases)
  digitalWrite(encoder0PinB, 1); // weak pullup
  
  pinMode(A0, INPUT);
  pinMode(3, INPUT);
  
  setPwmFrequency(M1, 1);
  
  // reading the input from the encoder at 4x resolution: (comment this out if the 2x block below is in use)
  PCintPort::attachInterrupt(encoder0PinB, doEncoderMotor0,CHANGE); // now with 4x resolution as we use the two edges of A & B pins
  attachInterrupt(0, doEncoderMotor0, CHANGE);  // encoder pin on interrupt 0 - pin 2
  
  // // reading the input from the encoder at 2x resolution: (comment this out if the 4x block above is in use)
  // use this if the encoder is extremely fine resolution, or has issues with speed too
  // great for a given resolution and clock speed
  // attachInterrupt(0, doHalfEncoderMotor0, CHANGE);  // encoder pin on interrupt 0 - pin 2
  
  // reading the input from GRBL:
  attachInterrupt(1, countStep, RISING);  //on pin 3
  
  // Serial.begin (4800);
} 

void loop() {
    // interpret received data as an integer (no CR LR)
    //if(Serial.available()) target=Serial.parseInt();

    margin = abs(encoder0Pos - target) - DZone; // how far off is the encoder?
    if(margin < 0) { margin = 0; }
    
    rate = 1000000 / spd;  // clicks per second, or something like that
 
    // every 50 millis, check if we've moved.  If not, then rate is zero.
    //if(lastPos == encoder0Pos) { rate = 0; }
    //if(millis() > delayMillis + 50) {
    //  lastPos = encoder0Pos;
    //  delayMillis = millis();
      // target -= 2;
    //}
    
    // The farther off target we are, the faster we should try to get there.
    // In other words, this setting governs how quickly torque will build up the farther
    // off target (or, say the margin) the encoder is
    targetRate = margin * RMult;
    if(targetRate > SMax) { targetRate = SMax; } // but we can only go x fast
    
    // we should only be a factor of y of the rate, depends how much resolution
    rate = rate / RDiv;
    if(rate - 20 > targetRate) { braking = true; } else { braking = false; }
    output = targetRate - rate;
    
    // integrate an offset:
    if(output > 0) { output += PWMin; }
    if(output < 0) { output -= PWMin; }
    
    // speed up, step on the gas!
    if(output > 255) {
      output = 255;
    }
    
    // slow down, put on the brakes!
    if(output < -255) { output = -255; }
    
    // invert PWM output for main direction:
    if(encoder0Pos < target) {
      output = output * -1;
    }
    
    // invert PWM in overshoot condition:
    plugging = false;
    if(rate > 0) {
      if(encoder0Pos < target && cw == false && output > 0) { // going away from target!!
        output = output * -1;
        plugging = true;
      } else if(encoder0Pos > target && cw == true && output < 0) { // going away from target!!
        output = output * -1;
        plugging = true;
      }
    }
    
    if(braking) {
      output = output * -1;
    }
    
    else if(plugging == true && margin > PMarg) {
      if(output > 0) { output = 255; }
      else if(output < 0) { output = -255; }
    }
    
    if(margin == 0) { output = 0; }
    
    // write the PWM force value to the speed control:
    pwmOutSigned(output);
    
    // print encoder and target every second throgh the serial port 
    //if(millis() > messageMillis+4000 )  {
    //    if(encoder0Pos == target) { output = 0; }
    //    Serial.print(encoder0Pos); Serial.print("->");
    //    Serial.print(output); Serial.print("->");
    //    Serial.println(rate);
    //    messageMillis=millis();
    //    if(target == 0) { target = 2000; } else { target = 0; }
    //}
}

void pwmOutSigned(int out) {
  if(out<0) {
    analogWrite(M1, abs(out));
    digitalWrite(M2, 0);
  } else {
    analogWrite(M1, out);
    digitalWrite(M2, 1);
  }
}

void doHalfEncoderMotor0(){
  if (((PIND&B0000100)>>2) == HIGH) {   // found a low-to-high on channel A; if(digitalRead(encoderPinA)==HIGH){.... read PB0
     spd = micros() - lastMicros;
     //digitalWrite(Qu2, HIGH);
    if ((PINB&B0000001) == LOW) {  // check channel B to see which way; if(digitalRead(encoderPinB)==LOW){.... read PB0
      encoder0Pos++ ;         // CCW
      cw = false;
      //digitalWrite(Qu1, LOW);
    } 
    else {
      encoder0Pos-- ;         // CW
      cw = true;
      //digitalWrite(Qu1, HIGH);
    }
  }
  else                                        // found a high-to-low on channel A
  { 
    spd = micros() - lastMicros;
    //digitalWrite(Qu2, LOW);
    if ((PINB&B0000001) == LOW) {   // check channel B to see which way; if(digitalRead(encoderPinB)==LOW){.... read PB0
      //digitalWrite(Qu1, LOW);
      encoder0Pos-- ;          // CW
      cw = true;
    } 
    else {
      //digitalWrite(Qu1, HIGH);
      encoder0Pos++ ;          // CCW
      cw = false;
   }
  }
  lastMicros = micros();
}

// Quadrature Encoder Matrix
const int QEM [16] = {0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0};
static unsigned char newQ, oldQ;
void doEncoderMotor0(){
  spd = micros() - lastMicros;
  oldQ = newQ;
  newQ = (PINB & 1 )+ ((PIND & 4) >> 1);
  char qChange = QEM [oldQ * 4 + newQ];
  encoder0Pos+= qChange;
  if(qChange < 0) cw = false;
  if(qChange > 0) cw = true;
  lastMicros = micros();
}

void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

static bool currentDir;
void countStep(){ // pin A0 represents direction
  bool dirPin = PINC&B0000001;
  if (dirPin) {
    target++;
  } else {
    target--;
  }
  
  // compensate for PWM "stiction", backlash, and such things:
  if(currentDir) {
    if(!dirPin) {
      target -= BLash;
    }
  } else {
    if(dirPin) {
      target += BLash;
    }
  }
  currentDir = dirPin;
}
