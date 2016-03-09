
/*
   This program uses an Arduino for a closed-loop control of a DC-motor. 
   Motor motion is detected by a quadrature encoder.
   Two inputs named STEP and DIR allow changing the target position.
   Serial port prints current position and target position every second.
   Serial input can be used to feed a new location for the servo (no CR LF).
   
   Pins used:
   Digital inputs 2 & 8 are connected to the two encoder signals (AB).
   Digital input 3 is the STEP input.
   Analog input 0 is the DIR input.
   Digital outputs 5 & 6 control the PWM outputs for the motor (I am using half L298 here).


   Please note PID gains kp, ki, kd need to be tuned to each different setup. 
*/

#include <PinChangeInt.h>
#define encoder0PinA  2 // PD2; 
#define encoder0PinB  8  // PB0;
#define M1            9  // PWM output
#define M2            10  // Sign output
#define L1            5  // H-bridge low side
#define L2            6  // H-bridge low side

double output=0, setpoint=180;
double margin = 0;

volatile long encoder0Pos = 0;

long messageMillis = 0;        // will store last time LED was updated
long delayMillis = 0;
double lastPos = 0;
volatile long spd = 0;
long rate = 0;
long targetRate = 0;
volatile long lastMicros;

long target=0;  // destination location at any moment

volatile bool cw = false; // motor spin direction, needed to compute forward or braking
volatile bool braking = false;
volatile bool plugging = false;
volatile bool hPol = false; // polarity of the H bridge, for switching low sides on or off

void setup() { 
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(2, INPUT);
  pinMode(encoder0PinA, INPUT); 
  pinMode(encoder0PinB, INPUT);
  digitalWrite(encoder0PinA, 1);
  digitalWrite(encoder0PinB, 1); // weak pullup
  
  pinMode(A0, INPUT);
  pinMode(3, INPUT);
  
  setPwmFrequency(M1, 1);
  
  PCintPort::attachInterrupt(encoder0PinB, doEncoderMotor0,CHANGE); // now with 4x resolution as we use the two edges of A & B pins
  attachInterrupt(0, doEncoderMotor0, CHANGE);  // encoder pin on interrupt 0 - pin 2
  attachInterrupt(1, countStep, RISING);  //on pin 3
  
  // Serial.begin (4800);
} 

void loop(){
    // interpret received data as an integer (no CR LR)
    //if(Serial.available()) target=Serial.parseInt();

    margin = abs(encoder0Pos - target) - 5; // how far off is the encoder?
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
    targetRate = margin * 16;
    if(targetRate > 512) { targetRate = 512; } // but we can only go x fast
    
    // we should only be a factor of y of the rate, depends how much resolution
    rate = rate / 16;
    if(rate - 20 > targetRate) { braking = true; } else { braking = false; }
    output = targetRate - rate;
    
    // integrate an offset:
    if(output > 0) { output += 5; }
    if(output < 0) { output -= 5; }
    
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
    
    else if(plugging == true && margin > 64) {
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

void pwmOut(int out) {
   if(out<0) {
     if(hPol) { hPol = false; analogWrite(M1, 0); analogWrite(M2, 0); digitalWrite(L1, 1); digitalWrite(L2, 0); }
     analogWrite(M1,0); analogWrite(M2,abs(out));
   } else {
     if(!hPol) { hPol = true; analogWrite(M1, 0); analogWrite(M2, 0); digitalWrite(L1, 0); digitalWrite(L2, 1); }
     analogWrite(M2,0); analogWrite(M1,abs(out));
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
  
  // an acceptible offset for compensating for PWM "stiction"
  if(currentDir) {
    if(!dirPin) {
      target -= 8;
    }
  } else {
    if(dirPin) {
      target += 8;
    }
  }
  currentDir = dirPin;
}
