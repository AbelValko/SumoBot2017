/*
 * Sumobot code V1.5
 * Created 12 Nov 2017
 * by Abel Valko
 */
#include <NewPing.h>

// button
const int button = A0;

// motor1
const int motorRight1 = 8;
const int motorRight2 = 9;
const int en1 = 11;

// motor2
const int motorLeft1 = 7;
const int motorLeft2 = 6;
const int en2 = 5;

// ultrasound sensor
const int pingRightEcho = 12;
const int pingRightTrig = 13;
const int pingLeftEcho = 3;
const int pingLeftTrig = 2;

// IR sensor
const int IRRight = 4;
const int IRLeft = 10;

// System parameters
const unsigned long searchTimeTimeout = 40000; // timeout to dumb mode after 1 min 20 sec
const unsigned long dumbModeTimeout = 150000; // timeout to dumb mode after 4 minutes running
const unsigned long timeoutOne = 6000; // timeout with severity 1 after 6 seconds LEGACY
const unsigned long timeoutTwo = 4000; // timeout with severity 2 after 4 seconds
const int detectDistance = 35;  // max object detection distance in cm
const int maxDistance = 160;  // ping sensor gives up waiting for reply after maxDistance ms
const int pingSpeed = 40; // time for one cycle of senses, ie one sense with each sensor
const int lmSize = 2; // size of last measurement array for running average

// Ultrasonic constructor
NewPing pingRight(pingRightTrig, pingRightEcho, maxDistance);
NewPing pingLeft(pingLeftTrig, pingLeftEcho, maxDistance);

bool smartMode = true; // navigation mode, default smart mode, switch to dumb mode after timeout 3
bool edgeAvoidFlag = false; // true if edge detected, all other functions will be put on pause
bool attackFlag = false;  // true if robot ready for attack maneuver
bool searchFlag = false;  // true if robot has not detected enemy, search maneuver will be performed
bool newSearch = true;  // true if new search is to be initiated, search time and direction is reset
bool avoidManeuver = false;
float rightSenseDistance;  // average of past maxPingCounter ultrasound readings
float leftSenseDistance; // average of past maxPingCounter ultrasound readings
float randomTurnTime;  // random time for dumb mode edge avoidance
unsigned long searchTime = 0;  // time of last new search
unsigned long lastPrint = 0;
unsigned long edgeSenseTime = 0;
unsigned long pingTimerRight, pingTimerLeft, avoidStart;
byte detectPosition;  // position of edge detection, 0 right, 1 left, 2 middle
byte searchTimeout = 0; // timeout severity, 0 none, 1 opposite rotation, 2 relocate maneuver, 3 switch to dumb mode
byte lastDetection = 1; // location of last detection, 0 right, 1 left
  
void setup() {
  // set motor pins to output
  pinMode(motorRight1, OUTPUT);
  pinMode(motorRight2, OUTPUT);
  pinMode(motorLeft1, OUTPUT);
  pinMode(motorLeft2, OUTPUT);
  pinMode(en1, OUTPUT);
  pinMode(en2, OUTPUT);
  // set IR sensor pins to input
  pinMode(IRRight, INPUT);
  pinMode(IRLeft, INPUT);

  // set ping timers for alternating senses
  pingTimerRight = millis() + pingSpeed;
  pingTimerLeft = pingTimerRight + (pingSpeed / 2);

  delay(5);  // short delay to allow for sensor initialization

  //Serial.begin(74880);

  initialRelocate();  
}

void loop() {

  unsigned long currentMillis = millis(); // time since code started running
  static unsigned long startTime = currentMillis; // start of program
  static unsigned long lastAttack = currentMillis;  // time of last attack
  static unsigned long edgeSenseStart;

  // default to "dumb mode" if searchTimeTimeout has passed since last attack or dumbModeTimeout since start of program
  if ((currentMillis - lastAttack) >= searchTimeTimeout || (currentMillis - startTime) >= dumbModeTimeout ){
    smartMode = false;
  }

  // serial output every 100ms for debugging, must be removed during actual usage for speed
  if ((currentMillis - lastPrint) >= 100){
    //dump();
    lastPrint = currentMillis;
  }
  if (edgeAvoidFlag == false){
    // read both IR sensors, returns 1 if edge detected and 0 if no edge detected
    int IRRight_read = digitalRead(IRRight);
    int IRLeft_read = digitalRead(IRLeft);
    // if field edge detected by both sensors
    if (IRRight_read == 1 && IRLeft_read == 1){
      detectPosition = 2;
      edgeAvoidFlag = true;
      avoidManeuver = true;
      randomTurnTime = random(300,700);
      avoidStart = currentMillis;
    }
    // if field edge detected by right sensor
    else if (IRRight_read == 1 && IRLeft_read == 0){
      detectPosition = 0;
      edgeSenseTime = currentMillis - edgeSenseStart;
      edgeAvoidFlag = true;
      avoidManeuver = true;
      randomTurnTime = random(250,450);
      avoidStart = currentMillis;
    }
    // if field edge detected by left sensor
    else if (IRRight_read == 0 && IRLeft_read == 1){
      detectPosition = 1;
      edgeSenseTime = currentMillis - edgeSenseStart;
      edgeAvoidFlag = true;
      avoidManeuver = true;
      randomTurnTime = random(250,450);
      avoidStart = currentMillis;
    }
    else{
      edgeSenseTime = 0;
    }
  }
  
  if (avoidManeuver == true){
    avoidEdge(detectPosition, avoidStart, randomTurnTime, edgeSenseTime);
  }
  
  if (smartMode == true){
    // ultrasound sensor is inactive during edge avoidance
    if (edgeAvoidFlag == false){
      if (millis() >= pingTimerRight){   
          pingTimerRight += pingSpeed;
          float rightSenseValue = pingRight.ping_cm();
          if (rightSenseValue != 0){
            rightSenseDistance = runningAverageRight(rightSenseValue);
          }
      }
      if (millis() >= pingTimerLeft){
          pingTimerLeft = pingTimerRight + (pingSpeed / 2);
          float leftSenseValue = pingLeft.ping_cm();
          if (leftSenseValue != 0){
            leftSenseDistance = runningAverageLeft(leftSenseValue);
          }
      }
    }
    
    if (edgeAvoidFlag == false){
      // if enemy detected in middle initiate attack maneuver
      if (rightSenseDistance <= detectDistance && leftSenseDistance <= detectDistance 
          && almostEqual(rightSenseDistance, leftSenseDistance) && leftSenseDistance != 0 && rightSenseDistance != 0){
        attackFlag = true;
        searchFlag = false;
        lastAttack = currentMillis;
      }
      // if enemy detected to the right, start search towards right
      else if (rightSenseDistance <= detectDistance && rightSenseDistance < leftSenseDistance && rightSenseDistance != 0){
        if (lastDetection == 1){
          newSearch = true;
        }
        else{
          newSearch = false;
        }
        lastDetection = 0;
        updateSearch();
      }
      // if enemy detected to the left, start search towards left
      else if (leftSenseDistance <= detectDistance && leftSenseDistance < rightSenseDistance && leftSenseDistance != 0){
        if (lastDetection == 0){
          newSearch = true;
        }
        else{
          newSearch = false;
        }
        lastDetection = 1;
        updateSearch();
      }
      else{
        updateSearch();
      }
    }      

    // search timeout 1 is legacy
    if (avoidManeuver == false){
      if (searchFlag == true){
        if ((millis() - searchTime) >= timeoutTwo){
          static unsigned long relocateStart;
          static byte turnDir;
          static byte turnAngle;
          if (searchTimeout != 2){
            relocateStart = millis();
            turnDir = random(0,1);
            turnAngle = random(50, 300);
          }
          searchTimeout = 2;
          analogWrite(en1, 255);
          analogWrite(en2, 255);
          relocate(relocateStart, turnDir, turnAngle);
          newSearch = true;
        }
        else{
           analogWrite(en1, 55);
           analogWrite(en2, 55);
           search();
        }
      }
    }

    if (edgeAvoidFlag == false){
      if (attackFlag == true){
        //POTENTIAL MANEUVER
        avoidManeuver = false;
        analogWrite(en1, 255);
        analogWrite(en2, 255);
        goStraight();
      }
    }
  }
  else{
    analogWrite(en1, 255);
    analogWrite(en2, 255);
    goStraight();
  }

}

// search function first pivots towards last detection (detection on right occurs by left sensor and vice versa) and then other way
void search (){
  unsigned long currentMillis = millis();
  static unsigned long searchStart = currentMillis;
  if (newSearch == true){
    searchStart = currentMillis;
  }
  
  if ((currentMillis - searchStart) <= 300){
    if (lastDetection == 0){
      pivot(1);
    }
    else if (lastDetection == 1){
      pivot(0);
    }
  }
  else{
    if (lastDetection == 0){
      pivot(0);
    }
    else if (lastDetection == 1){
      pivot(1);
    }
  }
}

void pivot (int robotDirection){
  if (robotDirection == 0){
    digitalWrite(motorRight1, LOW);
    digitalWrite(motorRight2, HIGH);
    digitalWrite(motorLeft1, HIGH);
    digitalWrite(motorLeft2, LOW);
  }
  else if (robotDirection == 1){
    digitalWrite(motorRight1, HIGH);
    digitalWrite(motorRight2, LOW);
    digitalWrite(motorLeft1, LOW);
    digitalWrite(motorLeft2, HIGH);
  }
}

void goBackwards(){
  digitalWrite(motorRight1, HIGH);
  digitalWrite(motorRight2, LOW);
  digitalWrite(motorLeft1, HIGH);
  digitalWrite(motorLeft2, LOW);
}

void goStraight(){
  digitalWrite(motorRight1, LOW);
  digitalWrite(motorRight2, HIGH);
  digitalWrite(motorLeft1, LOW);
  digitalWrite(motorLeft2, HIGH);
}

void motorOff(){
  digitalWrite(motorRight1, LOW);
  digitalWrite(motorRight2, LOW);
  digitalWrite(motorLeft1, LOW);
  digitalWrite(motorLeft2, LOW);
}

// relocate meneuver pivots a random angle in a random direction and then moves forward a given amount of time 
// for strategic relocation in case no enemy is detected. Initiated after timeout 2 is reached.
void relocate(unsigned long relocateStart, byte turnDir, unsigned long turnTime){
  unsigned long currentMillis = millis();
  if ((currentMillis - relocateStart) <= turnTime){
    pivot(turnDir);
  }
  else if ((currentMillis - relocateStart) <= (turnTime + 800)){
    goStraight();
  }
  else{
    searchTime = currentMillis;
    searchTimeout = 0;
  }
}

void initialRelocate(){
  analogWrite(en1,255);
  analogWrite(en2,255);
  pivot(1);
  delay(400);
  goStraight();
  delay(700);
  pivot(1);
  delay(250); 
}

// compares two floating point values and returns true if they are within delta of eachother
bool almostEqual(float a, float b){
  const float delta = 25;
  return fabs(a - b) <= delta;
}

void avoidEdge(int detectPosition, float avoidStartTime, float randomTurnTime, unsigned long senseTime){
  static int backUpTime = 900;
  static int turnTime = 400;
  static int minTurnTime = 200;
  unsigned long currentMillis = millis();

  if ((currentMillis - avoidStartTime) <= minTurnTime){
    analogWrite(en1, 255);
    analogWrite(en2, 255);
  }
  else{
    analogWrite(en1, 70);
    analogWrite(en2, 70);
  }

  if (detectPosition == 0){  // if edge detected on right
    if (randomTurnTime >= (currentMillis - avoidStartTime)){
      pivot(1);
    }
    else if ((randomTurnTime <= (currentMillis - avoidStartTime)) && ((backUpTime + randomTurnTime) >= (currentMillis - avoidStartTime))){
      goStraight();
    }
    else if ((currentMillis - avoidStartTime) >= (backUpTime + randomTurnTime)){
      avoidManeuver = false;
    }
  }
  else if (detectPosition == 1){  // if edge detected on left
    if (randomTurnTime >= (currentMillis - avoidStartTime)){
      pivot(0);
    }
    else if ((randomTurnTime <= (currentMillis - avoidStartTime)) && ((backUpTime + randomTurnTime) >= (currentMillis - avoidStartTime))){
      goStraight();
    }
    else if ((currentMillis - avoidStartTime) >= (backUpTime + randomTurnTime)){
      avoidManeuver = false;
    }
  }
  else if (detectPosition == 2){  // if edge detected on left
    if (randomTurnTime >= (currentMillis - avoidStartTime)){
      pivot(0);
    }
    else if ((randomTurnTime <= (currentMillis - avoidStartTime)) && ((backUpTime + randomTurnTime) >= (currentMillis - avoidStartTime))){
      goStraight();
    }
    else if ((currentMillis - avoidStartTime) >= (backUpTime + randomTurnTime)){
      avoidManeuver = false;
    }
  }

  if ((currentMillis - avoidStartTime) >= minTurnTime){
    edgeAvoidFlag = false;
  }
}

// returns amount of free ram, for debugging purposes
int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

// calculates the rolling average
float runningAverageRight(float M){  
  static float LM[lmSize];
  static byte index = 0;
  static float sum = 0;
  static byte count = 0;

  sum -= LM[index];
  LM[index] = M;
  sum += LM[index];
  index ++;
  index = index % lmSize;
  if (count < lmSize) count++;

  return sum / count;
}

// also calculates rolling average, too much effort to have two instances of the same function...
float runningAverageLeft(float M){  
  static float LM[lmSize];
  static byte index = 0;
  static float sum = 0;
  static byte count = 0;

  sum -= LM[index];
  LM[index] = M;
  sum += LM[index];
  index ++;
  index = index % lmSize;
  if (count < lmSize) count++;

  return sum / count;
}

// updates search conditions at the beginning of a new search command
void updateSearch(){
  if (searchFlag == false){
    //Serial.println("searchTimeout Reset!");
    searchTimeout = 0;
    searchTime = millis();
    newSearch = true;
  }
  searchFlag = true;
  attackFlag = false;
}

// dumps info on serial for debugging
void dump(){
  static unsigned long loops = 0;
  loops++;
  unsigned long currentMillis = millis();
  Serial.print(loops);
  Serial.print(" millis:");
  Serial.print(currentMillis);
  Serial.print(" smartMode: ");
  Serial.print(smartMode);
  Serial.print(" rightSenseDistance: ");
  Serial.print(rightSenseDistance);
  Serial.print(" leftSenseDistance: ");
  Serial.print(leftSenseDistance);    
  Serial.print(" searchFlag: ");
  Serial.print(searchFlag);
  Serial.print(" edgeAvoidFlag: ");
  Serial.print(edgeAvoidFlag);
  Serial.print(" searchTimeout: ");
  Serial.print(searchTimeout);
  Serial.print(" attackFlag: ");
  Serial.print(attackFlag);
  Serial.println();
}

