#include <ESP32Servo.h>

#define sleep_pin 21

#define ledGreen 17
#define ledRed 13

#define penServoPin 12
Servo penServo;

#define x_input1 25
#define x_input2 26
#define x_encoder 32

#define y_input1 27
#define y_input2 14
#define y_encoder 33

#define emergencyPin 16
#define xLimitSwitchPin 19
#define yLimitSwitchPin 18

bool emergencyTriggered = false;
bool emergencyHandled = false;

volatile unsigned long xPulseCount = 0;
volatile unsigned long yPulseCount = 0;

const float pulsesPerRevolution = 20.0;
unsigned long xTargetPulses = 0;
unsigned long yTargetPulses = 0;

void IRAM_ATTR xEncoderISR() {
  xPulseCount++;
}

void IRAM_ATTR yEncoderISR() {
  yPulseCount++;
}

void run_motor_X(bool dir, uint8_t speedPWM = 255) {
  if (dir) {
    analogWrite(x_input1, speedPWM);
    analogWrite(x_input2, 0);
  } else {
    analogWrite(x_input1, 0);
    analogWrite(x_input2, speedPWM);
  }
}

void stop_motor_X() {
  analogWrite(x_input1, 0);
  analogWrite(x_input2, 0);
}

void run_motor_Y(bool dir, uint8_t speedPWM = 255) {
  if (dir) {
    analogWrite(y_input1, speedPWM);
    analogWrite(y_input2, 0);
  } else {
    analogWrite(y_input1, 0);
    analogWrite(y_input2, speedPWM);
  }
}

void stop_motor_Y() {
  analogWrite(y_input1, 0);
  analogWrite(y_input2, 0);
}


void moveX(float revolutions, bool dir, uint8_t speedPWM = 255) {
  if (digitalRead(emergencyPin) == LOW && !emergencyTriggered && !emergencyHandled) return;

  xPulseCount = 0;
  xTargetPulses = (unsigned long)(revolutions * pulsesPerRevolution);
  run_motor_X(dir, speedPWM);
  while (xPulseCount < xTargetPulses) {
    if (digitalRead(emergencyPin) == LOW && !emergencyTriggered && !emergencyHandled) {
      stop_motor_X();
      return;
    }
    yield();
  }
  stop_motor_X();
  delay(300);
}

void moveY(float revolutions, bool dir, uint8_t speedPWM = 255) {
  if (digitalRead(emergencyPin) == LOW && !emergencyTriggered && !emergencyHandled) return;

  yPulseCount = 0;
  yTargetPulses = (unsigned long)(revolutions * pulsesPerRevolution);
  run_motor_Y(dir, speedPWM);
  while (yPulseCount < yTargetPulses) {
    if (digitalRead(emergencyPin) == LOW && !emergencyTriggered && !emergencyHandled) {
      stop_motor_Y();
      return;
    }
    yield();
  }
  stop_motor_Y();
  delay(300);
}

void moveDiagonalPID(float revY, bool dirY, float revX, bool dirX) {
  //penServo.detach();
  xPulseCount = 0;
  yPulseCount = 0;

  xTargetPulses = (unsigned long)(revX * pulsesPerRevolution);
  yTargetPulses = (unsigned long)(revY * pulsesPerRevolution);

  // PID tuning parameters
  float Kp = 10.0;  // Start here, adjust experimentally
  float Ki = 0.0;
  float Kd = 1.0;



  float integral = 0;
  float prevError = 0;

  // Speed profile settings
  int minPWM = 240;  // Start speed
  int maxPWM = 255;  // Cruising speed
  int pwmX = minPWM;
  int pwmY = minPWM;

  int accelStep = 3;      // PWM increment for acceleration
  int decelDistance = 3;  // pulses before target to start slowing down

  bool running = true;

  while (running) {
    // Emergency stop check
    if (digitalRead(emergencyPin) == LOW && !emergencyTriggered && !emergencyHandled) {
      stop_motor_X();
      stop_motor_Y();
      return;
    }

    // Calculate error
    float error = (float)xPulseCount - (float)yPulseCount;
    integral += error;
    float derivative = error - prevError;

    // PID correction
    float correction = Kp * error + Ki * integral + Kd * derivative;

    // Adjust base PWM for acceleration/deceleration
    int targetPWM = maxPWM;

    // Acceleration phase
    if (pwmX < targetPWM) pwmX += accelStep;
    if (pwmY < targetPWM) pwmY += accelStep;

    // Deceleration phase — when either axis nears target
    if ((xTargetPulses - xPulseCount) <= decelDistance || (yTargetPulses - yPulseCount) <= decelDistance) {
      targetPWM = minPWM;  // slow down to stop
    }

    // Apply PID correction to keep in sync
    int adjPWM_X = constrain(pwmX - correction, minPWM, 255);
    int adjPWM_Y = constrain(pwmY + correction, minPWM, 255);

    run_motor_X(dirX, adjPWM_X);
    run_motor_Y(dirY, adjPWM_Y);

    prevError = error;

    // Stop when either axis finishes
    if (xPulseCount >= xTargetPulses || yPulseCount >= yTargetPulses) {
      stop_motor_X();
      stop_motor_Y();
      running = false;
    }

    delay(5);  // small delay for stability
  }

  delay(300);  // small pause after move
}



void disableMotors() {
  analogWrite(x_input1, 0);
  analogWrite(x_input2, 0);
  analogWrite(y_input1, 0);
  analogWrite(y_input2, 0);
  digitalWrite(sleep_pin, LOW);  // put motor driver to sleep
}

void enableMotors() {
  digitalWrite(sleep_pin, HIGH);  // wake up motor driver
}

void isolateServoMode() {
  // Set motor pins to INPUT (disconnect)
  pinMode(x_input1, INPUT);
  pinMode(x_input2, INPUT);
  pinMode(y_input1, INPUT);
  pinMode(y_input2, INPUT);

  // Disable motor driver (DRV8833 sleep)
  digitalWrite(sleep_pin, LOW);

  // Disable encoder interrupts
  detachInterrupt(digitalPinToInterrupt(x_encoder));
  detachInterrupt(digitalPinToInterrupt(y_encoder));
}

void restoreMotorMode() {
  // Re-enable motor pins
  pinMode(x_input1, OUTPUT);
  pinMode(x_input2, OUTPUT);
  pinMode(y_input1, OUTPUT);
  pinMode(y_input2, OUTPUT);

  // Wake motor driver
  digitalWrite(sleep_pin, HIGH);

  // Reattach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(x_encoder), xEncoderISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(y_encoder), yEncoderISR, FALLING);
}

void penDown() {
  isolateServoMode();
  penServo.attach(penServoPin);
  penServo.write(90);
  delay(1000);
  penServo.detach();
  restoreMotorMode();
}

void penUp() {
  isolateServoMode();
  penServo.attach(penServoPin);
  penServo.write(120);
  delay(1000);
  penServo.detach();
  restoreMotorMode();
}

void homeAxes() {

  penUp();
  run_motor_Y(false, 255);
  while (digitalRead(yLimitSwitchPin) == HIGH) { yield(); }
  stop_motor_Y();
  delay(500);

  run_motor_X(true, 255);
  while (digitalRead(xLimitSwitchPin) == HIGH) { yield(); }
  stop_motor_X();
  delay(500);
}

void handleEmergency() {
  emergencyTriggered = true;
  emergencyHandled = true;
  Serial.println("Emergency stop triggered!");

  stop_motor_X();
  stop_motor_Y();

  digitalWrite(ledGreen, LOW);
  digitalWrite(ledRed, HIGH);

  delay(1000);
  homeAxes();
  digitalWrite(ledRed, LOW);

  Serial.println("Recovery completed. System ready again.");
}

void setup() {
  Serial.begin(115200);

  pinMode(sleep_pin, OUTPUT);

  pinMode(x_input1, OUTPUT);
  pinMode(x_input2, OUTPUT);
  pinMode(x_encoder, INPUT);

  pinMode(y_input1, OUTPUT);
  pinMode(y_input2, OUTPUT);
  pinMode(y_encoder, INPUT);

  pinMode(ledGreen, OUTPUT);
  pinMode(ledRed, OUTPUT);
  digitalWrite(ledGreen, LOW);
  digitalWrite(ledRed, LOW);

  pinMode(emergencyPin, INPUT_PULLUP);
  pinMode(xLimitSwitchPin, INPUT_PULLUP);
  pinMode(yLimitSwitchPin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(x_encoder), xEncoderISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(y_encoder), yEncoderISR, FALLING);

  penUp();

  Serial.println("Homing axes...");
  homeAxes();
  Serial.println("System ready. Type 'START' to begin.");
}

void loop() {
  if (digitalRead(emergencyPin) == LOW && !emergencyTriggered && !emergencyHandled) {
    digitalWrite(ledRed, HIGH);
    handleEmergency();
  }

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "Y+") moveY(0.5, true);
    else if (cmd == "Y-") moveY(0.5, false);
    else if (cmd == "X+") moveX(0.5,true );
    else if (cmd == "X-") moveX(0.5,false );

    else if (cmd == "DIAG_UL") moveDiagonalPID(0.5, true, 0.5, false);
    else if (cmd == "DIAG_UR") moveDiagonalPID(0.5, true, 0.5, true);
    else if (cmd == "DIAG_DL") moveDiagonalPID(0.5, false, 0.5, false);
    else if (cmd == "DIAG_DR") moveDiagonalPID(0.5, false, 0.5, true);

    else if (cmd == "PEN_UP") penUp();
    else if (cmd == "PEN_DOWN") penDown();

    else if (cmd == "HOME") homeAxes();
    else if (cmd == "EMERGENCY") handleEmergency();
    else if (cmd == "NAME") {
      Serial.println("Sequence started");
      
      digitalWrite(ledGreen, HIGH);
      //moveY(1.0, true);
      moveX(1.5, false);

      penDown();

      moveX(0.5, false);
      moveDiagonalPID(0.5, true, 0.5, true);
      moveX(0.5, false);
      
      delay(500);
      penUp();
      moveY(0.1, true);
      penDown();
      moveY(0.5, true);
      moveY(0.25, false);
      moveX(0.5, true);
      moveY(0.25, false);
      moveY(0.55, true);

      delay(500);
      penUp();
      moveY(0.1, true);
      penDown();
      moveX(0.55, false);
      moveX(0.25, true);
      moveY(0.25,true);
      moveX(0.25,false);
      moveX(0.5, true);

      delay(500);
      penUp();
      moveY(0.1, true);
      penDown();
      moveDiagonalPID(0.45,true, 0.45, false);
      moveDiagonalPID(0.45,true, 0.45, true);
      penUp();
      moveDiagonalPID(0.29,false, 0.25, false);
      penDown();
      //moveDiagonal(0.25,false, 0.25, false);
      moveY(0.33,false);
      //penUp();
      //moveY(0.3,true);
      //moveDiagonal(0.25,true, 0.25, true);

      delay(500);
      penUp();
      moveY(0.33,true);
      moveDiagonalPID(0.30,true, 0.25, true);
      moveX(0.5, false);
      penDown();
      //moveX(0.5, false);
      moveX(0.5,true);
      moveY(0.5,true);
    
      //moveY(1.0, true);
      // moveX(1.0, true);
      // moveY(1.0, false);
      // moveDiagonal(1.05, true, 1.05, false);
      // moveDiagonal(0.5, false, 0.5, false);
      // moveDiagonal(0.5, false, 0.5, true);
      // moveDiagonal(1.05, true, 1.05, true);
      penUp();
      delay(1000);
      Serial.println("Homing axes again...");
      homeAxes();
      digitalWrite(ledGreen, LOW);
      Serial.println(" Done.");
    }

    else if (cmd == "NIKOLAUS" && !emergencyTriggered) {
      emergencyHandled = false;  // Allow emergency again after START
      Serial.println("Sequence started");

      digitalWrite(ledGreen, HIGH);
      moveY(1.0, true);
      moveX(1.0, false);

      penDown();

      //first draws square

      moveX(1.0, false);
      moveY(1.08, true);
      moveX(1.0, true);
      moveY(1.08, false);

      delay(300);
      moveDiagonalPID(1.1, true, 1.1, false);
      delay(300);
      moveDiagonalPID(0.50, false, 0.50, false);
      delay(300);
      moveDiagonalPID(0.50, false, 0.50, true);
      delay(300);
      moveDiagonalPID(1.1, true, 1.1, true);

      // starts from a diagonal

      // delay(300);
      // moveDiagonalPID(1.06, true, 1.06, false);
      // delay(300);
      // moveX(1.0, true);
      // delay(300);
      // moveY(1.0, false);
      // delay(300);
      // moveX(1.0, false);
      // delay(300);
      // moveY(1.0, true);
      // delay(300);
      // moveDiagonalPID(0.5, false, 0.5, false);
      // delay(300);
      // moveDiagonalPID(0.5, false, 0.5, true);
      // delay(300);
      // moveDiagonalPID(1.06, true, 1.06, true);


      delay(1000);
      penUp();
      delay(1000);
      Serial.println("Homing axes again...");
      homeAxes();
      digitalWrite(ledGreen, LOW);
      Serial.println(" Done.");
    }
  }
}
