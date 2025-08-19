//sleep pin directly connected to power

#include <ESP32Servo.h>

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

void run_motor_X(bool dir) {
  digitalWrite(x_input1, dir ? HIGH : LOW);
  digitalWrite(x_input2, dir ? LOW : HIGH);
}

void stop_motor_X() {
  digitalWrite(x_input1, LOW);
  digitalWrite(x_input2, LOW);
}

void run_motor_Y(bool dir) {
  digitalWrite(y_input1, dir ? HIGH : LOW);
  digitalWrite(y_input2, dir ? LOW : HIGH);
}

void stop_motor_Y() {
  digitalWrite(y_input1, LOW);
  digitalWrite(y_input2, LOW);
}

void moveX(float revolutions, bool dir) {
  if (digitalRead(emergencyPin) == LOW && !emergencyTriggered && !emergencyHandled) return;

  xPulseCount = 0;
  xTargetPulses = (unsigned long)(revolutions * pulsesPerRevolution);
  run_motor_X(dir);
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

void moveY(float revolutions, bool dir) {
  if (digitalRead(emergencyPin) == LOW && !emergencyTriggered && !emergencyHandled) return;

  yPulseCount = 0;
  yTargetPulses = (unsigned long)(revolutions * pulsesPerRevolution);
  run_motor_Y(dir);
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

void moveDiagonal(float revY, bool dirY, float revX, bool dirX) {
  xPulseCount = 0;
  yPulseCount = 0;

  xTargetPulses = (unsigned long)(revX * pulsesPerRevolution);
  yTargetPulses = (unsigned long)(revY * pulsesPerRevolution);

  run_motor_X(dirX);
  run_motor_Y(dirY);

  bool xDone = false;
  bool yDone = false;

  while (!xDone || !yDone) {
    if (digitalRead(emergencyPin) == LOW && !emergencyTriggered && !emergencyHandled) {
      stop_motor_X();
      stop_motor_Y();
      return;
    }

    if (!xDone && xPulseCount >= xTargetPulses) {
      stop_motor_X();
      xDone = true;
    }
    if (!yDone && yPulseCount >= yTargetPulses) {
      stop_motor_Y();
      yDone = true;
    }
    yield();
  }

  delay(300);
}

void penDown() {
  penServo.attach(penServoPin);
  penServo.write(90); // Pen down
  delay(1000);
  //penServo.detach();
}

void penUp() {
  penServo.attach(penServoPin);
  penServo.write(120); // Pen up
  delay(1000);
  penServo.detach();
}

void homeAxes() {
  penUp();

  // Home Y first (anticlockwise)
  run_motor_Y(false);
  while (digitalRead(yLimitSwitchPin) == HIGH) {
    yield();
  }
  stop_motor_Y();
  delay(500);

  // Then home X (clockwise)
  run_motor_X(true);
  while (digitalRead(xLimitSwitchPin) == HIGH) {
    yield();
  }
  stop_motor_X();
  delay(500);
}

void handleEmergency() {
  emergencyTriggered = true;
  emergencyHandled = true;

  Serial.println ("Emergency stop triggered!");

  stop_motor_X();
  stop_motor_Y();

  digitalWrite(ledGreen, LOW);   // Turn off green LED
  digitalWrite(ledRed, HIGH);    // Turn on red LED

  delay(1000);
  homeAxes();

  digitalWrite(ledRed, LOW);

  Serial.println(" Recovery completed. System ready again.");
}

void setup() {
  Serial.begin(115200);

  pinMode(x_input1, OUTPUT);
  pinMode(x_input2, OUTPUT);
  pinMode(x_encoder, INPUT);

  pinMode(y_input1, OUTPUT);
  pinMode(y_input2, OUTPUT);
  pinMode(y_encoder, INPUT);
  
  pinMode(ledGreen, OUTPUT);
  digitalWrite(ledGreen, LOW);
  //pinMode(ledRed, OUTPUT);
  //digitalWrite(ledRed, LOW);  // Ensure red LED is OFF at startup


  pinMode(emergencyPin, INPUT_PULLUP);
  pinMode(xLimitSwitchPin, INPUT_PULLUP);
  pinMode(yLimitSwitchPin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(x_encoder), xEncoderISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(y_encoder), yEncoderISR, FALLING);

  penUp();

  Serial.println(" Homing axes...");
  homeAxes();

  Serial.println("System ready. Type 'START' to begin.");
}

void loop() {
  if (digitalRead(emergencyPin) == LOW && !emergencyTriggered && !emergencyHandled) {
    digitalWrite(ledRed, HIGH);
    handleEmergency();
  }
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "START" && !emergencyTriggered) {
      emergencyHandled = false;  // Allow emergency again after START
      Serial.println("Sequence started");
      
      digitalWrite(ledGreen, HIGH);
      moveY(1.0, true);
      moveX(1.0, false);

      penDown();

      moveX(1.0, false);
      moveY(1.0, true);
      moveX(1.0, true);
      moveY(1.0, false);
      moveDiagonal(1.05, true, 1.05, false);
      moveDiagonal(0.5, false, 0.5, false);
      moveDiagonal(0.5, false, 0.5, true);
      moveDiagonal(1.05, true, 1.05, true);
      penUp();
      delay(1000);
      Serial.println("Homing axes again...");
      homeAxes();
      digitalWrite(ledGreen, LOW);
      Serial.println(" Done.");
    }
    else if (command == "HOME") {
      Serial.println("Homing...");
      digitalWrite(ledGreen, HIGH);
      homeAxes();
      digitalWrite(ledGreen, LOW);
      Serial.println("Homing complete.");
    }

    else if (command == "XLINE_FORWARD") {
      Serial.println(" Moving X forward");
      digitalWrite(ledGreen, HIGH);
      moveY(1.0, true);
      moveX(1.0, false);
      penDown();
      moveX(1.0, true);  // Adjust as needed
      penUp();
      digitalWrite(ledGreen, LOW);
    }

    else if (command == "XLINE_BACKWARD") {
      Serial.println(" Moving X backward");
      digitalWrite(ledGreen, HIGH);
      moveY(1.0, true);
      moveX(1.0, false);
      penDown();
      moveX(1.0, false);
      penUp();
      digitalWrite(ledGreen, LOW);
    }

    else if (command == "YLINE_FORWARD") {
      Serial.println("Moving Y forward");
      digitalWrite(ledGreen, HIGH);
      moveY(1.0, true);
      moveX(1.0, false);
      penDown();
      moveY(1.0, true);
      penUp();
      digitalWrite(ledGreen, LOW);
    }

    else if (command == "YLINE_BACKWARD") {
      Serial.println(" Moving Y backward");
      digitalWrite(ledGreen, HIGH);
      moveY(1.0, true);
      moveX(1.0, false);
      penDown();
      moveY(1.0, false);
      penUp();
      digitalWrite(ledGreen, LOW);
    }

  }
}
