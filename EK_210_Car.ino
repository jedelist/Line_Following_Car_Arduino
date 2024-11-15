const int trigPin = 9;    // Ultrasonic sensor
const int echoPin = 10;   // Ultrasonic sensor echoPin
const int sl = A0;        // LEFT IR sensor (using analog pin A0)
const int sr = A1;        // RIGHT IR sensor (using analog pin A1)

#define MOTOR_SPEED 180

//Right motor
int enableRightMotor=6;
int rightMotorPin1=7;
int rightMotorPin2=8;

//Left motor
int enableLeftMotor=5;
int leftMotorPin1=4;  // formerly 9
int leftMotorPin2=3;  // formerly 10

//LEDs
const int PIN_LED_RED = 12;
const int PIN_LED_GREEN = 13;

// Buzzer
const int BUZZER_PIN = 1; //buzzer to arduino pin 9


void setup() {
  pinMode(trigPin, OUTPUT);  // Set Trig (Ultrasonic)
  pinMode(echoPin, INPUT);   // Set Echo (Ultrasonic)
  pinMode(sl, INPUT);        // LEFT IR
  pinMode(sr, INPUT);        // RIGHT IR
  Serial.begin(9600);        // Initialize serial communication

  // Motors
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  
  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  //LEDS
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);

  // Buzzer
  pinMode(BUZZER_PIN, OUTPUT); // Set pin 1 as buzzer output

  rotateMotor(0,0);
}

/*____________________________________________________________________________________________________________________*/

void loop() {
  int distance;
  short int threshold = 200;    // Adjust based on sensors behavior

  distance = getDistance(trigPin, echoPin);
  
  if (distance <= 16 ){
    // STOP MOTORS
    rotateMotor(0, 0);
    // SOUND BUZZER

    tone(BUZZER_PIN, 1300, 200);  // Play a 1000 Hz tone for 200 ms
    delay(200);  // Pause briefly
    tone(BUZZER_PIN, 1000, 200);  // Play a 1200 Hz tone for 200 ms
    delay(200);  // Pause briefly
    noTone(BUZZER_PIN);           // Stop the tone

    // RED LED HIGH
    digitalWrite (PIN_LED_GREEN, LOW);
    digitalWrite (PIN_LED_RED, HIGH);

  } else if ((isLeftHigh(threshold, sl) && !isRightHigh(threshold, sr))) {
    // TURN LEFT
  
    rotateMotor(MOTOR_SPEED, -MOTOR_SPEED); 

    // GREEN LED HIGH
    digitalWrite (PIN_LED_GREEN, HIGH);
    digitalWrite (PIN_LED_RED, LOW);
    
  } else if ((isRightHigh(threshold, sr) && !isLeftHigh(threshold, sl)) ) {
    // TURN RIGHT

    rotateMotor(-MOTOR_SPEED, MOTOR_SPEED);

    // GREEN LED HIGH
    digitalWrite (PIN_LED_GREEN, HIGH);
    digitalWrite (PIN_LED_RED, LOW);

  } else if ((!isRightHigh(threshold, sr) && !isLeftHigh(threshold, sl))) {
    // GO STRAIGHT: Both are low

    rotateMotor(MOTOR_SPEED, MOTOR_SPEED);

    // GREED LED HIGH    
    digitalWrite (PIN_LED_GREEN, HIGH);
    digitalWrite (PIN_LED_RED, LOW);

  } else if ((isRightHigh(threshold, sr) && isLeftHigh(threshold, sl))) {
    // FINISH LINE: Both are high, stop motors

    rotateMotor(0, 0);

    // RED LED HIGH:
    digitalWrite (PIN_LED_GREEN, LOW);
    digitalWrite (PIN_LED_RED, HIGH);

    tone(BUZZER_PIN, 600, 200);  // Low tone for 200 ms
    delay(100);                  // Delay
    tone(BUZZER_PIN, 800, 200);  // Med tone for 200 ms
    delay(100);
    tone(BUZZER_PIN, 1000, 400); // High tone 400 ms 
    delay(200);
    tone(BUZZER_PIN, 1200, 200); // Higher tone for 200 ms
    delay(100);
    tone(BUZZER_PIN, 1000, 300); // Descend slightly for a smooth ending
    delay(300);
    noTone(BUZZER_PIN);          // Stop the tone
    
  } else {
    // Undefined behavior, catch this exception
    // PRINT ERROR STATEMENT
    Serial.println(" Notice: Undefined behavior detected");
  }

}

float getDistance(int trigPin, int echoPin) {
  long duration;
  float distance;
  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // pulse duration in microseconds
  duration = pulseIn(echoPin, HIGH);

  if (duration == 0) {
    return 9999;  // Return a large number to indicate no obstacle
  }

  distance = duration * 0.034 / 2;

  return distance; // Return the distance in cm
}

bool isLeftHigh (int threshold, int sl){

  int svl = analogRead(sl);  // Read the analog value 
  
  if (svl < threshold) {
    return true; // WHITE
  } else {
    return false;
  }
}

bool isRightHigh (int threshold, int sr){

  int svr = analogRead(sr);  // Read the analog value 
  
  if (svr < threshold) {
    return true; // WHITE
  } else {
    return false;
  }
}

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed)
{
  
  if (rightMotorSpeed < 0)
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,HIGH);    
  }
  else if (rightMotorSpeed > 0)
  {
    digitalWrite(rightMotorPin1,HIGH);
    digitalWrite(rightMotorPin2,LOW);      
  }
  else
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,LOW);      
  }

  if (leftMotorSpeed < 0)
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,HIGH);    
  }
  else if (leftMotorSpeed > 0)
  {
    digitalWrite(leftMotorPin1,HIGH);
    digitalWrite(leftMotorPin2,LOW);      
  }
  else 
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,LOW);      
  }
  analogWrite(enableRightMotor, abs(rightMotorSpeed));
  analogWrite(enableLeftMotor, abs(leftMotorSpeed));    
}

