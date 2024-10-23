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

  rotateMotor(0,0);
}

void loop() {
  int distance;
  short int threshold = 200;    // Adjust based on sensors behavior

  distance = getDistance(trigPin, echoPin);
  
  if (distance <= 16 ){
    // STOP MOTORS
    rotateMotor(0, 0);

  } else if ((isLeftHigh(threshold, sl) && !isRightHigh(threshold, sr))) {
    // TURN LEFT
  
    rotateMotor(MOTOR_SPEED, -MOTOR_SPEED); 
    
  } else if ((isRightHigh(threshold, sr) && !isLeftHigh(threshold, sl)) ) {
    // TURN RIGHT

    rotateMotor(-MOTOR_SPEED, MOTOR_SPEED);

  } else if ((!isRightHigh(threshold, sr) && !isLeftHigh(threshold, sl))) {
    // GO STRAIGHT: Both are low

    rotateMotor(MOTOR_SPEED, MOTOR_SPEED);

  } else if ((isRightHigh(threshold, sr) && isLeftHigh(threshold, sl))) {
    // FINISH LINE: Both are high, stop motors

    rotateMotor(0, 0);
    
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

/*____________________________________________________________________________________________________________________*/
