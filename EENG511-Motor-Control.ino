#include <util/atomic.h>

// ENGR 511 Final Project


// Encoder Pins
#define ENCA 2   
#define ENCB 3   

// Motor driver pins
#define PWM 5    
#define IN1 6    
#define IN2 7    

// Encoder variables
volatile int encoderCount = 0;   

int previousCount = 0;
long previousTime = 0;
long previousPrintTime = 0;

// RPM variables
float rawRPM = 0.0;
float filteredRPM = 0.0;
float previousRawRPM = 0.0;

float countsPerRev = 600.0;



float b0 = 0.01546504;
float b1 = 0.01546504;
float a1 = 0.96906992;

// PID control variables

// Target motor speed in RPM
float targetRPM = 100.0;

// Tuning values
float Kp = 2.0;
float Ki = 0.05;
float Kd = 0.0;

float integralError = 0.0;
float previousError = 0.0;

void setup() {
  Serial.begin(115200);

  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);

  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

  previousTime = micros();

  Serial.println("Target_RPM Raw_RPM Filtered_RPM PWM");
}

void loop() {

  //Read encoder count safely
  int currentCount = 0;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    currentCount = encoderCount;
  }

  //Calculate elapsed time
  long currentTime = micros();
  float deltaTime = (currentTime - previousTime) / 1000000.0;

  if (deltaTime <= 0) {
    return;
  }

  previousTime = currentTime;

  //Calculate raw RPM
  int changeInCount = currentCount - previousCount;
  previousCount = currentCount;

  float countsPerSecond = changeInCount / deltaTime;

  rawRPM = (countsPerSecond / countsPerRev) * 60.0;

  //Apply low-pass filter
  filteredRPM = a1 * filteredRPM + b0 * rawRPM + b1 * previousRawRPM;
  previousRawRPM = rawRPM;

  //PID control
  float error = targetRPM - filteredRPM;

  integralError = integralError + error * deltaTime;

  float derivativeError = (error - previousError) / deltaTime;

  float controlSignal = Kp * error + Ki * integralError + Kd * derivativeError;

  previousError = error;

  //Convert PID output to PWM and direction
  int direction = 1;

  if (controlSignal < 0) {
    direction = -1;
  }

  int pwmValue = abs(controlSignal);

  if (pwmValue > 255) {
    pwmValue = 255;
  }

  //Send command to motor
  setMotor(direction, pwmValue);

  //Print data for Serial Plotter
  if (millis() - previousPrintTime >= 100) {
    previousPrintTime = millis();

    Serial.print(targetRPM);
    Serial.print(" ");
    Serial.print(rawRPM);
    Serial.print(" ");
    Serial.print(filteredRPM);
    Serial.print(" ");
    Serial.println(pwmValue);
  }

  delay(1);   
}

// Motor control function
void setMotor(int direction, int pwmValue) {
  analogWrite(PWM, pwmValue);

  if (direction == 1) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else if (direction == -1) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
}

// Encoder interrupt function
void readEncoder() {
  int b = digitalRead(ENCB);

  if (b > 0) {
    encoderCount--;
  }
  else {
    encoderCount++;
  }
}
