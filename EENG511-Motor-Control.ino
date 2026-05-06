#include <util/atomic.h>


// ENGR 511 Final Project
// Closed-loop DC motor speed control with encoder feedback




// Pin setup


// Encoder pins
#define ENCA 2   // Encoder Channel A
#define ENCB 3   // Encoder Channel B

// Motor driver pins
#define PWM 5    // PWM speed control pin
#define IN1 6    // Motor direction pin 1
#define IN2 7    // Motor direction pin 2


// Encoder variables
volatile int encoderCount = 0;   // Encoder count updated by interrupt

int previousCount = 0;
long previousTime = 0;
long previousPrintTime = 0;

// RPM variables
float rawRPM = 0.0;
float filteredRPM = 0.0;
float previousRawRPM = 0.0;


float countsPerRev = 600.0;


// Low-pass filter coefficients
// Cutoff frequency = 5 Hz
// Sampling frequency = 1000 Hz
// dt = 0.001 s
//
// Discrete update:
// y_filt[i] = a1*y_filt[i-1] + b0*y[i] + b1*y[i-1]

// y[i] = rawRPM
// y_filt[i] = filteredRPM

float b0 = 0.01546504;
float b1 = 0.01546504;
float a1 = 0.96906992;


// PID control variables


// Target motor speed in RPM
float targetRPM = 100.0;

// Tuning
// You can change these while tuning.
float Kp = 2.0;
float Ki = 0.05;
float Kd = 0.0;

float integralError = 0.0;
float previousError = 0.0;


// Setup
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


// Main loop
void loop() {

  // 1. Read encoder count safely
  int currentCount = 0;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    currentCount = encoderCount;
  }

  // 2. Calculate elapsed time
  long currentTime = micros();
  float deltaTime = (currentTime - previousTime) / 1000000.0;

  if (deltaTime <= 0) {
    return;
  }

  previousTime = currentTime;

  // 3. Calculate raw RPM
  int changeInCount = currentCount - previousCount;
  previousCount = currentCount;

  float countsPerSecond = changeInCount / deltaTime;

  rawRPM = (countsPerSecond / countsPerRev) * 60.0;

  // 4. Apply low-pass filter
  filteredRPM = a1 * filteredRPM + b0 * rawRPM + b1 * previousRawRPM;
  previousRawRPM = rawRPM;

  // 5. PID control
  float error = targetRPM - filteredRPM;

  integralError = integralError + error * deltaTime;

  float derivativeError = (error - previousError) / deltaTime;

  float controlSignal = Kp * error + Ki * integralError + Kd * derivativeError;

  previousError = error;

  // 6. Convert PID output to PWM and direction
  int direction = 1;

  if (controlSignal < 0) {
    direction = -1;
  }

  int pwmValue = abs(controlSignal);

  if (pwmValue > 255) {
    pwmValue = 255;
  }

  // 7. Send command to motor
  setMotor(direction, pwmValue);

  // 8. Print data for Serial Plotter
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

  delay(1);   // Approximately 1000 Hz sampling rate
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
