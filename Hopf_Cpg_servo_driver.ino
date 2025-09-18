// File: hopf_cpg_pca9685.ino
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// --- Servo Channel Mapping ---
const int HIP1 = 14, KNEE1 = 15, ANKLE1 = 13; // left
const int HIP2 = 1, KNEE2 = 0, ANKLE2 = 2;  // right

// --- Oscillator Parameters ---
const float mu = 0.8;
const float omega = 1.1 * M_PI;
const float gamma_coupling = 0.1;
const float dt = 0.01;

struct Complex {
  float real;
  float imag;
};

Complex z[6];

const float C[6][6] = {
  { 0,  0,  0, -1,  0,  0},
  { 0,  0,  0,  0, -1,  0},
  { 0,  0,  0,  0,  0, -1},
  {-1, 0,  0,  0,  0,  0},
  { 0, -1, 0,  0,  0,  0},
  { 0,  0, -1, 0,  0,  0}
};

Complex multiplyComplex(Complex z1, Complex z2) {
  Complex result;
  result.real = z1.real * z2.real - z1.imag * z2.imag;
  result.imag = z1.real * z2.imag + z1.imag * z2.real;
  return result;
}
// --- Joint Limits (in degrees) ---
const int hip_min = 60, hip_max = 180;
const int knee_min = 5, knee_max = 150;
const int ankle_min = 80, ankle_max = 100; //corrected

// --- phase for ankle ---
const int amp_ankle = 5;
const float phase_offset = 13 * M_PI / 6;
Complex offset[3];


// --- Timer ---
unsigned long prevTime = 0;

// Converts angle in degrees to PCA9685 PWM value
int angleToPulse(float angle) {
  float pulse_us = map(angle, 0, 180, 500, 2500);
  return int(pulse_us * 4096 / 20000);
}

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(50);

  pwm.setPWM(HIP1, 0, angleToPulse(90));
  pwm.setPWM(KNEE1, 0, angleToPulse(0));
  pwm.setPWM(ANKLE1, 0, angleToPulse(90));

  pwm.setPWM(HIP2, 0, angleToPulse(90));
  pwm.setPWM(KNEE2, 0, angleToPulse(180));
  pwm.setPWM(ANKLE2, 0, angleToPulse(90));
  
  delay(3000);

  // Initialize oscillators
  for (int i = 0; i < 6; i++) {
    z[i].real = 1.0;
    z[i].imag = 1.0;
  }

  // Phase shift for second leg
  z[3].real = -1.0; z[3].imag = -1.0;
  z[4].real = -1.0; z[4].imag = -1.0;
  z[5].real = -1.0; z[5].imag = -1.0;

  // Knee phase adjustment
  z[1].real = cos(1.2 * M_PI / 2);
  z[1].imag = sin(1.2 * M_PI / 2);
  z[4].real = cos(1.2 * M_PI / 2);
  z[4].imag = sin(1.2 * M_PI / 2);

  // ankle phase adjustment
  z[2].real = cos(-1.2 * M_PI / 2);
  z[2].imag = sin(-1.2 * M_PI / 2);
  z[5].real = cos(-1.2 * M_PI / 2);
  z[5].imag = sin(-1.2 * M_PI / 2);

  offset[0].real = cos(phase_offset);
  offset[0].imag = sin(phase_offset);
  offset[1].real = cos(phase_offset + 0.6 * M_PI);
  offset[1].imag = sin(phase_offset + 0.6 * M_PI);
  offset[2].real = cos(0.9 * M_PI);
  offset[2].imag = sin(0.9 * M_PI);
  
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - prevTime >= (dt * 1000)) {
    prevTime = currentTime;

    Complex z_new[6];

    for (int i = 0; i < 6; i++) {
      Complex coupling = {0.0, 0.0};
      for (int j = 0; j < 6; j++) {
        coupling.real += C[i][j] * z[j].real;
        coupling.imag += C[i][j] * z[j].imag;
      }

      float r2 = z[i].real * z[i].real + z[i].imag * z[i].imag;

      Complex dz;
      dz.real = (mu - r2) * z[i].real - omega * z[i].imag + gamma_coupling * coupling.real;
      dz.imag = (mu - r2) * z[i].imag + omega * z[i].real + gamma_coupling * coupling.imag;

      z_new[i].real = z[i].real + dt * dz.real;
      z_new[i].imag = z[i].imag + dt * dz.imag;
    }

    for (int i = 0; i < 6; i++) {
      z[i] = z_new[i];
    }

    float theta_hip1   = -11.0 * z[0].real;
    float theta_knee1  = -63.0 * z[1].real;
    float base = -amp_ankle *multiplyComplex(z[2], offset[0]).real;
    float harm = -(amp_ankle/0.8) * multiplyComplex(multiplyComplex(z[2],z[2]), offset[1]).real;
    float theta_ankle1 = base + harm;

    float theta_hip2   =  11.0 * z[3].real;
    float theta_knee2  =  63.0 * z[4].real;
    float base2 = -amp_ankle * multiplyComplex(z[5], offset[0]).real;
    float harm2 = -(amp_ankle/0.8) * multiplyComplex(multiplyComplex(z[5],z[5]), offset[1]).real;
    float theta_ankle2 = base2 + harm2;

    int hip1_angle   = constrain(90 + theta_hip1, hip_min, hip_max);
    int knee1_angle  = constrain(theta_knee1, knee_min, knee_max)+ constrain(-10.5 *  multiplyComplex(z[1], offset[2]).real, 0, knee_max);
    int ankle1_angle = constrain(90 + theta_ankle1, ankle_min, ankle_max);

    int hip2_angle   = constrain(90 + theta_hip2, 180 - hip_max, 180 - hip_min);
    int knee2_angle  = constrain(180 - theta_knee2, 180 - knee_max, 180 - knee_min)+ constrain(10.5 * multiplyComplex(z[4], offset[2]).real, 180 - knee_max, 180);
    int ankle2_angle = constrain(90 + theta_ankle2, 180 - ankle_max, 180 - ankle_min);

    pwm.setPWM(HIP1, 0, angleToPulse(hip1_angle));
    pwm.setPWM(KNEE1, 0, angleToPulse(knee1_angle));
    pwm.setPWM(ANKLE1, 0, angleToPulse(ankle1_angle));

    pwm.setPWM(HIP2, 0, angleToPulse(hip2_angle));
    pwm.setPWM(KNEE2, 0, angleToPulse(knee2_angle));
    pwm.setPWM(ANKLE2, 0, angleToPulse(ankle2_angle));

  }
  
}
