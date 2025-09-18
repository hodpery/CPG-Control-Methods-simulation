#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo channels on PCA9685
const int HIP1 = 14, KNEE1 = 15, ANKLE1 = 13; // left
const int HIP2 = 1, KNEE2 = 0, ANKLE2 = 2;  // right


// Motion parameters
const float frequency_hip = 0.6;
const float frequency_knee = 0.6;
const float frequency_ankle = 1.2;

const float amplitude_hip = 11;
const float amplitude_knee = 63;
const float amplitude_ankle = 5;

const float phase_offsets[3] = { 0, 13 * M_PI / 20, 5*M_PI/6 };
const float phase_offset_leg2 = 0;

// Servo angle limits
const int hip_min = 60, hip_max = 180;
const int knee_min = 5, knee_max = 150;
const int ankle_min = 80, ankle_max = 100;

// Timer
unsigned long prevTime = 0;
const int updateInterval = 20;

// Converts angle in degrees to PCA9685 pulse
int angleToPulse(float angle) {
  float pulseLength = map(angle, 0, 180, 500, 2500); // us
  int pwmValue = int(pulseLength * 4096 / 20000);    // 20ms = 50Hz
  return constrain(pwmValue, 102, 512);              // ~500–2500 µs
}

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(50); // Standard analog servo frequency
  
  pwm.setPWM(HIP1, 0, angleToPulse(90));
  pwm.setPWM(KNEE1, 0, angleToPulse(0));
  pwm.setPWM(ANKLE1, 0, angleToPulse(90));

  pwm.setPWM(HIP2, 0, angleToPulse(90));
  pwm.setPWM(KNEE2, 0, angleToPulse(180));
  pwm.setPWM(ANKLE2, 0, angleToPulse(90));
  
  delay(3000);
}

void loop() {
  unsigned long currentTime = millis();
  float t = currentTime / 1000.0;

  if (currentTime - prevTime >= updateInterval) {
    prevTime = currentTime;

    float theta_hip1 = amplitude_hip * sin(2 * M_PI * frequency_hip * t + phase_offsets[0]);
    float theta_knee1 = amplitude_knee * sin(2 * M_PI * frequency_knee * t + phase_offsets[1]);
    float theta_ankle1 = -amplitude_ankle * sin(2 * M_PI * frequency_ankle * t + phase_offsets[2])-(-2*amplitude_ankle/3) * sin(M_PI * frequency_ankle * t + phase_offsets[2] + 0.6 * M_PI);

    float theta_hip2 = amplitude_hip * sin(2 * M_PI * frequency_hip * t + phase_offsets[0] + phase_offset_leg2);
    float theta_knee2 = -amplitude_knee * sin(2 * M_PI * frequency_knee * t + phase_offsets[1] + phase_offset_leg2);
    float theta_ankle2 = -amplitude_ankle * sin(2 * M_PI * frequency_ankle * t + phase_offsets[2])-(-2*amplitude_ankle/3) * sin(M_PI * frequency_ankle * t + phase_offset_leg2 + 0.6 * M_PI);

    int hip1_angle = constrain(90 + theta_hip1, hip_min, hip_max);
    int knee1_angle = constrain(theta_knee1, knee_min, knee_max)+constrain((amplitude_knee/6) * sin(2 * M_PI * frequency_knee * t + 1.45 * M_PI), 0, knee_max);
    int ankle1_angle = constrain(90 + theta_ankle1, ankle_min, ankle_max);

    int hip2_angle = constrain(90 + theta_hip2, 180 - hip_max, 180 - hip_min);
    int knee2_angle = constrain(180 - theta_knee2, 180 - knee_max, 180 - knee_min)+constrain((-amplitude_knee/6) * sin(2 * M_PI * frequency_knee * t + phase_offset_leg2 + 1.45 * M_PI), 180 - knee_max, 180);
    int ankle2_angle = constrain(90 + theta_ankle2, 180 - ankle_max, 180 - ankle_min);

    pwm.setPWM(HIP1, 0, angleToPulse(hip1_angle));
    pwm.setPWM(KNEE1, 0, angleToPulse(knee1_angle));
    pwm.setPWM(ANKLE1, 0, angleToPulse(ankle1_angle));

    pwm.setPWM(HIP2, 0, angleToPulse(hip2_angle));
    pwm.setPWM(KNEE2, 0, angleToPulse(knee2_angle));
    pwm.setPWM(ANKLE2, 0, angleToPulse(ankle2_angle));

  }
}
