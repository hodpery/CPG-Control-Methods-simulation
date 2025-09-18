// File: matsuoka_cpg_pca9685.ino
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int HIP1 = 14, KNEE1 = 15, ANKLE1 = 13;
const int HIP2 = 1, KNEE2 = 0, ANKLE2 = 2;

const float tau_r = 0.30;
const float tau_a = 0.30;
const float beta = 2.5;
const float w_inhibit = 2.0;
const float s_input = 2.2;
const float k_leg = 0.1;

const float dt = 0.016;
const int buffer_size = 900;  // for arduino: buffer_size = 1
const int virtual_steps = 900;
const int delay_steps = int(1.4 * virtual_steps / (15.0 * 2));
const int mod_delay_steps = int(1.35 * virtual_steps / 15.0);

int t_counter = 0;

float osc[3][4];
float outputs[3];
float Couple[3][3] = {
  {0.0, k_leg, 0.0},
  {k_leg, 0.0, k_leg},
  {0.0, k_leg, 0.0}
};

float delay_buffer[3][buffer_size];
float mod_buffer[buffer_size];

unsigned long prevTime = 0;

int angleToPulse(float angle) {
  float pulse_us = map(angle, 0, 180, 500, 2500);
  return int(pulse_us * 4096 / 20000);
}

void setup() {
  pwm.begin();
  pwm.setPWMFreq(50);

  pwm.setPWM(HIP1, 0, angleToPulse(90));
  pwm.setPWM(KNEE1, 0, angleToPulse(0));
  pwm.setPWM(ANKLE1, 0, angleToPulse(90));

  pwm.setPWM(HIP2, 0, angleToPulse(90));
  pwm.setPWM(KNEE2, 0, angleToPulse(180));
  pwm.setPWM(ANKLE2, 0, angleToPulse(90));

  delay(3000);

  osc[0][0] = 0.5002; osc[0][1] = 0.1498; osc[0][2] = 0.4048; osc[0][3] = 0.0660;
  osc[1][0] = 0.6993; osc[1][1] = -0.0150; osc[1][2] = 0.3698; osc[1][3] = 0.2968;
  osc[2][0] = 0.5260; osc[2][1] = 0.1384; osc[2][2] = 0.4172; osc[2][3] = 0.0648;

  Serial.begin(115200);
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - prevTime >= (dt * 1000)) {
    prevTime = currentTime;

    float new_outputs[3];
    float osc_new[3][4];

    for (int j = 0; j < 3; j++) {
      float coupling_input = 0.0;
      for (int k = 0; k < 3; k++) {
        coupling_input += Couple[j][k] * outputs[k];
      }
      matsuoka_step(osc[j], coupling_input, osc_new[j], new_outputs[j]);
    }

    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 4; k++) {
        osc[j][k] = osc_new[j][k];
      }
      outputs[j] = new_outputs[j];
    }

    static int buffer_idx = 0;
    for (int j = 0; j < 3; j++) {
      delay_buffer[j][buffer_idx] = outputs[j];
    }
    mod_buffer[buffer_idx] = outputs[2] * outputs[2];

    buffer_idx = (buffer_idx + 1) % buffer_size;
    t_counter = (t_counter + 1) % virtual_steps;
    int idx_delay = ((t_counter - delay_steps) % virtual_steps) + 1;
    int idx_mod = ((t_counter - mod_delay_steps) % virtual_steps) + 1;

    float amp_ankle = PI / 36.0;
    float base = amp_ankle * outputs[2] - radians(3.2);
    float mod_term = (amp_ankle / 0.3) * mod_buffer[idx_mod] - radians(3.2);
    float theta_ankle1 = constrain(-base, radians(-10), radians(10)) + constrain(-mod_term, radians(-10), radians(10));

    float theta_hip1 = 1.1 * PI / 18.0 * outputs[0];
    float theta_knee1 = -7 * PI / 20.0 * outputs[1];

    float theta_hip2 = 1.1 * PI / 18.0 * outputs[0]; //delay_buffer[0][idx_delay];
    float theta_knee2 = 7 * PI / 20.0 * outputs[1]; //delay_buffer[1][idx_delay];

    float base2 = amp_ankle * outputs[2] - radians(3.2); //amp_ankle * delay_buffer[2][idx_delay] - radians(3.2);
    int idx_mod2 = (idx_delay + virtual_steps - mod_delay_steps) % virtual_steps;
    float mod_term2 = (amp_ankle / 0.3) * mod_buffer[idx_mod] - radians(3.2);
    float theta_ankle2 = constrain(base2, radians(-10), radians(10)) + constrain(mod_term2, radians(-10), radians(10));

    theta_hip1 = constrain(theta_hip1, radians(-30), radians(100));
    theta_knee1 = constrain(theta_knee1, radians(5), radians(150)) + constrain(theta_knee2 / 6.0, radians(0), radians(150));

    theta_hip2 = constrain(theta_hip2, radians(-30), radians(100));
    theta_knee2 = constrain(theta_knee2, radians(5), radians(150)) + constrain(theta_knee1 / 6.0, radians(0), radians(150));

    int hip1_angle = constrain(90 + degrees(theta_hip1), 60, 180);
    int knee1_angle = constrain(degrees(theta_knee1), 5, 150);
    int ankle1_angle = constrain(90 + degrees(theta_ankle1), 70, 110);

    int hip2_angle = constrain(90 + degrees(theta_hip2), 0, 120);
    int knee2_angle = constrain(180 - degrees(theta_knee2), 30, 175);
    int ankle2_angle = constrain(90 + degrees(theta_ankle2), 70, 110);

    pwm.setPWM(HIP1, 0, angleToPulse(hip1_angle));
    pwm.setPWM(KNEE1, 0, angleToPulse(knee1_angle));
    pwm.setPWM(ANKLE1, 0, angleToPulse(ankle1_angle));

    pwm.setPWM(HIP2, 0, angleToPulse(hip2_angle));
    pwm.setPWM(KNEE2, 0, angleToPulse(knee2_angle));
    pwm.setPWM(ANKLE2, 0, angleToPulse(ankle2_angle));
  }
}

void matsuoka_step(float state[4], float coupling, float next_state[4], float &output) {
  float u1 = state[0], u2 = state[1], v1 = state[2], v2 = state[3];
  float y1 = max(0.0f, u1);
  float y2 = max(0.0f, u2);

  float du1 = (-u1 - w_inhibit * y2 - beta * v1 + s_input + coupling) / tau_r;
  float du2 = (-u2 - w_inhibit * y1 - beta * v2 + s_input + coupling) / tau_r;
  float dv1 = (y1 - v1) / tau_a;
  float dv2 = (y2 - v2) / tau_a;

  next_state[0] = u1 + dt * du1;
  next_state[1] = u2 + dt * du2;
  next_state[2] = v1 + dt * dv1;
  next_state[3] = v2 + dt * dv2;

  output = y1 - y2;
}
