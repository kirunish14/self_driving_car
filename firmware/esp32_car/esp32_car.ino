/*
 * ESP32 motor + steering driver for the Ackermann kids ride-on car.
 *
 *  - 4 DC motors on ONE L298N:  front pair on channel A, rear pair on channel B.
 *    Both channels always get the SAME command (no differential steering).
 *  - Front steering servo (25 kg): 97 deg = centre, 77 deg = full left, 117 deg = full right.
 *  - Talks to the computer over the USB cable (115200 baud).
 *
 *  Command line from ROS:   C,<speed>,<steer>\n
 *      speed : -1.0 .. +1.0   (+ = forward)
 *      steer : -1.0 .. +1.0   (+ = LEFT, - = RIGHT)
 *
 *  SAFETY: if no valid command arrives for 400 ms the motors are switched OFF.
 *
 *  Libraries: "ESP32Servo" (Arduino Library Manager).  Works with ESP32 Arduino core 2.x and 3.x.
 *
 *  WIRING (change the pins below if you wired differently)
 *     L298N ENA -> GPIO25   IN1 -> GPIO26   IN2 -> GPIO27      (front motors)
 *     L298N ENB -> GPIO33   IN3 -> GPIO14   IN4 -> GPIO13      (rear  motors)
 *     Servo signal -> GPIO18
 *     REMOVE the two ENA/ENB jumpers on the L298N, otherwise PWM has no effect.
 *     Servo needs its OWN 5-6 V supply (BEC) that can give >= 3 A. Do NOT power it from the ESP32.
 *     ALL grounds (battery, L298N, servo supply, ESP32) must be connected together.
 */
#include <ESP32Servo.h>

// ---------------- pins ----------------
const int PIN_ENA = 25, PIN_IN1 = 26, PIN_IN2 = 27;   // front motors
const int PIN_ENB = 33, PIN_IN3 = 14, PIN_IN4 = 13;   // rear motors
const int PIN_SERVO = 18;

// If a motor pair spins BACKWARDS when it should go forward, flip its flag (or swap the wires)
const bool INVERT_A = false;
const bool INVERT_B = false;

// ---------------- servo ----------------
const int SERVO_CENTER = 97;
const int SERVO_LEFT   = 77;
const int SERVO_RIGHT  = 117;

// ---------------- motors ----------------
const int PWM_FREQ  = 2000;     // Hz
const int PWM_BITS  = 8;        // 0..255
const int MIN_PWM   = 70;       // smallest duty that really moves the car (tune: raise if it hums but does not move)
const float DEADZONE = 0.02f;   // |speed| below this = stop
const unsigned long CMD_TIMEOUT_MS = 400;

#if ESP_ARDUINO_VERSION_MAJOR < 3
const int CH_A = 14, CH_B = 15;  // LEDC channels (ESP32Servo takes the low ones)
#endif

Servo steering;
unsigned long lastCmdMs = 0;
unsigned long lastStatusMs = 0;
float curSpeed = 0, curSteer = 0;
String line;

void pwmBegin(int pin, int ch) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, PWM_FREQ, PWM_BITS);
#else
  ledcSetup(ch, PWM_FREQ, PWM_BITS);
  ledcAttachPin(pin, ch);
#endif
}

void pwmWrite(int pin, int ch, int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(ch, duty);
#endif
}

void setChannel(int en, int chn, int in1, int in2, float speed, bool invert) {
  if (invert) speed = -speed;
  if (fabs(speed) < DEADZONE) {              // stop (coast)
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    pwmWrite(en, chn, 0);
    return;
  }
  int duty = MIN_PWM + (int)((255 - MIN_PWM) * min(1.0f, fabs(speed)));
  digitalWrite(in1, speed > 0 ? HIGH : LOW);
  digitalWrite(in2, speed > 0 ? LOW : HIGH);
  pwmWrite(en, chn, duty);
}

void driveAll(float speed) {
  // BOTH channels get exactly the same command
  #if ESP_ARDUINO_VERSION_MAJOR >= 3
    setChannel(PIN_ENA, 0, PIN_IN1, PIN_IN2, speed, INVERT_A);
    setChannel(PIN_ENB, 0, PIN_IN3, PIN_IN4, speed, INVERT_B);
  #else
    setChannel(PIN_ENA, CH_A, PIN_IN1, PIN_IN2, speed, INVERT_A);
    setChannel(PIN_ENB, CH_B, PIN_IN3, PIN_IN4, speed, INVERT_B);
  #endif
}

void steer(float s) {                         // s: -1 (right) .. +1 (left)
  s = constrain(s, -1.0f, 1.0f);
  float angle = (s >= 0) ? SERVO_CENTER + s * (SERVO_LEFT - SERVO_CENTER)
                         : SERVO_CENTER + (-s) * (SERVO_RIGHT - SERVO_CENTER);
  steering.write((int)constrain(angle, SERVO_LEFT, SERVO_RIGHT));
}

void handleLine(const String &l) {
  if (!l.startsWith("C,")) return;
  int c1 = l.indexOf(',', 2);
  if (c1 < 0) return;
  float sp = l.substring(2, c1).toFloat();
  float st = l.substring(c1 + 1).toFloat();
  curSpeed = constrain(sp, -1.0f, 1.0f);
  curSteer = constrain(st, -1.0f, 1.0f);
  lastCmdMs = millis();
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  pwmBegin(PIN_ENA, 0); pwmBegin(PIN_ENB, 0);
#else
  pwmBegin(PIN_ENA, CH_A); pwmBegin(PIN_ENB, CH_B);
#endif
  driveAll(0);

  steering.setPeriodHertz(50);
  steering.attach(PIN_SERVO, 500, 2400);
  steer(0);                                   // centre (97 deg)

  line.reserve(48);
  Serial.println("READY");
}

void loop() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n') { handleLine(line); line = ""; }
    else if (ch != '\r' && line.length() < 40) line += ch;
    else if (line.length() >= 40) line = "";
  }

  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {   // WATCHDOG: lost the computer -> stop
    curSpeed = 0;
  }

  driveAll(curSpeed);
  steer(curSteer);

  if (millis() - lastStatusMs > 500) {
    lastStatusMs = millis();
    Serial.printf("S,%.2f,%.2f\n", curSpeed, curSteer);
  }
  delay(5);
}
