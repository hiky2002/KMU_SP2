#include <Servo.h>

// Arduino pin assignment
#define PIN_LED   9   // LED active-low
#define PIN_TRIG  12  // sonar sensor TRIGGER
#define PIN_ECHO  13  // sonar sensor ECHO
#define PIN_SERVO 10  // servo motor

// configurable parameters for sonar
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25       // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 180.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 360.0   // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL) // coefficient to convert duration to distance

#define _EMA_ALPHA 0.6    // EMA weight of new sample (range: 0 to 1)

// Target Distance
#define _TARGET_LOW  180.0 // lower threshold distance in mm (18cm)
#define _TARGET_HIGH 360.0 // upper threshold distance in mm (36cm)

// duty duration for myservo.writeMicroseconds()
// NEEDS TUNING (servo by servo)
#define _DUTY_MIN 500     // servo full clockwise position (0 degree)
#define _DUTY_NEU 1500    // servo neutral position (90 degree)
#define _DUTY_MAX 2500    // servo full counterclockwise position (180 degree)

// global variables
float dist_ema, dist_prev = _DIST_MAX; // unit: mm
unsigned long last_sampling_time;      // unit: ms

Servo myservo;

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);    // sonar TRIGGER
  pinMode(PIN_ECHO, INPUT);     // sonar ECHO
  digitalWrite(PIN_TRIG, LOW);  // turn-off Sonar

  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);

  // initialize serial port
  Serial.begin(57600);

  // initialize USS related variables
  dist_prev = _DIST_MIN; // raw distance output from USS (unit: mm)
}

void loop() {
  float dist_raw;

  // wait until next sampling time
  if (millis() < (last_sampling_time + INTERVAL)) 
    return;

  // read the distance from the sonar sensor
  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);

  // Apply range filter
  if ((dist_raw == 0.0) || (dist_raw > _DIST_MAX)) {
    dist_raw = dist_prev; // Prevent servo from moving unexpectedly
    digitalWrite(PIN_LED, LOW); // LED OFF (out of range)
  } else if (dist_raw < _DIST_MIN) {
    dist_raw = dist_prev; // Prevent movement below min range
    digitalWrite(PIN_LED, LOW); // LED OFF
  } else { // Valid range
    dist_prev = dist_raw; // update previous distance
    digitalWrite(PIN_LED, HIGH); // LED ON
  }

  // Apply EMA filter to smooth out noise
  dist_ema = (_EMA_ALPHA * dist_raw) + ((1 - _EMA_ALPHA) * dist_prev);

  // adjust servo position based on the EMA-filtered distance
  if (dist_ema <= _TARGET_LOW) { 
    myservo.writeMicroseconds(_DUTY_MIN); // 0 degrees
  } else if (dist_ema >= _TARGET_HIGH) { 
    myservo.writeMicroseconds(_DUTY_MAX); // 180 degrees
  } else {
    // Map distance in range [_TARGET_LOW, _TARGET_HIGH] to servo angle [0, 180]
    int angle = map(dist_ema, _TARGET_LOW, _TARGET_HIGH, _DUTY_MIN, _DUTY_MAX);
    myservo.writeMicroseconds(angle);
  }

  // output the distance and servo values to the serial monitor
  Serial.print("Min:");    Serial.print(_DIST_MIN);
  Serial.print(", Low:");   Serial.print(_TARGET_LOW);
  Serial.print(", Dist EMA:");  Serial.print(dist_ema);
  Serial.print(", Servo:"); Serial.print(myservo.read());  
  Serial.print(", High:");  Serial.print(_TARGET_HIGH);
  Serial.print(", Max:");   Serial.print(_DIST_MAX);
  Serial.println("");

  // update last sampling time
  last_sampling_time += INTERVAL;
}

// Get a distance reading from the ultrasonic sensor, return in mm
float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm
}
