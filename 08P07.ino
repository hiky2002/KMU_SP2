#define PIN_LED  9
#define PIN_TRIG 12   // sonar sensor TRIGGER
#define PIN_ECHO 13   // sonar sensor ECHO

// configurable parameters
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25      // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 100.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300.0   // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL) // coefficent to convert duration to distance

unsigned long last_sampling_time;   // unit: msec

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);  // sonar TRIGGER
  pinMode(PIN_ECHO, INPUT);   // sonar ECHO
  digitalWrite(PIN_TRIG, LOW);  // turn-off Sonar 
  
  // initialize serial port
  Serial.begin(57600);
}

void loop() {
  float distance;

  // wait until next sampling time. 
  if (millis() < (last_sampling_time + INTERVAL))
    return;

  distance = USS_measure(PIN_TRIG, PIN_ECHO); // read distance

  // Distance to LED brightness conversion
  if (distance >= _DIST_MIN && distance <= _DIST_MAX) {
    int brightness = calculateBrightness(distance);
    analogWrite(PIN_LED, brightness);
  } else {
    analogWrite(PIN_LED, 255);  // Turn LED off for out of range distances
  }

  // output the distance to the serial port
  Serial.print("Min:");        Serial.print(_DIST_MIN);
  Serial.print(",distance:");  Serial.print(distance);
  Serial.print(",Max:");       Serial.print(_DIST_MAX);
  Serial.println("");
  
  // update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm
}

// Calculate the LED brightness based on the distance
int calculateBrightness(float distance) {
  if (distance <= 150.0) {
    // Map distance between 100 and 150 mm to LED duty cycle (255 to 128)
    return map(distance, 100, 150, 255, 128);
  } else if (distance <= 200.0) {
    // Map distance between 150 and 200 mm to LED duty cycle (128 to 0)
    return map(distance, 150, 200, 128, 0);
  } else if (distance <= 250.0) {
    // Map distance between 200 and 250 mm to LED duty cycle (0 to 128)
    return map(distance, 200, 250, 0, 128);
  } else {
    // Map distance between 250 and 300 mm to LED duty cycle (128 to 255)
    return map(distance, 250, 300, 128, 255);
  }
}
