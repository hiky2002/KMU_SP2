#include <Servo.h>

// Arduino pin assignment
#define PIN_IR    0     // IR sensor at Pin A0
#define PIN_LED   9
#define PIN_SERVO 10

#define _DUTY_MIN 500  // servo full clock-wise position (0 degree)
#define _DUTY_NEU 1500 // servo neutral position (90 degree)
#define _DUTY_MAX 2500 // servo full counter-clockwise position (180 degree)

#define _DIST_MIN  100.0 // minimum distance 100mm (10cm)
#define _DIST_MAX  250.0 // maximum distance 250mm (25cm)

#define EMA_ALPHA  0.5   // for EMA Filter

#define LOOP_INTERVAL 20 // Loop Interval (unit: msec)

Servo myservo;
unsigned long last_loop_time; // unit: msec

float dist_prev = _DIST_MIN;
float dist_ema = _DIST_MIN;

void setup()
{
  pinMode(PIN_LED, OUTPUT);
  
  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);
  
  Serial.begin(1000000);    // 1,000,000 bps
}

void loop()
{
  unsigned long time_curr = millis();
  int duty;
  float a_value, dist_raw;

  // wait until next event time
  if (time_curr < (last_loop_time + LOOP_INTERVAL))
    return;
  last_loop_time += LOOP_INTERVAL;

  a_value = analogRead(PIN_IR);
  dist_raw = (6762.0/(a_value-9)-4.0)*10.0 - 60.0;
  //dist_raw = ((6762.0 / (a_value - 9.0)) - 4.0) * 10.0;

  // 거리 값이 10~25cm 범위에 있을 때 LED 켜기 
  if (dist_raw >= _DIST_MIN && dist_raw <= _DIST_MAX) {
  // 거리 값이 10~25cm 범위에 있을 때 LED 켬
  digitalWrite(PIN_LED, LOW);  
} else {
  digitalWrite(PIN_LED, HIGH);   // 그 외의 경우 LED 끔
}

// EMA 필터 적용 (현재 코드에서 dist_ema를 필터링)
dist_ema = (EMA_ALPHA * dist_raw) + ((1 - EMA_ALPHA) * dist_prev);

// 서보 각도 계산 (거리 값에 비례하여 0도에서 180도까지)
float servo_angle = (dist_ema - _DIST_MIN) * 180.0 / (_DIST_MAX - _DIST_MIN);

// 서보에 PWM 신호를 보내서 위치 조정
duty = _DUTY_MIN + (servo_angle / 180.0) * (_DUTY_MAX - _DUTY_MIN);  // 서보 위치에 맞는 PWM 값 계산
myservo.writeMicroseconds(duty);  // 서보 모터에 PWM 신호를 보냄


  Serial.print("_DUTY_MIN:");  Serial.print(_DUTY_MIN);
  Serial.print("_DIST_MIN:");  Serial.print(_DIST_MIN);
  Serial.print(",IR:");        Serial.print(a_value);
  Serial.print(",dist_raw:");  Serial.print(dist_raw);
  Serial.print(",ema:");       Serial.print(dist_ema);
  Serial.print(",servo:");     Serial.print(duty);
  Serial.print(",_DIST_MAX:"); Serial.print(_DIST_MAX);
  Serial.print(",_DUTY_MAX:"); Serial.print(_DUTY_MAX);
  Serial.println("");
}
