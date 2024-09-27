#define PIN_LED 7 // PIN_LED를 7번 핀으로 정의 
unsigned int count, toggle; // count와 toggle을 부호없는 정수 타입으로 전역 변수 선언

void setup() {
  pinMode(PIN_LED, OUTPUT); // 7번 핀을 출력모드로 설정
  Serial.begin(115200); // 시리얼 통신 시작 1152000bps
  while(!Serial) {
    ; 
  }
    // 시리얼 포트 연결 대기
}

void loop() {
  // 무한 루프에 진입
  while(1) {
    digitalWrite(PIN_LED, 0); // LED 켜기
    delay(1000); // 1초 대기
  
    for (int i = 0; i<5; i++) {
      // LED를 5번 깜빡이는 반복문
      digitalWrite(PIN_LED, 1); // LED 끔 
      delay(100);               // 100ms 대기
      digitalWrite(PIN_LED, 0); // LED 켬 
      delay(100);               // 100ms 대기  
    }

     digitalWrite(PIN_LED, 1); // LED 끔

     while(1) {
      
     } // 이후 더 이상 반복 x
  }
 
}
