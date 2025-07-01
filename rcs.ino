#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

// Khai báo môi trường
#define HOST_HAS_LEGACY_LIQUIDCRYSTAL_LIBRARY true

LiquidCrystal_I2C lcd(0x27, 16, 2);

const int buttonPin[4] = {29, 30, 31, 18}; // RIGHT, RUN, LEFT, STOP
const int modeSpeed[3] = {70, 100, 130}; // LOW, MEDIUM, HIGH
const int buttonNum = sizeof(buttonPin) / sizeof(buttonPin[0]);
const int speedNum = sizeof(modeSpeed) / sizeof(modeSpeed[0]);
int cntFragment = 0;
int cntSpeed = 1;
int RealSpeed = 0;
bool pre[] = {HIGH, HIGH, HIGH, HIGH};
bool cur[] = {HIGH, HIGH, HIGH, HIGH};
bool stop = false;
char c = 0;

// Khai báo chân cảm biến line
#define sensorLeft 24
#define sensorMiddle 26
#define sensorRight 28

// Khai báo chân cảm biến siêu âm - HCSR04
#define trigPin 16
#define echoPin 17

// Khai báo chân điều khiển động cơ
// Motor 1
#define in1_L298N_no1 4
#define in2_L298N_no1 5
// Motor 2
#define in3_L298N_no1 6
#define in4_L298N_no1 7
// Motor 3
#define in1_L298N_no2 10
#define in2_L298N_no2 11
// Motor 4
#define in3_L298N_no2 8
#define in4_L298N_no2 9
// PWM
#define ENA 3
#define ENB 2
//Line
int L,M,R;  // 1: Có line ; 0: Không có line
String datablue;
// Ultrasonic value 
#define OBSTACLE_DISTANCE   20
#define OBSTACLE_DISTANCE_LOW 10
#define MAX_DISTANCE    1000 
#define SONIC_TIMEOUT   (MAX_DISTANCE*60) 
#define SOUND_VELOCITY    340
int turnType = 1; 
int speed = 60;

/*
 * Các chức năng điều hướng cho Robot.
 */
// Cấu hình cho motor dừng lại
void Stop() {
    digitalWrite(in1_L298N_no1, LOW);
    digitalWrite(in2_L298N_no1, LOW);
    digitalWrite(in3_L298N_no1, LOW);
    digitalWrite(in4_L298N_no1, LOW);
    digitalWrite(in1_L298N_no2, LOW);
    digitalWrite(in2_L298N_no2, LOW);
    digitalWrite(in3_L298N_no2, LOW);
    digitalWrite(in4_L298N_no2, LOW);
}

// Cấu hình cho motor quay trái
void MoveLeft() {
    digitalWrite(in1_L298N_no1, HIGH);
    digitalWrite(in2_L298N_no1, LOW);
    digitalWrite(in3_L298N_no1, LOW);
    digitalWrite(in4_L298N_no1, HIGH);
    digitalWrite(in1_L298N_no2, HIGH);
    digitalWrite(in2_L298N_no2, LOW);
    digitalWrite(in3_L298N_no2, LOW);
    digitalWrite(in4_L298N_no2, HIGH);	
}

// Cấu hình cho motor quay phải
void MoveRight() {
    digitalWrite(in1_L298N_no1, LOW);
    digitalWrite(in2_L298N_no1, HIGH);
    digitalWrite(in3_L298N_no1, HIGH);
    digitalWrite(in4_L298N_no1, LOW);
    digitalWrite(in1_L298N_no2, LOW);
    digitalWrite(in2_L298N_no2, HIGH);
    digitalWrite(in3_L298N_no2, HIGH);
    digitalWrite(in4_L298N_no2, LOW);	
}

// Cấu hình cho motor đi tiến
void MoveForward() {
    digitalWrite(in1_L298N_no1, HIGH);
    digitalWrite(in2_L298N_no1, LOW);
    digitalWrite(in3_L298N_no1, HIGH);
    digitalWrite(in4_L298N_no1, LOW);
    digitalWrite(in1_L298N_no2, HIGH);
    digitalWrite(in2_L298N_no2, LOW);
    digitalWrite(in3_L298N_no2, HIGH);
    digitalWrite(in4_L298N_no2, LOW);	
}

// Cấu hình cho motor đi lùi
void MoveBackward() {
    digitalWrite(in1_L298N_no1, LOW);
    digitalWrite(in2_L298N_no1, HIGH);
    digitalWrite(in3_L298N_no1, LOW);
    digitalWrite(in4_L298N_no1, HIGH);
    digitalWrite(in1_L298N_no2, LOW);
    digitalWrite(in2_L298N_no2, HIGH);
    digitalWrite(in3_L298N_no2, LOW);
    digitalWrite(in4_L298N_no2, HIGH);
}

/*
 * Các hàm bên dưới bao quát phần điều khiển động cơ.
 * enum MovementType: Các hướng đi.
 * MotorInit(): Khởi tạo, setup động cơ
 * MotorControl(): Điều khiển động cơ theo các hướng được định sẵn
 */
 
enum MovementType {
  MOVE_STOP,		// Sang trái
  MOVE_FORWARD,		// Đi thẳng
  MOVE_BACKWARD,	// Đi lùi
  MOVE_LEFT,		// Sang trái
  MOVE_RIGHT		// Sang phải
};

void MotorInit(bool is_analog_needed) {
	// Cấu hình tất cả các chân đều là đầu ra
	pinMode(in1_L298N_no1, OUTPUT); 
	pinMode(in2_L298N_no1, OUTPUT); 
	pinMode(in3_L298N_no1, OUTPUT); 
	pinMode(in4_L298N_no1, OUTPUT);
	pinMode(in1_L298N_no2, OUTPUT); 
	pinMode(in2_L298N_no2, OUTPUT); 
	pinMode(in3_L298N_no2, OUTPUT); 
	pinMode(in4_L298N_no2, OUTPUT);
  
	pinMode(ENB, OUTPUT);
  
	// Chỉ chạy code dưới khi is_analog_needed = 1
	if (is_analog_needed)
		analogWrite(ENB, 80);
}	

/* Hàm điều khiển động cơ
 * Tốc độ động cơ sẽ tự động bằng 60 nếu không có
 * giá trị nào được truyền vào.
 */ 
void MotorControl(MovementType direction, int speed = 60) {
	RealSpeed = speed;

	switch (direction) {
		case MOVE_STOP:
			Serial.println("Stop");
			Stop();
			break;
		case MOVE_FORWARD:
			Serial.println("Forward");
			MoveForward();
			break;
		case MOVE_BACKWARD:
			Serial.println("Backward");
			MoveBackward();
			break;
		case MOVE_LEFT:
			Serial.println("Left");
			MoveLeft();
			break;
		case MOVE_RIGHT:
			Serial.println("Right");
			MoveRight();
			break;
	}
	analogWrite(ENB, RealSpeed);
}

void moveMotors(
  bool m1_in1, bool m1_in2, bool m1_in3, bool m1_in4,
  bool m2_in1, bool m2_in2, bool m2_in3, bool m2_in4,
  int speedA, int speedB
) {
  digitalWrite(in1_L298N_no1, m1_in1);
  digitalWrite(in2_L298N_no1, m1_in2);
  digitalWrite(in3_L298N_no1, m1_in3);
  digitalWrite(in4_L298N_no1, m1_in4);
  digitalWrite(in1_L298N_no2, m2_in1);
  digitalWrite(in2_L298N_no2, m2_in2);
  digitalWrite(in3_L298N_no2, m2_in3);
  digitalWrite(in4_L298N_no2, m2_in4);
  analogWrite(ENA, speedA);
  analogWrite(ENB, speedB);
}

// Do khoang cach
float getSonar() {
  unsigned long pingTime;
  float distance;
  
  // Gửi xung 10us từ trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Đo thời gian phản hồi từ echoPin
  pingTime = pulseIn(echoPin, HIGH, SONIC_TIMEOUT);

  if (pingTime > 0) {
    // Tính khoảng cách (cm) = (thời gian * tốc độ âm thanh) / 2
    distance = (pingTime * SOUND_VELOCITY) / 2.0 / 10000.0; // vì 1s = 10^6 us, 1m = 100cm
  } else {
    // Nếu không đo được, trả về khoảng cách tối đa
    distance = MAX_DISTANCE;
  }

  return distance;
}

void ultrasonic_setup(){
  // Cảm biến siêu âm - HCSR04
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT);   
  Serial.begin(9600);

  // Không cần analog
  MotorInit(false);
}

void line_scan_setup() {
  // Cảm biến line
  pinMode(sensorLeft, INPUT);
  pinMode(sensorMiddle, INPUT);
  pinMode(sensorRight, INPUT);

  // Cần analog
  MotorInit(true);
}

void bluetooth_setup() {
  //Bluetooth.begin(9600);
  Serial3.begin(9600); 

  // Cần analog
  MotorInit(true);
}

void camera_setup(){
  Serial.begin(9600); // Match this with the baud rate you use on Raspberry Pi

  // Không cần analog
  MotorInit(false);

  while (!Serial) {
    ; // Wait for serial port to connect. Needed for native USB (Leonardo, etc.)
  }
  Serial.println("Arduino ready to receive");
}

bool check_stop() {
  if (stop) {
    stop = false;
    lcd.clear();
    lcd.print("Stopped");
    MotorControl(MOVE_STOP, RealSpeed); // Dừng motor
    delay(1000);
    return true;
  }
  return false;
}

void ultrasonic_loop(){
  float distance = getSonar();
  Serial.print("Distance: ");
  Serial.println(distance);

  // kiem tra
  if(distance < OBSTACLE_DISTANCE_LOW){
    RealSpeed = 60;
    MotorControl(MOVE_BACKWARD, RealSpeed);		// Đi lùi
    for (int i = 0; i< 10; i++){
      if (check_stop()) {
        stop = true;
        return ;
      }
      delay(45);     
    }
    //delay(450); !!!!!!!!!!!!!      
  }    
  else if (distance < OBSTACLE_DISTANCE) {
    RealSpeed = 80;
    MovementType direction = (turnType / 3) % 2 == 0 ? MOVE_LEFT : MOVE_RIGHT;
	MotorControl(direction, RealSpeed);
    for (int i = 0; i < 3; i++, turnType++) {
      Serial.print("turnType: "); 
      Serial.println(turnType);  
      for (int i = 0; i< 10; i++){
        if (check_stop()) {
          stop = true;
          return;
        }
        delay(50); 
      }
      //delay(500); !!!!!!!!!!
      distance = getSonar(); 
      Serial.print("Distance: ");
      Serial.println(distance);
      delayMicroseconds(2 * SONIC_TIMEOUT);
      if (distance >= OBSTACLE_DISTANCE) {
        break; 
      }
    }
  }
  else if(distance >= OBSTACLE_DISTANCE)
  {
    RealSpeed = 60;
    MotorControl(MOVE_FORWARD, RealSpeed); // Đi thẳng      
    delay(3); 
  }    
  // speed = .. 
}

// Đọc cảm biến có lọc nhiễu
void readLineSensors() {
  int Lcount = 0, Mcount = 0, Rcount = 0;
  for (int i = 0; i < 5; i++) {
    Lcount += digitalRead(sensorLeft);
    Mcount += digitalRead(sensorMiddle);
    Rcount += digitalRead(sensorRight);
    delay(5);
  }
  L = (Lcount >= 1) ? 1 : 0;
  M = (Mcount >= 1) ? 1 : 0;
  R = (Rcount >= 1) ? 1 : 0;
}

void line_scan_loop() {
  readLineSensors();
  Serial.print("L: "); Serial.print(L);
  Serial.print("M: "); Serial.print(M);
  Serial.print("R: "); Serial.println(R);

  if (M == 1 && L == 0 && R == 0) {
    MotorControl(MOVE_FORWARD, RealSpeed);
    delay(50); // Đi thẳng
  } 
  else if (L == 1 && M == 1 && R == 0) {
    MotorControl(MOVE_RIGHT, RealSpeed);
    delay(350); // Rẽ phải mạnh (vì xung PWM chạy trong 350ms)
  } 
  else if (R == 1 && M == 1 && L == 0) {
    MotorControl(MOVE_LEFT, RealSpeed);
    delay(350); // Rẽ trái mạnh (vì xung PWM chạy trong 350ms)
  }
  else if (L == 1 && M == 0 && R == 0) {
    MotorControl(MOVE_RIGHT, RealSpeed);
    delay(50); // Rẽ phải nhẹ (vì xung PWM chỉ chạy trong 50ms)
  }
  else if (R == 1 && M == 0 && L == 0) {
    MotorControl(MOVE_LEFT, RealSpeed);
    delay(50); // Rẽ trái nhẹ (vì xung PWM chỉ chạy trong 50ms)
  }
  else {
    MotorControl(MOVE_FORWARD, RealSpeed);  // Mặc định tiến nếu nhiều cảm biến cùng thấy line
  }
}

void bluetooth_loop() {
  bool commandReceived = false;
  
  //while(Serial3.available()){
  if(Serial3.available() > 0){
    c = Serial3.read();

    Serial.print(c);
    Serial.print(" ");

    // If we received a complete c
    switch(c){
      case 'S':
        Serial.println(" -> Dừng");
        moveMotors(
          0, 0, 0, 0,  // L298N_no1
          0, 0, 0, 0,  // L298N_no2
          0, 0       // Speeds
        );
      break; 
      case 'F':
        Serial.println(" -> Đi Thẳng");
        moveMotors(
          1, 0, 1, 0,  // L298N_no1
          1, 0, 1, 0,  // L298N_no2
          80, 80       // Speeds
        );
      break;
      case 'R':
        Serial.println(" -> Đi Sang Phải");
        moveMotors(
          1, 0, 0, 1,  // L298N_no1
          0, 1, 1, 0,  // L298N_no2
          100, 100       // Speeds
        );
      break;
      case 'L':
        Serial.println(" -> Đi Sang Trái");
        moveMotors(
          0, 1, 1, 0,  // L298N_no1
          1, 0, 0, 1,  // L298N_no2
          100, 100       // Speeds
        );
      break;
      case 'B':
        Serial.println(" -> Đi Lùi");
        moveMotors(
          0, 1, 0, 1,  // L298N_no1
          0, 1, 0, 1,  // L298N_no2
          80, 80       // Speeds
        );
      break;
      case 'A':
        Serial.println(" -> Đi Thẳng Chéo Trái");
        moveMotors(
          0, 0, 1, 0,  // L298N_no1
          1, 0, 0, 0,  // L298N_no2
          180, 180       // Speeds
        );
      break;
      case 'I':
        Serial.println(" -> Đi Thẳng Chéo Phải");
        moveMotors(
          1, 0, 0, 0,  // L298N_no1
          0, 0, 1, 0,  // L298N_no2
          180, 180       // Speeds
        );
      break;
      case 'C':
        Serial.println(" -> Đi Lùi Chéo Trái");
        moveMotors(
          0, 1, 0, 0,  // L298N_no1
          0, 0, 0, 1,  // L298N_no2
          180, 180       // Speeds
        );
      break;
      case 'D':
        Serial.println(" -> Đi Lùi Chéo Phải");
        moveMotors(
          0, 0, 0, 1,  // L298N_no1
          0, 1, 0, 0,  // L298N_no2
          180, 180       // Speeds
        );
      break;
      case 'G':
        Serial.println(" -> Quay Vòng Trái");
        moveMotors(
          0, 1, 1, 0,  // L298N_no1
          0, 1, 1, 0,  // L298N_no2
          80, 80       // Speeds
        );
      break;
      case 'H':
        Serial.println(" -> Quay Vòng Phải");
        moveMotors(
          1, 0, 0, 1,  // L298N_no1
          1, 0, 0, 1,  // L298N_no2
          80, 80       // Speeds
        );
      break; 
      case 'M':
        Serial.println(" -> Dừng Khẩn Cấp");
        //stop();
        moveMotors(
          0, 0, 0, 0,  // L298N_no1
          0, 0, 0, 0,  // L298N_no2
          0, 0       // Speeds
        );
      break;
    }
  }
}

void camera_loop(){
  if(Serial.available() > 0){
    c = Serial.read();
    
    Serial.print(c);
    Serial.print(" ");

    // If we received a complete c
    switch(c){
      case 'S':
        Serial.println(" -> Dừng");
        moveMotors(
          0, 0, 0, 0,  // L298N_no1
          0, 0, 0, 0,  // L298N_no2
          0, 0       // Speeds
        );
      break; 
      case 'F':
        Serial.println(" -> Đi Thẳng");
        moveMotors(
          1, 0, 1, 0,  // L298N_no1
          1, 0, 1, 0,  // L298N_no2
          80, 80       // Speeds
        );
      break;
      case 'R':
        Serial.println(" -> Đi Sang Phải");
        moveMotors(
          1, 0, 0, 1,  // L298N_no1
          0, 1, 1, 0,  // L298N_no2
          100, 100       // Speeds
        );
      break;
      case 'L':
        Serial.println(" -> Đi Sang Trái");
        moveMotors(
          0, 1, 1, 0,  // L298N_no1
          1, 0, 0, 1,  // L298N_no2
          100, 100       // Speeds
        );
      break;
      case 'B':
        Serial.println(" -> Đi Lùi");
        moveMotors(
          0, 1, 0, 1,  // L298N_no1
          0, 1, 0, 1,  // L298N_no2
          80, 80       // Speeds
        );
      break;
    }    
  }
}

void stopISR(){
  stop = true;
}

void printRun() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.println("Robot is running");
  lcd.setCursor(0, 1);
  lcd.print("Speed = ");
  lcd.print(RealSpeed);
}

void ultrasonic_mode() {
  bool printed = false;
  ultrasonic_setup();
  printRun();
  while (1) {
    if (check_stop()) {
      break;
    }
    ultrasonic_loop();
  }
  
  RealSpeed = 70; // Đặt lại speed real
  MotorControl(MOVE_STOP, RealSpeed); // Dừng motor
}

void line_scan_mode() {
  line_scan_setup();
  printRun();
  while (true) {
    if (check_stop()) {
      break;
    }
    line_scan_loop();
  }
  
  MotorControl(MOVE_STOP, RealSpeed); // Dừng motor
}

void camera_mode() {
  bool printed = false;
  camera_setup();
  printRun();
  while (1) {
    if (check_stop()) {
      break;
    }
    camera_loop();
  }
  
  MotorControl(MOVE_STOP, RealSpeed); // Dừng motor
}

void bluetooth_mode() {
  bluetooth_setup();
  printRun();
  while (1) {
    if (check_stop()) {
      break;
    }
    bluetooth_loop();
  }
  
  MotorControl(MOVE_STOP, RealSpeed); // Dừng motor
}

void showFragment(int cnt) {
  lcd.clear();
  switch (cnt) {
    case 0:
      lcd.print("< Bluetooth >");
      break;
    case 1:
      lcd.print("< Camera >");
      break;
    case 2:
      lcd.print("< Line scan >");
      break;
    case 3:
      lcd.print("< Untrasonic >");
      break;
  }
}

void showModeSpeed(int cnt) {
  lcd.clear();
  switch (cnt) {
    case 0:
      lcd.print("< Low >");
      break;
    case 1:
      lcd.print("< Medium >");
      break;
    case 2:
      lcd.print("< High >");
      break;
  }
}

bool speedRun() {
  cntSpeed = 1;
  showModeSpeed(cntSpeed);
  while (1) {
    for (int i = 0; i < buttonNum; i += 1) {
      if (check_stop()) {
        return false;
      }
      cur[i] = digitalRead(buttonPin[i]);
      if (pre[i] == HIGH && cur[i] == LOW) {
        switch (i) {
          case 0: // LEFT
            cntSpeed -= 1;
            if (cntSpeed < 0) {
              cntSpeed = speedNum - 1;
            }
            showModeSpeed(cntSpeed);
            break;
          case 1: // RUN
            RealSpeed = modeSpeed[cntSpeed];
            for (int i = 0; i < buttonNum; i += 1) {
              pre[i] = HIGH;
            }
            return true;
          case 2: // RIGHT
            cntSpeed += 1;
            if (cntSpeed >= speedNum) {
              cntSpeed = 0;
            }
            showModeSpeed(cntSpeed);
            break;
        }
      }
      pre[i] = cur[i];
    }
  }
  return false;
}

void runFragment(int cnt) {
  lcd.clear();
  lcd.print("Done !!!");
  delay(1000);
  lcd.clear();
  bool check = speedRun();
  if (check == false) {
    return;
  }
  switch (cnt) {
    case 0:
      bluetooth_mode();
      break;
    case 1:
      camera_mode();
      break;
    case 2:
      line_scan_mode();
      break;
    case 3:
      ultrasonic_mode();
      break;
  }
}

void loading() {
  lcd.print("Loading all mode");
  for (int i = 0; i < 3; i += 1) {
    delay(350);
    lcd.print(".");
  }
}

void setup() {
  Serial.begin(9600);

  for (int i = 0; i < buttonNum; i += 1) {
    pinMode(buttonPin[i], INPUT_PULLUP);
  }
  attachInterrupt(digitalPinToInterrupt(buttonPin[3]), stopISR, FALLING);
#if HOST_HAS_LEGACY_LIQUIDCRYSTAL_LIBRARY
  lcd.begin();
#else
  lcd.init();
#endif
  lcd.backlight();
  loading();
  showFragment(cntFragment);
}

void loop() {
  for (int i = 0; i < buttonNum; i += 1) {
    cur[i] = digitalRead(buttonPin[i]);
    if (pre[i] == HIGH && cur[i] == LOW) {
      stop = false;
      switch (i) {
        case 0: // LEFT
          cntFragment -= 1;
          if (cntFragment < 0) {
            cntFragment = buttonNum - 1;
          }
          showFragment(cntFragment);
          break;
        case 1: // RUN
          stop = false;
          runFragment(cntFragment);
          showFragment(cntFragment);
          for (int i = 0; i < buttonNum; i += 1) {
            pre[i] = HIGH;
          }
          break;
        case 2: // RIGHT
          cntFragment += 1;
          if (cntFragment >= buttonNum) {
            cntFragment = 0;
          }
          showFragment(cntFragment);
          break;
      }
    }
    pre[i] = cur[i];
  }
}
