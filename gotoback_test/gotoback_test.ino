#include <Servo.h>
Servo servo;

const int SERVO_PIN = 9;      // 서보모터1 연결핀
const int IR_R = 3;  //  적외선센서 우측 핀
const int IR_L = 4;  // 적외선센서 좌측 핀

const int M1_PWM = 5;   // DC모터1 PWM 핀 왼l
const int M1_DIR1 = 7;   // DC모터1 DIR1 핀
const int M1_DIR2 = 8;   // DC모터 1 DIR2 핀

const int M2_PWM = 6;   // DC모터2 PWM 핀
const int M2_DIR1 = 11;   // DC모터2 DIR1 핀
const int M2_DIR2 = 12;   // DC모터2 DIR2 핀

const int FC_TRIG  = 13;   // 전방 초음파 센서 TRIG 핀
const int FC_ECHO = 10;  // 전방 초음파 센서 ECHO 핀
const int L_TRIG = A2;  // 좌측 초음파 센서 TRIG 핀
const int L_ECHO = A1;  // 좌측 초음파 센서 ECHO 핀
const int R_TRIG = A5;   // 우측 초음파 센서 TRIG 핀
const int R_ECHO = 2;  // 우측 초음파 센서 ECHO 핀

const int MAX_DISTANCE = 2000; // 초음파 센서의 최대 감지거리


float center;
float left;
float right;

int state = 0;
// 자동차 튜닝 파라미터 =====================================================================
boolean detect_ir = true; // 검출선이 흰색 = true, 검정색 = false

int punch_pwm = 200; // 정지 마찰력 극복 출력 (0 ~ 255)
int punch_time = 50; // 정지 마찰력 극복 시간 (단위 msec)
int stop_time = 300; // 전진후진 전환 시간 (단위 msec)

int max_ai_pwm = 110; // 자율주행 모터 최대 출력 (0 ~ 255)
int min_ai_pwm = 70; // 자율주행 모터 최소 출력 (0 ~ 255)

int angle_offset = 5; // 서보 모터 중앙각 오프셋 (단위: 도) 마이너스 값에서 바퀴가 왼쪽으로 정렬되고 플러스 값에서 오른쪽으로 정렬됨
int angle_limit = 60; // 서보 모터 회전 제한 각 (단위: 도) //55->60으로 수정하였음


float cur_steering;     //현재 스티어링 방향
float cur_speed;        //현재 속도
float compute_steering; //계산한 스티어링 방향(적용될 방향)
float compute_speed;    //계산한 속도(적용될 속도)

float max_pwm;
float min_pwm;

boolean rightspace_counter = false;
boolean rightspace_open = false;
boolean rightspace_complete = false;

// 초음파 거리측정
float GetDistance(int trig, int echo)
{
  digitalWrite(trig, LOW);
  delayMicroseconds(4);
  digitalWrite(trig, HIGH);
  delayMicroseconds(20);
  digitalWrite(trig, LOW);

  unsigned long duration = pulseIn(echo, HIGH, 5000); //echo가 high상태후 low까지의 시간을 반환(msec)
  if (duration == 0) //펄스가 시작되지 않을 시 0을 반환 ->
    return MAX_DISTANCE;      //초음파센서 최대 감지가능 거리
  else
    return duration * 0.17;   // 음속 340m/s
}

// 앞바퀴 조향
void SetSteering(float steering) // -1, 0, 1 파라미터(1은 오른쪽, -1은 왼쪽, 0)
{
  cur_steering = constrain(steering, -1, 1);  // constrain -1~ 1 값으로 제한

  float angle = cur_steering * angle_limit; //(-1 ~ 1) * 55(초기값)
  int servoAngle = angle + 90;
  servoAngle += angle_offset;

  servoAngle = constrain(servoAngle, 0, 180);
  servo.write(servoAngle);
}


// 뒷바퀴 모터회전
void SetSpeed(float speed)                //0.1의 값, 1의 값
{
  speed = constrain(speed, -1, 1);
  
  // 움직이는 중 반대 방향 명령이거나 움직이다가 정지라면
  if ((cur_speed * speed < 0) || (cur_speed != 0 && speed == 0))  
  {
    cur_speed = 0; //움직임 정지명령
    digitalWrite(M1_PWM, HIGH); //왼쪽 뒷바퀴 off
    digitalWrite(M1_DIR1, LOW);
    digitalWrite(M1_DIR2, LOW);

    digitalWrite(M2_PWM, HIGH); //오른 뒷바퀴 off
    digitalWrite(M2_DIR1, LOW);
    digitalWrite(M2_DIR2, LOW);

    if (stop_time > 0)
      delay(stop_time);
  }

  if (cur_speed == 0 && speed != 0) // 정지상태에서 출발이라면
  {
    if (punch_time > 0) //정지마찰력 극복시간 : 50
    {
      if (speed > 0)
      {
        analogWrite(M1_PWM, punch_pwm); //정지마찰력 극복출력 : 200
        digitalWrite(M1_DIR1, HIGH);
        digitalWrite(M1_DIR2, LOW);

        analogWrite(M2_PWM, punch_pwm);
        digitalWrite(M2_DIR1, HIGH);
        digitalWrite(M2_DIR2, LOW);
      }
      else if (speed < 0)
      {
        analogWrite(M1_PWM, punch_pwm);
        digitalWrite(M1_DIR1, LOW);
        digitalWrite(M1_DIR2, HIGH);

        analogWrite(M2_PWM, punch_pwm);
        digitalWrite(M2_DIR1, LOW);
        digitalWrite(M2_DIR2, HIGH);
      }
      delay(punch_time);
    }
  }

  if (speed != 0) // 명령이 정지가 아니라면
  {
    int pwm = abs(speed) * (max_pwm - min_pwm) + min_pwm;           // 0 ~ 255로 변환

    if (speed  > 0) //전진?
    {
      analogWrite(M1_PWM, pwm);
      digitalWrite(M1_DIR1, HIGH);
      digitalWrite(M1_DIR2, LOW);

      analogWrite(M2_PWM, pwm);
      digitalWrite(M2_DIR1, HIGH);
      digitalWrite(M2_DIR2, LOW);
    }
    else if (speed < 0) //후진?
    {
      analogWrite(M1_PWM, pwm);
      digitalWrite(M1_DIR1, LOW);
      digitalWrite(M1_DIR2, HIGH);

      analogWrite(M2_PWM, pwm);
      digitalWrite(M2_DIR1, LOW);
      digitalWrite(M2_DIR2, HIGH);
    }
  }
  cur_speed = speed;
}
void parking_to_backward() {
  SetSpeed(0); delay(1500);        //완전히 정차 후 주차시작
  SetSteering(1);                  //오른쪽으로 바퀴정렬
  SetSpeed(-1);
  delay(900);     //오른쪽 후진으로 0.8초
  SetSteering(-1);
  delay(900);     //왼쪽 후진으로 0.4초 //아마 초음파센서 추가활용가능성
  SetSteering(0);
  SetSpeed(0);
  delay(2000);//주차완료

  //좌측으로 나가기
  SetSteering(-1);
  SetSpeed(1);
  delay(400);
  rightspace_complete = true;
  return;
}

void setup() {

  Serial.begin(115200);
  servo.attach(SERVO_PIN); //서보모터 초기화

  pinMode(IR_R, INPUT);
  pinMode(IR_L, INPUT);

  pinMode(M1_PWM, OUTPUT);
  pinMode(M1_DIR1, OUTPUT);
  pinMode(M1_DIR2, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  pinMode(M2_DIR1, OUTPUT);
  pinMode(M2_DIR2, OUTPUT);

  pinMode(FC_TRIG, OUTPUT);
  pinMode(FC_ECHO, INPUT);

  pinMode(L_TRIG, OUTPUT);
  pinMode(L_ECHO, INPUT);

  pinMode(R_TRIG, OUTPUT);
  pinMode(R_ECHO, INPUT);

  max_pwm = max_ai_pwm;  //파워최대값 : 110
  min_pwm = min_ai_pwm;  //파워최소값 : 70

  SetSteering(0);
  SetSpeed(0);
}

void loop() {
  compute_steering = cur_steering;
  compute_speed = cur_speed;
  
  center = GetDistance(FC_TRIG, FC_ECHO);
  left = GetDistance(L_TRIG, L_ECHO);
  right = GetDistance(R_TRIG, R_ECHO);
  //카운터 감지
  if (right >= 220) {
    rightspace_open = true;
    rightspace_counter = true;
  }
  else if (right <= 120) {
    rightspace_open = false;
  }
  
  if (!(rightspace_complete == true) && rightspace_open == false
      && rightspace_counter == true) { //공간감지완료 및 이미 사용여부
    parking_to_backward();
  }
  //직진차선
  else if (digitalRead(IR_R) == detect_ir && digitalRead(IR_L) == detect_ir) {
    compute_steering = 0; //바퀴정렬
    compute_speed = 1;    //속도 1
  }
  // 양쪽 차선이 모두 검출된 경우(정지선)
  else if (digitalRead(IR_R) != detect_ir && digitalRead(IR_L) != detect_ir) {
    compute_steering = 0;
    compute_speed = 0;
    //추가필요 : 약간의 전진으로 다시작동
  }
  // 오른쪽 차선이 검출된 경우
  else if (digitalRead(IR_R) != detect_ir) { 
    compute_steering = -1;//왼쪽으로 바퀴정렬
    compute_speed = 0.1;  //속도 0.1
  }
  
  //왼쪽 차선이 검출된 경우
  else if (digitalRead(IR_L) != detect_ir) { 
    compute_steering = 1; //오른쪽으로 바퀴정렬
    compute_speed = 0.1;  //속도 0.1
  }
  SetSpeed(compute_speed);
  SetSteering(compute_steering);
}
