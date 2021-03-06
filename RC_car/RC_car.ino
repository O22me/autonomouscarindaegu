#include <Servo.h>
Servo servo;

const int SERVO_PIN = 9;      // 서보모터1 연결핀
const int IR_R = 3;  //  적외선센서 우측 핀
const int IR_L = 4;  // 적외선센서 좌측 핀

const int M1_PWM = 5;   // DC모터1 PWM 핀 왼
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

int max_ai_pwm = 130; // 자율주행 모터 최대 출력 (0 ~ 255)
int min_ai_pwm = 70; // 자율주행 모터 최소 출력 (0 ~ 255)

int angle_offset = 0; // 서보 모터 중앙각 오프셋 (단위: 도) 마이너스 값에서 바퀴가 왼쪽으로 정렬되고 플러스 값에서 오른쪽으로 정렬됨
int angle_limit = 55; // 서보 모터 회전 제한 각 (단위: 도)


float cur_steering;
float cur_speed;
float compute_steering;
float compute_speed;

float max_pwm;
float min_pwm;

// 초음파 거리측정
float GetDistance(int trig, int echo)
{
  digitalWrite(trig, LOW);
  delayMicroseconds(4);
  digitalWrite(trig, HIGH);
  delayMicroseconds(20);
  digitalWrite(trig, LOW);

  unsigned long duration = pulseIn(echo, HIGH, 5000);
  if (duration == 0)
    return MAX_DISTANCE;
  else
    return duration * 0.17;     // 음속 340m/s
}

// 앞바퀴 조향
void SetSteering(float steering)
{
  cur_steering = constrain(steering, -1, 1);// constrain -1~ 1 값으로 제한

  float angle = cur_steering * angle_limit;
  int servoAngle = angle + 90;
  servoAngle += angle_offset;

  servoAngle = constrain(servoAngle, 0, 180);
  servo.write(servoAngle);
}


// 뒷바퀴 모터회전
void SetSpeed(float speed)
{
  speed = constrain(speed, -1, 1);

  if ((cur_speed * speed < 0) // 움직이는 중 반대 방향 명령이거나
      || (cur_speed != 0 && speed == 0)) // 움직이다가 정지라면
  {
    cur_speed = 0;
    digitalWrite(M1_PWM, HIGH);
    digitalWrite(M1_DIR1, LOW);
    digitalWrite(M1_DIR2, LOW);

    digitalWrite(M2_PWM, HIGH);
    digitalWrite(M2_DIR1, LOW);
    digitalWrite(M2_DIR2, LOW);

    if (stop_time > 0)
      delay(stop_time);
  }

  if (cur_speed == 0 && speed != 0) // 정지상태에서 출발이라면
  {
    if (punch_time > 0)
    {
      if (speed > 0)
      {
        analogWrite(M1_PWM, punch_pwm);
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

    if (speed  > 0)
    {
      analogWrite(M1_PWM, pwm);
      digitalWrite(M1_DIR1, HIGH);
      digitalWrite(M1_DIR2, LOW);

      analogWrite(M2_PWM, pwm);
      digitalWrite(M2_DIR1, HIGH);
      digitalWrite(M2_DIR2, LOW);
    }
    else if (speed < 0)
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

  //Ultrasonic
  pinMode(FC_TRIG, OUTPUT);
  pinMode(FC_ECHO, INPUT);
  
  pinMode(L_TRIG, OUTPUT);
  pinMode(L_ECHO, INPUT);
  
  pinMode(R_TRIG, OUTPUT);
  pinMode(R_ECHO, INPUT);

  max_pwm = max_ai_pwm;
  min_pwm = min_ai_pwm;

  SetSteering(0);
  SetSpeed(0);
}

void loop() {
  compute_steering = cur_steering;
  compute_speed = cur_speed;

  center = GetDistance(FC_TRIG, FC_ECHO);
  left = GetDistance(L_TRIG, L_ECHO);
  right = GetDistance(R_TRIG, R_ECHO);

  if (center <= 150) {
    compute_steering = 0;
    compute_speed = 0;
  }

  else if (digitalRead(IR_R) == detect_ir && digitalRead(IR_L) == detect_ir) { 
    compute_steering = 0;
    compute_speed = 1;
  }
  else if (digitalRead(IR_R) != detect_ir) { // 오른쪽 차선이 검출된 경우
    compute_steering = -1;
    compute_speed = 0.1;
  }

  else if (digitalRead(IR_L) != detect_ir) { //왼쪽 차선이 검출된 경우
    compute_steering = 1;
    compute_speed = 0.1;
  }
  
  SetSpeed(compute_speed);
  SetSteering(compute_steering);

  //Ultrasonic to confirm
  Serial.print("C:");
  Serial.print(center);

  Serial.print("     L:");
  Serial.print(left);
  
  Serial.print("     R:");
  Serial.println(right);
  
}
