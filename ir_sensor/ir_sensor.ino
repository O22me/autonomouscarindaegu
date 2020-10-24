const int IR_R = 3;  //  적외선센서 우측 핀
const int IR_L = 4;  // 적외선센서 좌측 핀

void setup() {
    Serial.begin(9600);
    pinMode(IR_R, INPUT);
    pinMode(IR_L, INPUT);
}

void loop() {

    Serial.print("Right : "); Serial.print(digitalRead(IR_R));
     Serial.print("    Left : "); Serial.println(digitalRead(IR_L));
    delay(500);
    
}
