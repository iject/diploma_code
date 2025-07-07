//  Нумерация приводов робота
//  -----               -----
// |  7  |             |  1  |
//  ----- -----   ----- -----
//       |  8  | |  2  |
//        -----   -----
//       |  6  | |  4  |
//  ----- -----   ----- -----
// |  5  |             |  3  |
//  -----               -----  (Вид сверху)
//

/*
 Пр.пер. колено, Пр. пер. плечо, 
 Пр.зад. колено, Пр. зад. плечо,
 Лев.зад. колено, Лев. зад. плечо,
 Лев.пер. колено, лев.пер. плечо
*/

// ПОДКЛЮЧЕНИЕ БИБЛИОТЕК
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// дефайны для отладки
// #define MY_DBG_SERIAL
// #define MY_DBG_ALL_SERIAL
// #define MY_DBG_SERVO
// #define MY_DBG_SERVO_ALL
// #define MY_DBG_MSG
// #define MY_DBG_MPU

#define MY_ERROR_MSG(x) Serial.println("Error: " + String(x))
#define MY_DEBUG_PRINT(x) Serial.println(x)
#define MY_DEBUG_PRINT(x)

#define MY_TEMP 25
#define MY_HC_ECHO 2
#define MY_HC_TRIG 3
#define MY_BAUDRATE 115200

// Границы сигналов для каждого привода
const short border_arr[][2] = { { 114, 490 }, 
                                { 86,  474 },
                                { 74,  460 },
                                { 122, 504 },
                                { 68,  472 },
                                { 70,  452 },
                                { 78,  452 },
                                { 78,  468 } };
const int MPU_addr=0x68;
int16_t MPU_data[6];
float MPU_data_filt[6] = {0., 0., 0., 0., 0., 0.};
float dist_filt = 0;
uint32_t HC_tmr = -50;
const char* digits = "1234567890";

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

struct DataStruct{
  char data[25];
  int idx;
  bool read;
};

DataStruct python;

void setup() {
  startMPU();
  
  Serial.begin(MY_BAUDRATE);

  pinMode(MY_HC_ECHO, INPUT);
  pinMode(MY_HC_TRIG, OUTPUT);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  
  Serial.println("setup finish");
}

void loop() {
  getMPU();
  if (millis() - HC_tmr >= 50) {
    HC_tmr = millis();
    getDist();
  }

  readData();
  MY_DEBUG_PRINT("end loop");
}

void readData() {
  while(Serial.available()) {
    MY_DEBUG_PRINT("start readData");
    
    char c = Serial.read();

    #ifdef MY_DBG_ALL_SERIAL
      Serial.println("a.debug > c = " + String(c));
    #endif

    if (python.read) {
      if (c == '>') {
        python.read = false;
        python.data[python.idx] = '\0';

        #ifdef MY_DBG_SERIAL
          Serial.println("a.debug > finish read. | Serial.available() = " + String(Serial.available()));
        #endif

        receivedData(python);
      } 
      else {
        if (python.idx == 25) {
          MY_ERROR_MSG("Выход за границу массива. Нарушен протокол.");
          while(1);
        }
        python.data[python.idx] = c;
        python.idx++;
      }
    } 
    else if (c == '<') {
      python.read = true;
      python.idx = 0;

      #ifdef MY_DBG_SERIAL
        Serial.println("a.debug > start read. | Serial.available() + 1 = " + String(Serial.available() + 1));
      #endif
    }
  }
}

void receivedData(DataStruct rec_d) {
  #ifdef MY_DBG_MSG
    Serial.println("a.debug > data from python: " + String(rec_d.data));
  #endif

  if (rec_d.data[0] == 'd') {
    printMPU();
    printDist();
    return;
  }
  else if (rec_d.data[0] == 's') {
    int angles[8];

    char angle[4];
    angle[3] = '\0';

    int i, j;
    for (byte i = 0; i < 8; i++) {
      for (byte j = 0; j < 3; j++) {
        angle[j] = rec_d.data[1 + i * 3 + j];
      }
      if (strspn(angle, digits) != 3) {
        MY_ERROR_MSG("Встречен нечисловой символ.");
        while(1);
      }
      else {
        angles[i] = atoi(angle);
      }
    }
    
    set(angles);
  } else {
    Serial.println("a.msg: unknown command");
  }
}

void startMPU() {
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void getMPU() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);
  for (byte i = 0; i < 6; i++) {
    if (i != 3){
      MPU_data[i] = Wire.read() << 8 | Wire.read();
    }
    else{
      Wire.read(); Wire.read();
      MPU_data[i] = Wire.read() << 8 | Wire.read();
    }
  }
  
  for (byte i = 0; i < 6; i++) {
    MPU_data_filt[i] += (MPU_data[i] - MPU_data_filt[i]) * 0.2;
  }
  
  Wire.endTransmission(true);
}

void getDist() {
  digitalWrite(MY_HC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(MY_HC_TRIG, LOW);

  uint32_t pulse_len = pulseIn(MY_HC_ECHO, HIGH);

  float dist = pulse_len * (MY_TEMP * 6 / 10 + 330) / 2000;
  dist_filt += (dist - dist_filt) * 0.2;
}

void printDist() {  
  Serial.println(dist_filt);
}

void printMPU() {
  for (byte i = 0; i < 5; i++) {
    Serial.print(MPU_data_filt[i]);
    Serial.print("\t");
  }
  Serial.println(MPU_data_filt[5]);
}

void set(int *angles) {
  #if defined(MY_DBG_SERVO) || defined(MY_DBG_SERVO_ALL)
    Serial.print("a.debug > set(angles): ");
    Serial.print("[");
  #endif

  for (byte i = 0; i < 8; i++) {
    if (angles[i] < 0 || angles[i] > 180) {
      MY_ERROR_MSG("Угол с индексом i=" + String(i) + " задан вне предусмотренного диапазона.");
      while(1);
    }

    #if defined(MY_DBG_SERVO)
      Serial.print(String(angles[i]) + ", ");
    #elif defined(MY_DBG_SERVO_ALL)
      Serial.print(String(angles[i]) + ", ");
      move(i, angles[i]);
    #else
      move(i, angles[i]);
    #endif
  }

  #if defined(MY_DBG_SERVO) || defined(MY_DBG_SERVO_ALL)
    Serial.println("]");
  #endif
}

int angleToPulse(byte num, int ang) {
  int pulse = map(ang, 0, 180, border_arr[num][0], border_arr[num][1]);
  return pulse;
}

void move(byte chanel, byte angle) {
  switch (chanel) {
    case 1:
    case 2:
    case 3:
    case 6:
      pwm.setPWM(chanel, 0, angleToPulse(chanel, 180 - angle));
      break;
    case 0:
    case 4:
    case 5:
    case 7:
      pwm.setPWM(chanel, 0, angleToPulse(chanel, angle));
      break;
  }
}