#include <IRremote.hpp>
#include <Wire.h>
#include <VL53L0X.h>


int error;
int last_error;

int KP_LEFT;
int KP_RIGHT;
#define ESQ_IN1_PIN PD7  //7
#define ESQ_IN2_PIN PB0  //8
#define ESQ_PWM_PIN PB1  //9
#define ESQ_STDBY PD6    //6

#define DIR_PWM_PIN PB2  //10
#define DIR_IN1_PIN PC1  //A1
#define DIR_IN2_PIN PC0  //A0
#define DIR_STDBY PC2    //A2

#define IR_RECEIVE_PIN 11
#define READY_CODE 0x80
#define START_CODE 0x81
#define STOP_CODE 0x82
#define ESTRATEGIA_A 0xF30CFF00
#define ESTRATEGIA_B 0xE718FF00
#define ESTRATEGIA_C 0xA15EFF00
#define ESTRATEGIA_D 0xF708FF00
#define PID_F 0xA55AFF00
#define PID_E 0xE31CFF00

uint8_t ESTRATEGIA = 0;
uint8_t PID = 0;
unsigned long TEMPO = 0;

VL53L0X sensor_left;
VL53L0X sensor_right;

#define dist_left 2
#define dist_right A3
#define LIMIAR 500
#define LIMIAR2 300

uint16_t distancia_left;
uint16_t distancia_right;
uint16_t last_dist_left = 600;
uint16_t last_dist_right = 600;


void setup() {
  Serial.print("A");
  IrReceiver.begin(IR_RECEIVE_PIN);

  MOTORS_CONFIG();
  Serial.begin(9600);
  VL53L0X_CONFIG();
  /*CALC_ERROR();
  CALC_ERROR();*/
  Serial.print("1");
  SELECIONA_PID();
  SELECIONA_ESTRATEGIA();
  CHECK_START();
  switch (PID) {
    case 11:
      EXECUTA_ESTRATEGIA_B();
      break;
    case 22:
      EXECUTA_ESTRATEGIA_SB();
      break;
  }
}

void loop() {
  CHECK_STOP();

  switch (PID) {
    case 11:
      CALC_ERROR1();
      break;
    case 22:
      CALC_ERROR2();
      break;
  }

  KP_LEFT = int(0.25 * distancia_left);
  KP_RIGHT = int(0.25 * distancia_right);


  switch (error) {
    case -2:
      SET_MOTORS((-110 + int(0.1 * last_dist_left)), (110 - int(0.1 * last_dist_left)));
      break;
    case -1:
      SET_MOTORS((250 - KP_LEFT - KP_LEFT), (250 - KP_LEFT));
      break;
    case 0:
      SET_MOTORS((260 - KP_LEFT), (260 - KP_RIGHT));
      break;
    case 1:
      SET_MOTORS((250 - KP_RIGHT), (250 - KP_RIGHT - KP_RIGHT));
      break;
    case 2:
      SET_MOTORS((110 - int(0.1 * last_dist_left)), (-110 + int(0.1 * last_dist_left)));
      break;
  }
}

void CALC_ERROR1() {

  //distancia_left = sensor_left.readRangeContinuousMillimeters();
  //distancia_right = sensor_right.readRangeContinuousMillimeters();
  FILTRO();
  if (distancia_left < LIMIAR) {
    if (distancia_right < LIMIAR) {  // o 2 sensores detectam o inimigo
      error = 0;
    } else {  // apenas o sensor da esquerda detecta o inimigo
      error = -1;
      last_error = -1;
      last_dist_left = distancia_left;
    }
  } else if (distancia_right < LIMIAR) {  // apenas o sensor da direita detecta o inimigo
    error = 1;
    last_error = 1;
    last_dist_right = distancia_right;
  } else if (last_error == 1) {  // nenhum sensor detecta o inimigo, mas o último a detectar foi o da direita
    error = 2;
  } else {
    error = -2;
  }
}
void CALC_ERROR2() {
  //distancia_left = sensor_left.readRangeContinuousMillimeters();
  //distancia_right = sensor_right.readRangeContinuousMillimeters();
  FILTRO();
  if (distancia_right < LIMIAR2 && distancia_left < LIMIAR2) {
    error = 0;
  } else if (distancia_left < LIMIAR) {
    if (distancia_right < LIMIAR) {  // o 2 sensores detectam o inimigo
      error = 0;
      SET_MOTORS(-200, 190);
      while (distancia_left < LIMIAR) {
        FILTRO();
        //distancia_left = sensor_left.readRangeContinuousMillimeters();
      }
      TEMPO = millis();
      SET_MOTORS(120, -110);
      FILTRO();
      //distancia_right = sensor_right.readRangeContinuousMillimeters();
      while (!(distancia_right > LIMIAR && distancia_left < LIMIAR)) {
        //distancia_right = sensor_right.readRangeContinuousMillimeters();
        //distancia_left = sensor_left.readRangeContinuousMillimeters();
        FILTRO();
        if (millis() - TEMPO > 12000) break;
      }
      unsigned long TEMPO_QUE_PASSOU = millis() - TEMPO;
      TEMPO = millis();
      unsigned long TEMPO_PARA_ESPERAR = TEMPO_QUE_PASSOU / 2;

      SET_MOTORS(-120, 110);
      while (millis() < TEMPO + TEMPO_PARA_ESPERAR)
        ;
      SET_MOTORS(0, 0);
      FILTRO();
      //distancia_right = sensor_right.readRangeContinuousMillimeters();
      //distancia_left = sensor_left.readRangeContinuousMillimeters();

    } else {  // apenas o sensor da esquerda detecta o inimigo
      error = -1;
      last_error = -1;
      last_dist_left = distancia_left;
    }
  } else if (distancia_right < LIMIAR) {  // apenas o sensor da direita detecta o inimigo
    error = 1;
    last_error = 1;
    last_dist_right = distancia_right;
  } else if (last_error == 1) {  // nenhum sensor detecta o inimigo, mas o último a detectar foi o da direita
    error = 2;

  } else {
    error = -2;
  }
}
void SELECIONA_PID(void) {
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  while (PID == 0) {
    if (IrReceiver.decode()) {
      IrReceiver.resume();
      switch (IrReceiver.decodedIRData.decodedRawData) {
        case PID_E:
          Serial.println("ESTRATEGIA 11 SELECIONADA"); // sem bandeira
          PID = 11;
          break;
        case PID_F:
          Serial.println("ESTRATEGIA 22 SELECIONADA"); // bandeira
          PID = 22;
          break;
        default:
          break;
      }
    }
  }
}
void SELECIONA_ESTRATEGIA(void) {
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  while (ESTRATEGIA == 0) {
    if (IrReceiver.decode()) {
      IrReceiver.resume();
      switch (IrReceiver.decodedIRData.decodedRawData) {
        case ESTRATEGIA_A:
          Serial.println("ESTRATEGIA A SELECIONADA");
          ESTRATEGIA = 1;
          break;
        case ESTRATEGIA_B:
          Serial.println("ESTRATEGIA B SELECIONADA");
          ESTRATEGIA = 2;
          break;
        case ESTRATEGIA_C:
          Serial.println("ESTRATEGIA C SELECIONADA");
          ESTRATEGIA = 3;
          break;
        case ESTRATEGIA_D:
          Serial.println("ESTRATEGIA D SELECIONADA");
          ESTRATEGIA = 4;
          break;
        default:
          break;
      }
    }
  }
  digitalWrite(13, HIGH);
}

void EXECUTA_ESTRATEGIA_B(void) {
  error = -2;
  Serial.println(ESTRATEGIA);

  switch (ESTRATEGIA) {

    case 1: { // CurvaEsqFUNDO
      TEMPO = millis();
      Serial.print("2");

      SET_MOTORS(0, 255); // gira pra esquerda
      delay(200);

      float MULTIPLIER = 1.7;
      int SPEED = 255 / MULTIPLIER;
      SET_MOTORS(SPEED * MULTIPLIER, SPEED);

      while ((millis() - TEMPO) < 700) {
        CALC_ERROR1();
        CHECK_STOP();
      }

      SET_MOTORS(0, 0);
      break;
    }

    case 2: { // CurvaDirFUNDO
      TEMPO = millis();
      Serial.println("Executou estrategia B");

      SET_MOTORS(0, 255); // gira pra direita
      delay(200);

      float MULTIPLIER = 1.7;
      int SPEED = 255 / MULTIPLIER;
      SET_MOTORS(SPEED * MULTIPLIER, SPEED);

      while ((millis() - TEMPO) < 700) {
        CALC_ERROR1();
        CHECK_STOP();
      }

      SET_MOTORS(0, 0);
      break;
    }

    case 3: { // CurvaRapidaMEIO
      TEMPO = millis();
      CALC_ERROR1();

      SET_MOTORS(0, 255); // Gira pra esquerda 90°
      delay(180);
      SET_MOTORS(0, 0);

      const int vel = 200;
      SET_MOTORS(vel, vel / 2.2); // faz a curva

      while ((millis() - TEMPO) < 500) {
        CALC_ERROR1();
        CHECK_STOP();
      }

      SET_MOTORS(0, 0);
      break;
    }

    case 4: { // FRENTESUICIDIO
      FILTRO();
      const int LIMIAR_ATAQUE = 200;

      while (distancia_left > LIMIAR_ATAQUE || distancia_right > LIMIAR_ATAQUE) {
        FILTRO();

        if (distancia_right > LIMIAR_ATAQUE) SET_MOTORS(0, 255);
        if (distancia_left > LIMIAR_ATAQUE)  SET_MOTORS(255, 0);

        CHECK_STOP(); // segurança: permite parar se necessário
      }

      delay(300);
      CALC_ERROR1();
      break;
    }

    /* 
    case 5: { // CURVADOMEIOSUICIDIO
      TEMPO = millis();
      SET_MOTORS(-255, 255);
      delay(60);
      SET_MOTORS(255, 160);
      delay(700);
      SET_MOTORS(255, 255);
      while (1) CHECK_STOP(); 
      break;
    }
    */
  }
}

void EXECUTA_ESTRATEGIA_SB(void) {
  switch (ESTRATEGIA) {
    case 1:  // Sem estratégia
      CALC_ERROR2();
      break;
    case 2:  //FRENTESUICIDIO
      break;
    case 3:  // CURVADO
      break;
      
  }
}

void FILTRO(void) {
  distancia_left = sensor_left.readRangeContinuousMillimeters();
  distancia_right = sensor_right.readRangeContinuousMillimeters();
  distancia_left = (distancia_left + sensor_left.readRangeContinuousMillimeters())/2;
  distancia_right = (distancia_right + sensor_right.readRangeContinuousMillimeters())/2;
  distancia_left = (distancia_left + sensor_left.readRangeContinuousMillimeters())/2;
  distancia_right = (distancia_right + sensor_right.readRangeContinuousMillimeters())/2;
}

void MOTORS_CONFIG(void) {
  DDRB = ((1 << ESQ_IN2_PIN) | (1 << ESQ_PWM_PIN) | (1 << DIR_PWM_PIN));
  DDRC = ((1 << DIR_IN1_PIN) | (1 << DIR_IN2_PIN) | (1 << DIR_STDBY));
  DDRD = ((1 << ESQ_IN1_PIN) | (1 << ESQ_STDBY));

  TCCR1A = ((1 << COM1A1) | (1 << COM1B1) | (1 << WGM10));
  TCCR1B = ((1 << WGM12) | (1 << CS10));

  OCR1A = 0;  // Motor da Esquerda
  OCR1B = 0;  // Motor da Direita

  PORTC = (1 << DIR_STDBY);
  PORTD = (1 << ESQ_STDBY);
}

void SET_MOTORS(int PWM_ESQ, int PWM_DIR) {
  if (PWM_ESQ > 0) {
    if (PWM_ESQ > 255)
      PWM_ESQ = 255;
    OCR1A = PWM_ESQ;
    PORTD &= ~(1 << ESQ_IN1_PIN);
    PORTB |= (1 << ESQ_IN2_PIN);
  } else {
    if (PWM_ESQ < -255)
      PWM_ESQ = -255;
    OCR1A = -PWM_ESQ;
    PORTD |= (1 << ESQ_IN1_PIN);
    PORTB &= ~(1 << ESQ_IN2_PIN);
  }
  if (PWM_DIR > 0) {
    if (PWM_DIR > 255)
      PWM_DIR = 255;
    OCR1B = PWM_DIR;
    PORTC |= (1 << DIR_IN1_PIN);
    PORTC &= ~(1 << DIR_IN2_PIN);
  } else {
    if (PWM_DIR < -255)
      PWM_DIR = -255;
    OCR1B = -PWM_DIR;
    PORTC &= ~(1 << DIR_IN1_PIN);
    PORTC |= (1 << DIR_IN2_PIN);
  }
}

void CHECK_START(void) {
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  while (IrReceiver.decodedIRData.decodedRawData != START_CODE) {
    if (IrReceiver.decode()) {
      IrReceiver.resume();
    }
  }
  Serial.println("Start");
  digitalWrite(13, LOW);
}

void CHECK_STOP(void) {
  if (IrReceiver.decode()) {
    IrReceiver.resume();
    if (IrReceiver.decodedIRData.decodedRawData == STOP_CODE) {
      while (1)
        SET_MOTORS(0, 0);
      ;
    }
  }
}

void VL53L0X_CONFIG(void) {
  pinMode(dist_left, OUTPUT);
  pinMode(dist_right, OUTPUT);
  digitalWrite(dist_left, LOW);
  digitalWrite(dist_right, LOW);

  delay(500);
  Wire.begin();

  pinMode(dist_left, INPUT);
  delay(150);
  sensor_left.init(true);
  delay(100);
  sensor_left.setAddress((uint8_t)01);


  pinMode(dist_right, INPUT);
  delay(150);
  sensor_right.init(true);
  delay(100);
  sensor_right.setAddress((uint8_t)02);


  sensor_left.setMeasurementTimingBudget(20000);
  sensor_right.setMeasurementTimingBudget(20000);
  sensor_left.setSignalRateLimit(0.5);   // Default is 0.25
  sensor_right.setSignalRateLimit(0.5);  // Default is 0.25
  sensor_left.startContinuous();
  sensor_right.startContinuous();
}
