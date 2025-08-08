#include <IRremote.hpp>
#include <Wire.h>
#include <VL53L0X.h>
VL53L0X sensor_left;
VL53L0X sensor_right;

#define dist_left 2
#define dist_right A3

uint16_t distancia_left;
uint16_t distancia_right;
#define ESQ_IN1_PIN PD7  //7
#define ESQ_IN2_PIN PB0  //8
#define ESQ_PWM_PIN PB1  //9
#define ESQ_STDBY PD6    //6

#define DIR_PWM_PIN PB2  //10
#define DIR_IN1_PIN PC1  //A1
#define DIR_IN2_PIN PC0  //A0
#define DIR_STDBY PC2    //A2

#define START_CODE 0x81
#define IR_RECEIVE_PIN 11

#define LIMIAR 1200
int error = -2;
int error_antg;

void setup() {
  Serial.begin(9600);
  Serial.print("0");
  VL53L0X_CONFIG();
  IrReceiver.begin(IR_RECEIVE_PIN);
  MOTORS_CONFIG();
  CHECK_START();
}

void loop() {
  // adicionar logica aqui
}

void MOVIMENTO_INICIAL() {
  const LIMIAR_ATAQUE = 200;
  while (distancia_left > LIMIAR_ATAQUE || distancia_right > LIMIAR_ATAQUE) {
    distancia_right = sensor_right.readRangeContinuousMillimeters();
    distancia_left = sensor_left.readRangeContinuousMillimeters();
    if (distancia_right > LIMIAR_ATAQUE) SET_MOTORS(0,255)
    if (distancia_left > LIMIAR_ATAQUE) SET_MOTORS(255,0)
  }

  delay(300)
  SET_MOTORS(255,255)
}

void CALCULA_ERRO(void) {
  error_antg=error;
   distancia_left = sensor_left.readRangeContinuousMillimeters();
  distancia_right = sensor_right.readRangeContinuousMillimeters();

  distancia_left = (distancia_left + sensor_left.readRangeContinuousMillimeters()) / 2;
  distancia_right = (distancia_right + sensor_right.readRangeContinuousMillimeters()) / 2;


  if (distancia_left < LIMIAR) {
    if (distancia_right < LIMIAR) {
      error = 0;
    } else {
      error = -1;
    }
  } else if (distancia_right < LIMIAR) {
    error = 1;
  }else if(error_antg<=0){
    error=-2;
  }else if(error_antg<=0){
    error=2;
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
  sensor_left.startContinuous();
  sensor_right.startContinuous();
}
void SET_MOTORS(int PWM_ESQ, int PWM_DIR) {
  const int MAX_PWM = 200;
  if (PWM_ESQ > 0) {
    if (PWM_ESQ > MAX_PWM)
      PWM_ESQ = MAX_PWM;
    OCR1A = PWM_ESQ;
    PORTD &= ~(1 << ESQ_IN1_PIN);
    PORTB |= (1 << ESQ_IN2_PIN);
  } else {
    if (PWM_ESQ < -MAX_PWM)
      PWM_ESQ = -MAX_PWM;
    OCR1A = -PWM_ESQ;
    PORTD |= (1 << ESQ_IN1_PIN);
    PORTB &= ~(1 << ESQ_IN2_PIN);
  }
  if (PWM_DIR > 0) {
    if (PWM_DIR > MAX_PWM)
      PWM_DIR = MAX_PWM;
    OCR1B = PWM_DIR;
    PORTC |= (1 << DIR_IN1_PIN);
    PORTC &= ~(1 << DIR_IN2_PIN);
  } else {
    if (PWM_DIR < -MAX_PWM)
      PWM_DIR = -MAX_PWM;
    OCR1B = -PWM_DIR;
    PORTC &= ~(1 << DIR_IN1_PIN);
    PORTC |= (1 << DIR_IN2_PIN);
  }
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

void CHECK_START(void) {
  while (IrReceiver.decodedIRData.decodedRawData != START_CODE) {
    if (IrReceiver.decode()) {
      IrReceiver.resume();
    }
  }
}
