// azul com reflexo de barata
#include <IRremote.hpp>
#include <Wire.h>
#include <VL53L0X.h>
VL53L0X sensor_front;
VL53L0X sensor_left;
VL53L0X sensor_right;
int dirDisparo = 1;
#define dist_left 2
#define dist_front 3
#define dist_right 4

#define IR_RECEIVE_PIN 11
#define READY_CODE 0x80
#define START_CODE 0x81
#define STOP_CODE 0x82

#define ESQ_PWM_PIN 9
#define ESQ_IN1_PIN 7
#define ESQ_IN2_PIN 8

#define DIR_PWM_PIN 10
#define DIR_IN1_PIN A0
#define DIR_IN2_PIN A1

#define LIMIAR 600

uint16_t distancia_left;
uint16_t distancia_front;
uint16_t distancia_right;

int ERRO = -3;
int ERRO_ANTIGO = -3;

void setup() {
  IrReceiver.begin(IR_RECEIVE_PIN);

  MOTORS_CONFIG();

  VL53L0X_CONFIG();

  CALCULA_ERRO();
  CALCULA_ERRO();

  CHECK_START();
}

void loop() {
  CHECK_STOP();

  CALCULA_ERRO();
  const int mov = 70;
  SET_MOTORS(mov * dirDisparo * -1, mov * dirDisparo * 1);
  delay(120);
  if (dirDisparo == 1) {
    dirDisparo = -4;
  } else {
    dirDisparo = 1;
  }
  // if (abs(ERRO) > abs(ERRO_ANTIGO)) {
  //   if (ERRO > 0) {
  //     SET_MOTORS(-50, -50);
  //     delay(50);
  //     SET_MOTORS(255, -255);
  //   } else {
  //     SET_MOTORS(-50, -50);
  //     delay(50);
  //     SET_MOTORS(-255, 255);
  //   }
  // } else {
  //   switch (ERRO) {
  //     case -3:
  //       int mov = 70;
  //       SET_MOTORS(mov * dirDisparo * -1.5, mov * dirDisparo * 1);
  //       delay(150);
  //       dirDisparo = dirDisparo * -1;
  //       break;
  //     case -2:
  //       SET_MOTORS(100, 255);
  //       break;
  //     case -1:
  //       if (distancia_front < 100) {
  //         SET_MOTORS(255, 255);
  //       } else {
  //         SET_MOTORS(160, 255);
  //       }
  //       break;
  //     case 0:
  //       if (distancia_front < 400) {
  //         SET_MOTORS(255, 255);
  //       } else {
  //         SET_MOTORS(160, 160);
  //       }
  //       break;
  //     case 1:
  //       if (distancia_front < 100) {
  //         SET_MOTORS(255, 255);
  //       } else {
  //         SET_MOTORS(255, 160);
  //       }
  //       break;
  //     case 2:
  //       SET_MOTORS(255, 100);
  //       break;
  //     case 3:
  //       SET_MOTORS(180 * dirDisparo, 90 * dirDisparo);
  //       dirDisparo *= -1;
  //       break;
  //   }
  // }
}

void CALCULA_ERRO(void) {
  ERRO_ANTIGO = ERRO;

  distancia_left = sensor_left.readRangeContinuousMillimeters();
  distancia_front = sensor_front.readRangeContinuousMillimeters();
  distancia_right = sensor_right.readRangeContinuousMillimeters();

  distancia_left = (distancia_left + sensor_left.readRangeContinuousMillimeters()) / 2;
  distancia_front = (distancia_front + sensor_front.readRangeContinuousMillimeters()) / 2;
  distancia_right = (distancia_right + sensor_right.readRangeContinuousMillimeters()) / 2;

  if (distancia_front < LIMIAR) {
    ERRO = 0;
    if (distancia_left < LIMIAR) {
      ERRO = ERRO - 1;
    }
    if (distancia_right < LIMIAR) {
      ERRO = ERRO + 1;
    }
  } else if (distancia_left < LIMIAR) {
    ERRO = -2;
  } else if (distancia_right < LIMIAR) {
    ERRO = 2;
  } else if (ERRO_ANTIGO <= 0) {
    ERRO = -3;
  } else {
    ERRO = 3;
  }
}

void VL53L0X_CONFIG(void) {
  pinMode(dist_left, OUTPUT);
  pinMode(dist_front, OUTPUT);
  pinMode(dist_right, OUTPUT);
  digitalWrite(dist_left, LOW);
  digitalWrite(dist_front, LOW);
  digitalWrite(dist_right, LOW);

  delay(500);
  Wire.begin();

  pinMode(dist_left, INPUT);
  delay(150);
  sensor_left.init(true);
  delay(100);
  sensor_left.setAddress((uint8_t)01);

  pinMode(dist_front, INPUT);
  delay(150);
  sensor_front.init(true);
  delay(100);
  sensor_front.setAddress((uint8_t)02);

  pinMode(dist_right, INPUT);
  delay(150);
  sensor_right.init(true);
  delay(100);
  sensor_right.setAddress((uint8_t)03);

  sensor_left.setMeasurementTimingBudget(20000);
  sensor_front.setMeasurementTimingBudget(20000);
  sensor_right.setMeasurementTimingBudget(20000);
  sensor_left.startContinuous();
  sensor_front.startContinuous();
  sensor_right.startContinuous();
}

void SET_MOTORS(int PWM_ESQ, int PWM_DIR) {
  if (PWM_ESQ > 0) {
    analogWrite(ESQ_PWM_PIN, PWM_ESQ);
    digitalWrite(ESQ_IN2_PIN, HIGH);
    digitalWrite(ESQ_IN1_PIN, LOW);
  } else {
    analogWrite(ESQ_PWM_PIN, -PWM_ESQ);
    digitalWrite(ESQ_IN2_PIN, LOW);
    digitalWrite(ESQ_IN1_PIN, HIGH);
  }
  if (PWM_DIR > 0) {
    analogWrite(DIR_PWM_PIN, PWM_DIR);
    digitalWrite(DIR_IN1_PIN, HIGH);
    digitalWrite(DIR_IN2_PIN, LOW);
  } else {
    analogWrite(DIR_PWM_PIN, -PWM_DIR);
    digitalWrite(DIR_IN1_PIN, LOW);
    digitalWrite(DIR_IN2_PIN, HIGH);
  }
}

void MOTORS_CONFIG(void) {
  pinMode(ESQ_PWM_PIN, OUTPUT);
  pinMode(ESQ_IN1_PIN, OUTPUT);
  pinMode(ESQ_IN2_PIN, OUTPUT);
  pinMode(DIR_PWM_PIN, OUTPUT);
  pinMode(DIR_IN1_PIN, OUTPUT);
  pinMode(DIR_IN2_PIN, OUTPUT);

  digitalWrite(ESQ_PWM_PIN, LOW);
  digitalWrite(ESQ_IN1_PIN, LOW);
  digitalWrite(ESQ_IN2_PIN, LOW);
  digitalWrite(DIR_PWM_PIN, LOW);
  digitalWrite(DIR_IN1_PIN, LOW);
  digitalWrite(DIR_IN2_PIN, LOW);
}

void CHECK_START(void) {
  while (IrReceiver.decodedIRData.decodedRawData != START_CODE) {
    if (IrReceiver.decode()) {
      IrReceiver.resume();
    }
  }
}

void CHECK_STOP(void) {
  if (IrReceiver.decode()) {
    IrReceiver.resume();
    if (IrReceiver.decodedIRData.decodedRawData == STOP_CODE) {
      while (IrReceiver.decodedIRData.decodedRawData != START_CODE)
        SET_MOTORS(0, 0);
      ;
    }
  }
}
