#include <IRremote.h>  // Biblioteca IRemote
#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor_front;
VL53L0X sensor_left;
VL53L0X sensor_right;

int RECV_PIN = 12;        // Arduino pino 12 conectado no Receptor IR
IRrecv irrecv(RECV_PIN);  // criando a instância
decode_results results;   // declarando os resultados

#define dist_left 3
#define dist_front 4
#define dist_right 7

#define READY_CODE 0x10
#define START_CODE 0x810
#define STOP_CODE 0x410

#define ENA 9
#define IN1 5
#define IN2 6
#define IN3 2
#define IN4 8
#define ENB 10

uint16_t distancia_left;
uint16_t distancia_front;
uint16_t distancia_right;

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(ENB, OUTPUT);
  digitalWrite(ENA, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(ENB, LOW);

  pinMode(dist_left, OUTPUT);
  pinMode(dist_front, OUTPUT);
  pinMode(dist_right, OUTPUT);
  digitalWrite(dist_left, LOW);
  digitalWrite(dist_front, LOW);
  digitalWrite(dist_right, LOW);

  delay(500);
  Wire.begin();

  digitalWrite(dist_left, HIGH);
  delay(150);
  sensor_left.init(true);
  delay(100);
  sensor_left.setAddress((uint8_t)01);

  digitalWrite(dist_front, HIGH);
  delay(150);
  sensor_front.init(true);
  delay(100);
  sensor_front.setAddress((uint8_t)02);

  digitalWrite(dist_right, HIGH);
  delay(150);
  sensor_right.init(true);
  delay(100);
  sensor_right.setAddress((uint8_t)03);

  sensor_left.startContinuous();
  sensor_front.startContinuous();
  sensor_right.startContinuous();

  irrecv.enableIRIn();  // Inicializa a recepção de códigos


  /*while (results.value != READY_CODE) {
    if (irrecv.decode(&results))  // se algum código for recebido
    {
      irrecv.resume();  // reinicializa o receptor
    }
  }*/
  while (results.value != START_CODE) {
    if (irrecv.decode(&results))  // se algum código for recebido
    {
      irrecv.resume();  // reinicializa o receptor
    }
  }
}

void loop() {
  if (results.value != STOP_CODE) {
    if (irrecv.decode(&results))  // se algum código for recebido
    {
      irrecv.resume();  // reinicializa o receptor
    }
  } else {
    PARAR();
    while (1)
      ;
  }

  distancia_front = sensor_front.readRangeContinuousMillimeters();
  distancia_left = sensor_left.readRangeContinuousMillimeters();
  distancia_right = sensor_right.readRangeContinuousMillimeters();

  FRENTE(50);
  GIRAR_ESQ(50);
  
  if (distancia_left < 200) {
    GIRAR_ESQ(150);
  } else if (distancia_right < 200) {
    GIRAR_DIR(100);
  } else if (distancia_front < 200) {
    FRENTE(100);
  } else {
    GIRAR_DIR(100);
  }
}


void GIRAR_ESQ(int PWM) {
  analogWrite(ENA, PWM);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, PWM);
}

void GIRAR_DIR(int PWM) {
  analogWrite(ENA, PWM);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, PWM);
}

void FRENTE(int PWM) {
  analogWrite(ENA, PWM);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, PWM);
}

void PARAR(void) {
  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}
