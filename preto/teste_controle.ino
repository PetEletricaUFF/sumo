/*
 * Código Simples para Testar Receptor de IR
 * * Este código lê os sinais de um controle remoto infravermelho
 * e exibe o código hexadecimal de cada botão pressionado no Monitor Serial.
 * * Conexão:
 * - Pino de SINAL do sensor IR -> Pino 11 do Arduino
 * - Pino VCC (ou +) do sensor IR -> Pino 5V do Arduino
 * - Pino GND (ou -) do sensor IR -> Pino GND do Arduino
 * * Requer a biblioteca "IRremote". Instale via Gerenciador de Bibliotecas.
 */

#include <IRremote.h>

// Define o pino onde o sensor de IR está conectado.
// Você pode mudar para qualquer outro pino digital se precisar.
const int RECV_PIN = 11;

// Cria um objeto 'irrecv' da classe IRrecv e informa o pino do receptor.
IRrecv irrecv(RECV_PIN);

// Cria um objeto 'results' para armazenar os resultados decodificados.
decode_results results;

void setup() {
  // Inicia a comunicação serial para que possamos ver os resultados no computador.
  // A velocidade (baud rate) é 9600.
  Serial.begin(9600); 

  // Inicia o receptor de IR.
  irrecv.enableIRIn(); 
  
  Serial.println("Receptor IR pronto. Aponte seu controle e pressione um botão.");
}

void loop() {
  // Verifica se o receptor de IR recebeu um sinal.
  // A função decode() armazena os dados decodificados no objeto 'results'.
  if (irrecv.decode(&results)) {
    
    // Imprime o código hexadecimal do botão pressionado no Monitor Serial.
    // Usamos HEX para garantir que o formato seja consistente entre diferentes controles.
    Serial.print("Código Recebido: ");
    Serial.println(results.value, HEX);

    // Se o código for FFFFFFFF, geralmente significa que o botão está sendo mantido pressionado.
    if (results.value == 0xFFFFFFFF) {
        Serial.println("(Sinal de repetição)");
    }

    // Prepara o receptor para receber o próximo sinal.
    // É muito importante chamar resume() para limpar o último código e aguardar o próximo.
    irrecv.resume(); 
  }
  
  // Um pequeno delay para evitar leituras instáveis, mas não é estritamente necessário.
  delay(100); 
}
