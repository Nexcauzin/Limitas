// ATIVAÇÃO DO PRINT A CADA 100ms
// false = dt de 100ms
// true = dt de execução do código
bool testaDt = false;

// ========== Definições para o Motor ===========
#include <Servo.h>
#define receptorPin 13
#define pinoMotor 9

Servo motorPin;                                          // Criando uma classe servo com nome sinalReceptor

// Limites em Microssegundos
const int valor_min = 1321;
const int valor_max = 1983;
const int valor_act = 2000;
int pwm_out = 0;


// =========== Definições para o PID =============
// Variáveis do PID
float Kp = 50;                                            // Ganho proporcional
float Ki = 100;                                            // Ganho integral
float Kd = 50;                                            // Ganho derivativo
float N = 2;                                              // Filtro do termo derivativo
float integral;

// Variáveis avulsas (organizar depois)
float lastError = 0;                                     // Último erro
float targetValue = 670;                                 // Valor de referência (Limite de Potência)
float margemSeguranca = 630;                             // Valor adotado para o acionamento do PD (Potência)
float pwm_static = 0;


// =========== SENSOR DE TENSÃO ==================
// Sensor de Tensão
#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

// ========== Sensor de Corrente ================
#define PinoCorrente A0
// Definindo variáveis do Sensor de Corrente
//float sensitivity = 58.895;                             // Para a calibração de 25V por 0,833333ohm deu 58.895
float sensitivity = 68;
int adcValue = 0;
int offsetVoltage = 2500;
double adcVoltage = 0;
double currentValue = 0;
float Corrente = 0;

// Variáveis para a potência
const int numReadings = 30;                               // Tamanho da amostra de média móvel
int readings[numReadings];                                // Vetor para armazenar as variáveis da amostra
int readIndex = 0;                                        // Variável que vai iterar o vetor e apontar para o novo valor
float Potencia = 0.0;                                     // Variável final da potência

// Variáveis para o cálculo do Throttle (%) (Fora de uso por agora, só para rela)
float subtrai = 0;
float valorConvertido = 0;

// Variáveis para controle de tempo
unsigned long previousMillis = 0;
unsigned long printMillis = 0;
const unsigned long testaPrint = 100;
unsigned long interval = 0;

// ===== Funções para a operação de Potência =====
// Função para calcular a Corrente
float calculoCorrente(){                                      
  // Cálculo da corrente (sensor ACS712)
  adcValue = analogRead(PinoCorrente);
  adcVoltage = (adcValue / 1024.0) * 5000;
  currentValue = ((adcVoltage - offsetVoltage) / sensitivity);
  return mediaMovel(currentValue);                                            // Retorna o valor calculado
}

// Média móvel potência
float mediaMovel(float corLida){
  // Rotação do vetor
  for(int i = numReadings-1; i > 0; i--){
    readings[i] = readings[i-1];
  }

  readings[0] = corLida;                // Armazena última potência lida

  float soma = 0.0;                     // Definindo a variável zerada da soma que será realizada

  // Realiza a soma dos valores armazenados
  for(int i = 0; i < numReadings; i++){
    soma += readings[i];
  }

  return soma/numReadings;              // Retorna a média móvel
}

// Calcula o Throttle (%)
float throttleValue(float dadoBruto){
  return map(dadoBruto, 1321, 1828, 0, 100);               // Retorno
}

// Calcula o tempo entre execuções
unsigned long loopInterval(){
  unsigned long currentMillis = millis();                   // Pega o tempo atual em milissegundos
  unsigned long interval = currentMillis - previousMillis;  // Calcula o intervalo
  previousMillis = currentMillis;                           // Atualiza o previousMillis para o próximo loop  
  return interval;                                          // Retorna o intervalo
}


// ================== SETUP =====================
void setup(){
  pinMode(receptorPin, INPUT);
  motorPin.attach(pinoMotor);                 // Definindo o pino do motor como a saída de PWM
  motorPin.writeMicroseconds(valor_min);      // Inicializando o sinal com PWM em 0
  // Inicializando o vetor das amostras
  for (int i = 0; i < numReadings; i++){      // Vai criar o vetor zerado
    readings[i] = 0;
  }
  delay(5000);                                // Só por desencargo da consciência
  Serial.begin(9600);
}


// ================== LOOP =====================
void loop(){    
  // ============== Extração do dt ====================
  unsigned long currentMillis = millis();
  interval = loopInterval();

  int pwm_in = pulseIn(receptorPin, HIGH);                                           // Faz a leitura do pino do receptor
  // ============== Cálculo da Potência ===============
  Corrente = abs(calculoCorrente());                                                 // Obtendo a corrente
  float Tensao = ina219.getBusVoltage_V();                                                          // Obtendo a tensão
  Potencia = Corrente * Tensao;                                          // Calculando a potência

  if(pwm_in <= valor_act || Potencia <= margemSeguranca){                            // Testa se deve acionar o PD
    pwm_out = constrain(pwm_in, valor_min, valor_max);                               // Força que o valor esteja no intervalo de interesse
  }


  else{
    pwm_static = pwm_in;                                                              // Pegando o valor que vai ser iterado pelo PD4
    float lastPotencia = Potencia;
    while(1){                                                                         // Loop infinito
      // ============== Extração do dt ====================
      unsigned long currentMillis = millis();
      interval = loopInterval();
      
      // ============== Teste de continuidade ==============
      // Testa se deve sair da lógica comparando a entrada com o valor de ativação do PD
      int pwm_in = pulseIn(receptorPin, HIGH);                                        // Lendo o novo sinal do receptr
      if(pwm_in <= pwm_static){                                                       // Testa se deve desativar o PD
        pwm_out = constrain(pwm_in, valor_min, valor_max);                            // Se sim, pega o novo pwm_out
        break;                                                                        // Finaliza a execução do PD
      }

      // ============== Cálculo da Potência ===============
      Corrente = abs(calculoCorrente());                                               // Obtendo a corrente
      float Tensao = ina219.getBusVoltage_V();                                               // Obtendo a tensão
      Potencia = Corrente * Tensao;                                                    // Calculando a potência

      // ============== Operações do PID ===================
      float error = targetValue - Potencia;                                            // Calcula o termo potencial
      float proportional = Kp * error;
      integral += Ki * interval * error;
      float filteredError = (N/(1 + N * interval)) * (error - lastError);
      float derivative = Kd * filteredError;                                           // Calcula o termo derivativo
      // Caso o derivativo saia errado:
      // float derivative = (lastPotencia - Potencia) * Kd * interval
  
      // Se tiver dando problema, o erro pode ser essa lógica de somar o pwm_static
      float output = proportional + integral + derivative + pwm_static;                          // Calculo do PD (output)

      pwm_out = constrain(output, valor_min, valor_max);                               // O valor que será impresso no motor
      pwm_static = pwm_out;                                                            // Preparando o pwm_static para a proxima iteração
      lastError = error;                                                               // Erro da próxima iteração
      motorPin.writeMicroseconds(pwm_out);                                             // Escreve o PWM no pino do motor

      //=========== Valor do Throttle do motor ===========
      float Throttle = throttleValue(pwm_static);                                      // Calcula a % de Throttle do motor
      
      // Testa que tipo de print vai fazer
      if(testaDt){
        Serial.println(String(Potencia) + "," + String(Tensao) + "," + String(Corrente) + "," + String(Throttle)  + "," + String(pwm_out) + "," + String(interval)); // Pra salvar o Log 
      }

      else{
        if (currentMillis - printMillis >= testaPrint){
          Serial.println(String(Potencia) + "," + String(Tensao) + "," + String(Corrente) + "," + String(Throttle)  + "," + String(pwm_out) + "," + String(currentMillis - printMillis)); // Pra salvar o Log 
          printMillis = currentMillis;
        }
      }
    }
  }

  //=========== Valor do Throttle do motor ===========
  float Throttle = throttleValue(pwm_out);                                             // Calcula a % de Throttle do motor

  motorPin.writeMicroseconds(pwm_out);                                                 // Escreve o PWM no pino do motor
  
    if(testaDt){
      Serial.println(String(Potencia) + "," + String(Tensao) + "," + String(Corrente) + "," + String(Throttle)  + "," + String(pwm_out) + "," + String(interval)); // Pra salvar o Log 
    }

    else{
      if (currentMillis - printMillis >= testaPrint){
        Serial.println(String(Potencia) + "," + String(Tensao) + "," + String(Corrente) + "," + String(Throttle)  + "," + String(pwm_out) + "," + String(currentMillis - printMillis)); // Pra salvar o Log 
        printMillis = currentMillis;
    }
  }
}