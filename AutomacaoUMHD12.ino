/*  ESP32 DevKit V1                    01.10.2020
     GPIO2 - Led azul do ESP, e LED_3 da placa   
     PINO3 - no ESP32 - tecla EN, reseta quando nivel baixo   
     GPIO0 - no ESP32 - tecla BOOT, para carregar o programa
     Fase R - input pin = 34
     Fase S - input pin = 35
     Fase T - input pin = 36
     Corrente - input pin = 39
     Trigger do Sensor Ultrasonico = pin = 32
     Echo do Sensor Ultrasonicopino = pin = 33
     Sensor Input Stand-by - input pin = 13
     Sensor Input Motor ON/OFF - input pin = 12
     Sensor Status Modo de Operação - input pin = 14 (GND = LOCAL)
   
   Blynk.virtualWrite(V0): Tensao de Fase R
   Blynk.virtualWrite(V1): Tensao de Fase S
   Blynk.virtualWrite(V2): Tensao de Fase T
   Blynk.virtualWrite(V3): Corrente do Motor
   Blynk.virtualWrite(V4): Indicador de nivel de agua
   Blynk.virtualWrite(V5): LED App - Bomba Submersa (ON/OFF)
   Blynk.virtualWrite(V6): LED App - Nivel de agua (Baixo/Ok)
   Blynk.virtualWrite(V7): LED App - Motor (ON/OFF)
   Blynk.virtualWrite(V8): LED App - Solenoide (ON/OFF)
   Blynk.virtualWrite(V9): Status do Modo de Operação - Local(0)/Remoto
   Blynk.virtualWrite(V10): Status do Motor - Ligado/Desligado
   BLYNK_WRITE(V20): Botao App - Comando Liga Motor
   BLYNK_WRITE(V21): Botao App - Comando Desliga Motor
   BLYNK_WRITE(V22): Botao App - Comando RESET
   
   EEPROM LIB - Created for arduino-esp32 on 25 Dec, 2017
   by Elochukwu Ifediora (fedy0)
   converted to nvs by lbernstone - 06/22/2019
      
 */
 
#define BLYNK_PRINT Serial
#include "EmonLib.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Ultrasonic.h>
#include "EEPROM.h"
#include "esp_system.h"

EnergyMonitor emon1, emon2, emon3;
#define vCalibrationR 163.4
#define vCalibrationS 163.5
#define vCalibrationT 163.2
#define ADC_INPUT 39                              // Define GPIO39 entrada analógica
#define ADC_BITS 10                               // Força EmonLib usar AC com resolução de 10bit
#define ADC_COUNTS (1<<ADC_BITS)

#define pino_trigger 32                           // Define os pinos para o trigger do Sensor Ultrasonico
#define pino_echo 33                              // Define os pinos para o echo do Sensor Ultrasonico
Ultrasonic ultrasonic(pino_trigger, pino_echo);   // Inicializa o sensor nos pinos definidos acima

#define EEPROM_SIZE 1                             // Define tamanho da memória

BlynkTimer timer;
char auth[] = "6l84bGRUSx8r7-XvHux4aB7PVxbPlN4V"; // Autenticação no Servidor Blynk

// Credenciais da rede WiFi
//char ssid[] = "TIM 2";
//char pass[] = "5221529352";
char ssid[] = "WIFI_BOMBA";
char pass[] = "1234567p";
//char ssid[] = "NET_2G45211C";
//char pass[] = "B345211C";
//char ssid[] = "iPhone";
//char pass[] = "12345670";
//char ssid[] = "Celular Gladimir";
//char pass[] = "1234567890";  

const int Botao = 23;                       // Botao para uso geral
const int StatusModoOperacao = 14;          // Sensor do Modo de Operação Local/Remoto (GND = LOCAL)
const int infoMotorONOFF = 12;              // Sensor de Motor ligado / desligado (GND = Ligado)
int ModoOperacao = 0;                       // Variavel para enviar sinalização ao Blink
const int LED_1 = 22;                       // Led 1 - Sinalização do Motor ON/OFF
const int LED_2 = 4;                        // Led 2 - Sinalização Nivel de agua baixo
const int LED_3 = 2;                        // Led 3 - led azul do ESP
const int LED_4 = 15;                       // Sinalização Nivel de agua baixo
const int MotorON = 21;                     // Rele 1 - Motor ON
const int MotorOFF = 19;                    // Rele 2 - Motor OFF
const int BombaSub = 18;                    // Rele 3 - BombaSubmersa
const int Solenoide = 5;                    // Rele 4 - Solenoide do ar
int StateMotor = 0;                         // Informacao de estado do Motor (posicao 0 da memoria)
int LastStateMotor = 0;                     // Próxima informacao de estado do Motor
int connection_loss_count = 0;              // Contado de perda de conexão "tentativas"
int infoMotor = 0;                          // Motor ligado / desligado (GND = Ligado)
int LastInfoMotor = 0;                      // Variavel para teste do sensor Motor ON/OFF

int BotaoB1;
BLYNK_WRITE(V20)                            // BotaoB1 = Virtual 20 do APP
{ BotaoB1 = param.asInt(); }                // Função de Liga Motor

int BotaoB2;                    
BLYNK_WRITE(V21)                            // BotaoB2 = Virtual 21 do APP
{ BotaoB2 = param.asInt(); }                // Função de Desliga Motor

int BotaoRESET;                    
BLYNK_WRITE(V22)                            // BotaoRESET = Virtual 22 do APP
{ BotaoRESET = param.asInt(); }             // Função de RESET

// ---------------------------------------------------------------------------------------------------
//                                WATCHDOG
// ---------------------------------------------------------------------------------------------------

hw_timer_t *timerRST = NULL; //faz o controle do temporizador (interrupção por tempo)

// hw_timer_t * timerBegin(uint8_t num, uint16_t divider, bool countUp)
    /*
      num: é a ordem do temporizador. Podemos ter quatro temporizadores, então a ordem pode ser [0,1,2,3].
      divider: É um prescaler (reduz a frequencia por fator). Para fazer um agendador de um segundo, 
      usaremos o divider como 80 (clock principal do ESP32 é 80MHz). Cada instante será T = 1/(80) = 1us
      countUp: True o contador será progressivo
    */
void IRAM_ATTR resetModule(){                             // função que o temporizador irá chamar
     ets_printf("(Watchdog) Sistema será reiniciado..."); // imprime no log
     esp_restart();                                       // reinicia o ESP
}
// ---------------------------------------------------------------------------------------------------
//                                SETUP
// ---------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(Botao,INPUT);
  pinMode(StatusModoOperacao,INPUT_PULLUP);
  pinMode(infoMotorONOFF,INPUT_PULLUP);
  pinMode(LED_1,OUTPUT);
  pinMode(LED_2,OUTPUT);
  pinMode(LED_3,OUTPUT);
  pinMode(LED_4,OUTPUT);
  pinMode(MotorON,OUTPUT);
  pinMode(MotorOFF,OUTPUT);
  pinMode(BombaSub,OUTPUT);
  pinMode(Solenoide,OUTPUT);
  
  digitalWrite(MotorON, HIGH);
  digitalWrite(MotorOFF, HIGH);
  digitalWrite(BombaSub, HIGH);
  digitalWrite(Solenoide, HIGH);

  /*Blynk.begin(auth, ssid, pass);                 // Se não conectar ?! 
  timer.setInterval(2000L, ConnectionCheck);     // Verificar resultado, antes ou depois do RELIGAR Motor ?!
  timer.setInterval(1000L, Rotina);
  Blynk.virtualWrite(V9, "RESET");               // Envia ao Blynk informação
  */
    
  EEPROM.begin(EEPROM_SIZE);
  delay(10);
  StateMotor = EEPROM.read(0);                   // Lê da memória o ultimo estado do Motor ON(255)/OFF(0)
  LastInfoMotor = LOW;                           // 
    if (StateMotor > 100) {                      // Se for ON, espera um tempo e religa o motor
            LastInfoMotor = !StateMotor;         // INVERSO do StateMOtor Garante que será diferente da ultima infoMotor
            Serial.println("Reiniciando por RESET... temporizando Bomba (1 minuto...)");
            delay(75000);                        // A cada reset espera XX tempo para religar a Bomba
            digitalWrite(LED_1, HIGH);           // Liga o LED_1
            digitalWrite(MotorON, LOW);          // Liga por 2s o Relé 1
            delay(2000);
            digitalWrite(MotorON, HIGH);
            }
        
  Blynk.begin(auth, ssid, pass);                 // Se não conectar ?! 
  timer.setInterval(2000L, ConnectionCheck);     // Verificar resultado, antes ou depois do RELIGAR Motor ?!
  timer.setInterval(1000L, Rotina);
  Blynk.virtualWrite(V9, "RESET");               // Envia ao Blynk informação
  switch (StateMotor){
     case 0:
          Blynk.notify ("Bomba Desligada!");     // Envia Push Blynk informação - Motor Ligado
          Blynk.virtualWrite(V10, "DESLIGADO");  // Envia ao Blynk informação - Ligado
          break;
     case 255:
          Blynk.notify ("Bomba Ligada!");        // Envia Push Blynk informação - Motor Ligado
          Blynk.virtualWrite(V10, "LIGADO");     // Envia ao Blynk informação - Ligado
          break;
     }
// ----------------------------- SETUP do Watchdog ---------------------------------------------------
  timerRST = timerBegin(1, 80, true);                           //Watchdog timerID 1, div 80
  timerAttachInterrupt(timerRST, &resetModule, true);           //Watchdog timer, callback, interrupção de borda
  timerAlarmWrite(timerRST, 50000000, true);                    //Watchdog timer, tempo (us), repetição
  timerAlarmEnable(timerRST);                                   //Watchdog habilita a interrupção

  // timerWrite(timer, 0); //USAR no loop - função que reseta o contador do timer definido no setup
// ----------------------------- FIM do SETUP do Watchdog --------------------------------------------

  emon1.voltage(34, vCalibrationR, 1.7);  // Fase R - input pin = 34, vCalibration, calibration, phase_shift
  emon2.voltage(35, vCalibrationS, 1.7);  // Fase S - input pin = 35, vCalibration, calibration, phase_shift
  emon3.voltage(36, vCalibrationT, 1.7);  // Fase T - input pin = 36, vCalibration, calibration, phase_shift
  emon1.current(ADC_INPUT, 22.5);         // Corrente - input pin = 39, calibration
}

// ---------------------------------------------------------------------------------------------------
//                                INICIO DOS LOOP´s 
// ---------------------------------------------------------------------------------------------------

void ConnectionCheck() {
  if(Blynk.connected() == true){
    Serial.print("Blynk conectado...");
    long rssi = WiFi.RSSI();
    Serial.print("       RF Signal Level: ");
    Serial.println(rssi);                     // Escreve o indicador de nível de sinal Wi-Fi
    digitalWrite(LED_3, HIGH);                // Liga o LED_3
    connection_loss_count = 0;                // Reseta o contator de perda tentativas de conexão
  }
  else if(Blynk.connected() == false){
    connection_loss_count = connection_loss_count + 1;
    Serial.print("Blynk sem conexão!!! Tentativa de reconexão nr.: ");
    Serial.println(connection_loss_count);
    digitalWrite(LED_3, LOW);                 // Desliga o LED_3
      if(WiFi.status() == WL_CONNECTED){
      Serial.println("Wi-Fi ok...");
      } else {
      Serial.println("Sem rede de Wi-Fi !!!");
      WiFi.reconnect();
      }
    Blynk.connect();
   }
}

void Rotina() {

  timerWrite(timerRST, 0);                      // A cada "loop executado" reseta o contador do timer Watchdog 
  emon1.calcVI(160, 2000);                      // Quantidade de semiciclos, tempo para leitura para calculo
  emon2.calcVI(160, 2000);                      // Quantidade de semiciclos, tempo para leitura para calculo
  emon3.calcVI(160, 2000);                      // Quantidade de semiciclos, tempo para leitura para calculo
     
     // Exibe sensor de Fase R
  Serial.print("Fase R: ");
  Serial.println(emon1.Vrms, 1);                // Lê a tensão, 1 casa decimal
  Blynk.virtualWrite(V0,(int)emon1.Vrms);       // Envia ao Blynk parte inteira de emon.Vrms (XXX.xx)
  
     // Exibe sensor de Fase S
  Serial.print("Fase S: ");
  Serial.println(emon2.Vrms, 1);                // Lê a tensão, 1 casa decimal
  Blynk.virtualWrite(V1,(int)emon2.Vrms);       // Envia ao Blynk parte inteira de emon.Vrms (XXX.xx)
  
     // Exibe sensor de Fase T
  Serial.print("Fase T: ");
  Serial.println(emon3.Vrms, 1);                // Lê a tensão, 1 casa decimal
  Blynk.virtualWrite(V2,(int)emon3.Vrms);       // Envia ao Blynk parte inteira de emon.Vrms (XXX.xx)
  
     // Exibe sensor de Corrente R
  Serial.print("Corrente R: ");
  Serial.println(emon1.Irms, 1);                // Lê informaçõa de corrente, com uma casa decimal
  Blynk.virtualWrite(V3,(int)emon1.Irms);       // Exibe sensor ultrasonico inteira de emon.Irms (XXX.xx)

     // Le as informacoes do sensor ultrasonico - nível de água
  float cmMsec, inMsec;
  long microsec = ultrasonic.timing();
  cmMsec = ultrasonic.convert(microsec, Ultrasonic::CM);
  Serial.print("Nivel da agua (cm): ");          // Exibe a leitura
  Serial.println(cmMsec,1 );            
  Blynk.virtualWrite(V4,(int)cmMsec);            // Envia ao Blynk - Indicador de Nível de Água
    if (cmMsec < 100) {
          Blynk.virtualWrite(V6, 255);           // Envia ao Blynk informação - Nível de Água Baixo 
          digitalWrite(LED_4, HIGH);             // Liga o LED_4 - Nível de Água Baixo 
        } else {
          Blynk.virtualWrite(V6, 50);            // Envia ao Blynk informação - Nível de Água OK
          digitalWrite(LED_4, LOW);              // Desliga o LED_4 - Nível de Água OK
        }
        
     // Verifica Modo de Operação
  ModoOperacao = digitalRead(StatusModoOperacao);   
  Serial.print("Modo de Operação: ");
  if (ModoOperacao == HIGH){                     // Lê a entrada - GPIO 14
      Serial.println("Remoto");              
      Blynk.virtualWrite(V9, "Remoto");          // Envia ao Blynk informação - Modo Remoto
      } else {
      Serial.println("Local");               
      Blynk.virtualWrite(V9, "Local");           // Envia ao Blynk informação - Modo Local
      }

      // Lê ultimo estado do Motor do contato auxiliar da contactora (ligado ou delisgado)
      // Grava na memória e envia ao Blynk 
       
  infoMotor = digitalRead(infoMotorONOFF);       // Lê o pino 12 - vem do contato auxiliar da contactora  
  Serial.println(LastInfoMotor);
  Serial.println(infoMotor); 
  
  if (infoMotor != LastInfoMotor) {              // Compara se teve alteração 
    if (infoMotor == LOW) {                      // Se estiver em nivel baixo = Motor LIGADO
          EEPROM.write(0, 255);                  // Escreve na memória 255
          EEPROM.commit();
          digitalWrite(LED_1, HIGH);             // Liga o LED_1
          Blynk.virtualWrite(V7, 255);           // Envia ao Blynk informação - Motor Ligado
          Blynk.notify ("Bomba Ligada!");        // Envia Push Blynk informação - Motor Ligado
          Blynk.virtualWrite(V10, "LIGADO");     // Envia ao Blynk informação - Ligado
          // envia comando
          digitalWrite(MotorON, LOW);            // Comando de liga, ativa o Rele 1 por 2 segundos
          delay(2000);
          digitalWrite(MotorON, HIGH);
          } else {                               // Caso contrário = Motor DESLIGADO
            EEPROM.write(0, 0);                  // Escreve na memória 0
            EEPROM.commit();
            digitalWrite(LED_1, LOW);            // Desliga o LED_1
            Blynk.virtualWrite(V7, 50);          // Envia ao Blynk informação - Motor Desligado
            Blynk.notify ("Bomba Desligada!");   // Envia Push Blynk informação - Motor Desligado
            Blynk.virtualWrite(V10, "DESLIGADO");// Envia ao Blynk informação - Desligado
            // envia comando
            digitalWrite(MotorOFF, LOW);         // Comando de desliga, ativa o Rele 2 por 2 segundos
            delay(2000);
            digitalWrite(MotorOFF, HIGH);     
            }
          LastInfoMotor = infoMotor;
   }

        // Lê ultimo comando do Motor salvo na memoria e escreve nas saidas do ESP
      StateMotor = EEPROM.read(0);
      switch (StateMotor){
        case 0:
        Serial.println("Leitura da memoria Status do Motor: Desligado");
        break;
        case 255:
        Serial.println("Leitura da memoria Status Motor: Ligado");
        break;
      }
      Serial.println("---------------------------------------------");

   // Le as informacoes dos botoes de comando do Motor do APP
  if (BotaoB1 == 1) {
          digitalWrite(MotorON, LOW);            // Comando de liga, ativa o Rele 1 por 2 segundos
          delay(2000);
          digitalWrite(MotorON, HIGH);
        } 
  if (BotaoB2 == 1) {
          digitalWrite(MotorOFF, LOW);         // Comando de desliga, ativa o Rele 2 por 2 segundos
          delay(2000);
          digitalWrite(MotorOFF, HIGH);  
        }
//______________________________________________________________________________________________________________  
  if (emon1.Irms < 40 && StateMotor > 100) {     // Se corrente baixa e motor ligado...
         digitalWrite(BombaSub, LOW);            // Liga Relé 3
         digitalWrite(LED_2, HIGH);              // Liga LED_2
         Blynk.virtualWrite(V5, 255);            // Envia ao Blynk informação - BombaSub Ligado
         //
         digitalWrite(Solenoide, LOW);           // Liga Relé 4
         Blynk.virtualWrite(V8, 255);            // Envia ao Blynk informação - Solenoide Ligado
         } else {                                // Caso contrário desliga...
           digitalWrite(BombaSub, HIGH);         // Desliga Relé 3
           digitalWrite(LED_2, LOW);             // Desliga LED_2
           Blynk.virtualWrite(V5, 50);           // Envia ao Blynk informação - BombaSub Desligado
           //
           digitalWrite(Solenoide, HIGH);        // Desliga Relé 4
           Blynk.virtualWrite(V8, 50);           // Envia ao Blynk informação - Solenoide Desligado
           }    
  while (BotaoRESET == 1) {
        Serial.println("Botão de RESET pressionado...");
        digitalWrite(LED_3, LOW);                // Desliga o LED_3
        delay(250);
        digitalWrite(LED_3, HIGH);               // Liga o LED_3
        delay(250);
        }
}

void loop(){
  timer.run();
  if (Blynk.connected() == true) {
      Blynk.run();
      }
}
