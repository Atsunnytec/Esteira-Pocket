#ifndef detectorDeRotulos_h
#define detectorDeRotulos_h

#include <Arduino.h>
#include <HardwareSerial.h>
#include <extendedIOs.h>
#include <esp32Industrial_v2.1.h>
#include <checkSensorPulse.h>
#include <ihmSunnytecMaster_v2.0.h>
#include <motorPWM.h>
#include "defines.h"

enum Evento
{
  EVT_NENHUM,
  EVT_FALHA,
  EVT_SEM_FALHAS,
  EVT_PARADA_EMERGENCIA,
  EVT_TESTE,
  EVT_PLAY_PAUSE,
  EVT_START,
  EVT_SENSOR,
  EVT_HOLD_PLAY_PAUSE,
};

enum Estado
{
  ESTADO_EMERGENCIA,
  ESTADO_STOP,
  ESTADO_EM_FUNCIONAMENTO,
  ESTADO_CICLO,
} fsm;
uint16_t fsm_substate = fase1;

// variaveis:
bool configuracaoPelaIhm = true;
bool flag_debugEnabled = true;
bool flag_bloqueio = false;
uint32_t timer_atrasoProduto = 0;
uint32_t timer_ppm = 0;
uint32_t timer_fechamento = 0;

// objetos
extendedIOs extIOs = extendedIOs(PIN_IO_CLOCK, PIN_IO_LATCH, PIN_INPUT_DATA, PIN_OUTPUT_DATA);
ihmSunnytecMaster ihm{protocoloIhm{PIN_RS485_RX, PIN_RS485_TX, PIN_RS485_EN}};
TaskHandle_t h_eeprom;
SemaphoreHandle_t mutex_rs485;
QueueHandle_t eventQueue;
checkSensorPulse SP = checkSensorPulse(PIN_SP);
checkSensorPulse START = checkSensorPulse(PIN_START);
checkSensorPulse STOP = checkSensorPulse(PIN_STOP);
motorPWM esteira = motorPWM(PIN_MOTOR);

// parametros:
int32_t contador = 0;
int32_t atrasoProduto = 210; //ms
int32_t duracaoPistao = 400; //ms
// parametros de manutencao:
int32_t contadorAbsoluto = 0;   // to do:
int32_t velocidade = 95;        // 0 - 100 % // to do:
int32_t rampa = 50;             // ms
int32_t atrasoPistao = 5;       //ms
int32_t atrasoSaida = 50;       // ms
int32_t atrasoNovoProduto = 50; //ms

// menus
Menu menu_contador = Menu("Contador", READONLY, &contador, " ");
Menu menu_atrasoProduto = Menu("Atraso Produto", PARAMETRO, &atrasoProduto, "ms", 50u, 0u, 5000);
Menu menu_duracaoPistao = Menu("Duracao Pistao", PARAMETRO, &duracaoPistao, "ms", 50u, 0u, 5000);
// menus de manutencao
Menu menu_velocidade = Menu("Velocidade", PARAMETRO, &velocidade, "%", 1u, 10u, 100);

// prototypes:
void desligaTodosOutputs();
Evento recebeEventos();
void ligaEsteira();
void desligaEsteira();
void enviaEvento(Evento event);
void changeFsmState(Estado estado);
void acionaPistao();
void desacionaPistao();
void taskSensores();
int32_t maximo(int32_t *, int16_t);
int32_t minimo(int32_t *, int16_t);
void incrementaContadores();
void resetDosContadores();
void colocaNovoGarrafaoNaFila();
void liberaMenusDaIhm();

// functions:

void incrementaContadores()
{
  contador++;
}

void acionaPistao()
{
  digitalWrite(PIN_PISTAO, LOW);
}

void desacionaPistao()
{
  digitalWrite(PIN_PISTAO, HIGH);
}

void ligaEsteira()
{
  esteira.aceleraEsteira();
}

void desligaEsteira()
{
  esteira.desaceleraEsteira();
}

void paradaEmergenciaEsteira()
{
  // to do:
}

void changeFsmState(Estado estado)
{
  fsm = estado;
  fsm_substate = fase1;
}

Evento recebeEventos()
{
  Evento receivedEvent = EVT_NENHUM;
  xQueueReceive(eventQueue, &receivedEvent, 0);
  return receivedEvent;
}

void enviaEvento(Evento event)
{
  if (xQueueSend(eventQueue, (void *)&event, 10 / portTICK_PERIOD_MS) == pdFALSE)
  {
    Serial.print("erro enviaEvento: ");
    Serial.println(event);
  }
  // Serial.print("enviou evento: "); Serial.println(event);
}

void desligaTodosOutputs()
{
  // to do:
  uint8_t output = 0;
  output = bit(DO4) | bit(DO5) | bit(DO6) | bit(DO7);
  extIOs.changeOutputState(output);
}

void t_ihm(void *p)
{
  ihm.configDefaultMsg("      Sunnybot");
  ihm.configDefaultMsg2((String)contadorAbsoluto);
  xSemaphoreTake(mutex_rs485, portMAX_DELAY);
  ihm.setup();
  ihm.desligaLEDvermelho();
  ihm.desligaLEDverde();
  xSemaphoreGive(mutex_rs485);

  delay(3000);

  liberaMenusDaIhm();
  ihm.goToMenu(&menu_contador);

  while (1)
  {
    xSemaphoreTake(mutex_rs485, portMAX_DELAY);
    ihm.checkAndUpdateScreen();
    xSemaphoreGive(mutex_rs485);
    delay(100);
  }
}

void t_botoesIhm(void *p)
{
  delay(4000); // aguarda o objeto ihm ser inicializado

  while (1)
  {
    delay(100);

    uint16_t bt = 0;

    xSemaphoreTake(mutex_rs485, portMAX_DELAY);
    bt = ihm.requestButtons();
    xSemaphoreGive(mutex_rs485);

    if (bt == TIMEOUT)
    {
      Serial.println("timeout ihm");
    }
    else if (bt == BOTAO_NENHUM)
    {
      // Serial.print("NENHUM...");
    }
    else if (bt == BOTAO_PLAY_PAUSE)
    {
      enviaEvento(EVT_PLAY_PAUSE);
      Serial.println("PLAY/PAUSE");
    }
    else if (bt == BOTAO_HOLD_PLAY_PAUSE)
    {
      enviaEvento(EVT_HOLD_PLAY_PAUSE);
      Serial.println("HOLD PLAY PAUSE");
    }
    else if (configuracaoPelaIhm) // só checa os botoes direcionais se a configuracao estiver liberada.
    {
      if (bt == BOTAO_CIMA)
      {
        Serial.println("CIMA");

        ihm.incrementaParametroAtual();

        Menu *checkMenu = ihm.getMenu();
        if (checkMenu == &menu_contador)
        {
          contador = 0;
        }
      }
      else if (bt == BOTAO_ESQUERDA)
      {
        Serial.println("ESQUERDA");
        ihm.goToPreviousMenu();
      }
      else if (bt == BOTAO_BAIXO)
      {
        Serial.println("BAIXO");

        ihm.decrementaParametroAtual();

        Menu *checkMenu = ihm.getMenu();
        if (checkMenu == &menu_contador)
        {
          contador = 0;
        }
      }
      else if (bt == BOTAO_DIREITA)
      {
        Serial.println("DIREITA");
        ihm.goToNextMenu();
      }
      else if (bt == BOTAO_HOLD_CIMA)
      {
        Serial.println("HOLD CIMA");
      }
      else if (bt == BOTAO_HOLD_ESQUERDA)
      {
        Serial.println("HOLD ESQUERDA");
      }
      else if (bt == BOTAO_HOLD_DIREITA)
      {
        Serial.println("HOLD DIREITA");
      }
      else if (bt == BOTAO_HOLD_BAIXO)
      {
        Serial.println("HOLD BAIXO");
      }
      else if (bt == BOTAO_HOLD_DIREITA_ESQUERDA)
      {
        Serial.println("HOLD DIREITA E ESQUERDA");

        // if (flag_manutencao == false)
        // {
        //   liberaMenusDeManutencao();
        //   // ihm.goToMenu(&menu_contadorAbsoluto);
        //   ihm.showStatus2msg("MANUTENCAO LIBERADA");
        //   flag_manutencao = true;
        // }
      }
    }
    else
    {
      Serial.print("desabilitado ou erro> bt=");
      Serial.println(bt);
    }
  }
}

void habilitaConfiguracaoPelaIhm()
{
  configuracaoPelaIhm = true;
}

void desabilitaConfiguracaoPelaIhm()
{
  configuracaoPelaIhm = false;
}

void liberaMenusDaIhm()
{
  ihm.addMenuToIndex(&menu_contador);
  ihm.addMenuToIndex(&menu_atrasoProduto);
  ihm.addMenuToIndex(&menu_duracaoPistao);
}

void t_debug(void *p)
{
  while (1)
  {
    delay(2000);
    Serial.print(" on:");
    Serial.print(millis() / 1000);
    Serial.print(" ");

    // Serial.print(" fsm:");
    // Serial.print(fsm);
    // Serial.print(" sub:");
    // Serial.print(fsm_substate);

    // Serial.print("noGood: ");
    // for (int i = 0; i < LIMITE_DE_PRODUTOS_NA_LINHA; i++)
    // {
    //   Serial.print(noGood.peekPosition(i));
    //   Serial.print("  ");
    // }

    Serial.println();
  }
}

void t_emergencia(void *p)
{
  unsigned int interval = 200;

  while (1)
  {
    delay(interval);

    extIOs.updateInputState();
    if (extIOs.checkInputState(PIN_EMERGENCIA) == LOW)
    {
      enviaEvento(EVT_PARADA_EMERGENCIA);
      // Serial.println("envia evt: emg");
    }
  }
}

void t_blink(void *p)
{
  unsigned int interval = 400;
  pinMode(PIN_STATUS, OUTPUT);

  while (1)
  {
    GPIO.out_w1ts = (1 << PIN_STATUS);
    delay(interval / 2);
    GPIO.out_w1tc = (1 << PIN_STATUS);
    delay(interval / 2);
    GPIO.out_w1ts = (1 << PIN_STATUS);
    delay(interval / 2);
    GPIO.out_w1tc = (1 << PIN_STATUS);
    delay(interval * 2.5);
  }
}

#endif