#ifndef detectorDeRotulos_h
#define detectorDeRotulos_h

#include <Arduino.h>
#include <HardwareSerial.h>
#include <extendedIOs.h>
#include <esp32Industrial_v2.1.h>
#include <ihmSunnytecMaster_v2.0.h>
#include <protocoloIhm_v2.0.h>
#include <EEPROM.h>
#include <checkSensorPulse.h>
#include <encoderVirtual.h>
#include <fifo.h>
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
  ESTADO_STOP,
  ESTADO_CALIBRACAO,
  ESTADO_EM_FUNCIONAMENTO,
  ESTADO_TESTE_VELOCIDADE,
  ESTADO_FALHA,
} fsm;
uint16_t fsm_substate = fase1;

// variaveis:
bool flag_debugEnabled = true;
bool flag_bloqueio = false;


// objetos
extendedIOs extIOs = extendedIOs(PIN_IO_CLOCK, PIN_IO_LATCH, PIN_INPUT_DATA, PIN_OUTPUT_DATA);
QueueHandle_t eventQueue;
checkSensorPulse SP = checkSensorPulse(PIN_SP);
checkSensorPulse START = checkSensorPulse(PIN_START);
checkSensorPulse STOP = checkSensorPulse(PIN_STOP);

uint32_t ppr = 2000;
uint32_t diametro = 60;
encoderVirtual enc = encoderVirtual(ppr, diametro);

uint32_t contador = 0;

// parametros:
int32_t velocidade = 73; // 0 - 100 %
int32_t atrasoProduto = 700;
int32_t atrasoSaida = 700;
int32_t duracaoPistao = 700; //mm

// prototypes:
void desligaTodosOutputs();
Evento recebeEventos();
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

void incrementaContadores()
{
  contador++;
}



void taskSensores()
{
  // if (sb2.check() || sb1.check())
  if (sb2.onChange() || sb1.onChange())
  {
    enviaEvento(EVT_SENSOR);
  }
}

void acionaPistao()
{
  digitalWrite(PIN_PISTAO, HIGH);
}

void desacionaPistao()
{
  digitalWrite(PIN_PISTAO, LOW);
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