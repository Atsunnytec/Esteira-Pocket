#include "esteiraPocket.h"

void setup()
{
  Serial.begin(115200);
  Serial.println("Esteira Pocket ready");

  pinInitialization();
  extIOs.init();
  desligaTodosOutputs();
  desligaEsteira();
  delay(1000);

  eventQueue = xQueueCreate(2, sizeof(Evento));

  if (flag_debugEnabled)
  {
    xTaskCreatePinnedToCore(t_debug, "debug task", 2048, NULL, PRIORITY_1, NULL, CORE_0);
  }
  xTaskCreatePinnedToCore(t_blink, "blink task", 1024, NULL, PRIORITY_1, NULL, CORE_0);
}

void loop()
{
  Evento evento = recebeEventos();

  switch (fsm)
  {
  case ESTADO_EMERGENCIA:
  {
    if (START.check())
    {
      changeFsmState(ESTADO_EM_FUNCIONAMENTO);
    }
    break;
  }
  case ESTADO_EM_FUNCIONAMENTO:
  {
    if(evento == EVT_PARADA_EMERGENCIA)
    {
      desligaEsteira();
      changeFsmState(ESTADO_EMERGENCIA);
      break;
    }

    // to do: botao stop nf
    if (START.check())
    {
      desligaEsteira();
      changeFsmState(ESTADO_EMERGENCIA);
      break;
    }

    static uint32_t timer_esteira = 0;

    if (fsm_substate == fase1)
    {
      if (millis() - timer_esteira >= 2000)
      {
        ligaEsteira();
        timer_esteira = millis();
        fsm_substate = fase2;
      }
    }
    else if (fsm_substate == fase2)
    {
      if (millis() - timer_esteira >= 2000)
      {
        desligaEsteira();
        timer_esteira = millis();
        fsm_substate = fase1;
      }
    }
    else if (fsm_substate == fase3)
    {
      if (millis() - timer_esteira >= 2000)
      {
        timer_esteira = millis();
        fsm_substate = fase4;
      }
    }
    else if (fsm_substate == fase4)
    {

      fsm_substate = fase5;
    }
    else if (fsm_substate == fase5)
    {

      fsm_substate = fase1;
    }
    break;
  }
  case ESTADO_CICLO:
  {
    break;
  }
  }
}