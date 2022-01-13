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
  xTaskCreatePinnedToCore(t_emergencia, "emergencia task", 1024, NULL, PRIORITY_2, NULL, CORE_0);
}

void loop()
{
  Evento evento = recebeEventos();

  switch (fsm)
  {
  case ESTADO_EMERGENCIA:
  {
    static uint32_t timer_emergencia = 0;
    const uint32_t timeout_emergencia = 250;

    if (fsm_substate == fase1)
    {
      timer_emergencia = millis();
      fsm_substate = fase2;
    }
    else if (fsm_substate == fase2)
    {
      if (evento == EVT_PARADA_EMERGENCIA)
      {
        timer_emergencia = millis();
      }
      else if (millis() - timer_emergencia > timeout_emergencia)
      {
        Serial.println("STOP");
        changeFsmState(ESTADO_STOP);
      }
    }
    break;
  }
  case ESTADO_STOP:
  {
    if (evento == EVT_PARADA_EMERGENCIA)
    {
      desligaEsteira();
      changeFsmState(ESTADO_EMERGENCIA);
      Serial.println("EMERGENCIA");
      break;
    }

    if (START.check())
    {
      Serial.println("EM FUNCIONAMENTO");
      changeFsmState(ESTADO_EM_FUNCIONAMENTO);
    }
    break;
  }
  case ESTADO_EM_FUNCIONAMENTO:
  {
    if (evento == EVT_PARADA_EMERGENCIA)
    {
      desligaEsteira();
      changeFsmState(ESTADO_EMERGENCIA);
      Serial.println("EMERGENCIA");
      break;
    }

    // to do: botao stop nf
    if (STOP.check(true))
    {
      desligaEsteira();
      changeFsmState(ESTADO_EMERGENCIA);
      Serial.println("STOP");
      break;
    }

    break;
  }
  case ESTADO_CICLO:
  {
    static uint32_t timer_esteira = 0;
    static bool flag_stop = false;

    if (evento == EVT_PARADA_EMERGENCIA)
    {
      desligaEsteira();
      desacionaPistao();
      changeFsmState(ESTADO_EMERGENCIA);
      Serial.println("EMERGENCIA");
      break;
    }

        // to do: botao stop nf
    if (STOP.check(true))
    {
      flag_stop = true;
      // to do: na última fase, checa pela flag stop
      Serial.println("STOP");
      break;
    }

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
  }
}