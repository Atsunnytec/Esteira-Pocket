#include "esteiraPocket.h"

void setup()
{
  Serial.begin(115200);
  Serial.println("Esteira Pocket ready");

  pinInitialization();
  extIOs.init();
  desligaTodosOutputs();
  enc.setVelocidade(velocidade);
  delay(1000);

  eventQueue = xQueueCreate(3, sizeof(Evento));

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
  case ESTADO_STOP:
  {
    if (evento == EVT_PLAY_PAUSE)
    {
      changeFsmState(ESTADO_EM_FUNCIONAMENTO);
    }
    break;
  }
  case ESTADO_EM_FUNCIONAMENTO:
  {

    if (evento == EVT_PLAY_PAUSE)
    {
      changeFsmState(ESTADO_STOP);
      break;
    }

    if (fsm_substate == fase1)
    {
      if (evento == EVT_SENSOR)
      {
        fsm_substate = fase2;
      }
    }
    else if (fsm_substate == fase2)
    {
      fsm_substate = fase3;
    }
    else if (fsm_substate == fase3)
    {
      fsm_substate = fase4;
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