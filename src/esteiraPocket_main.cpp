#include "esteiraPocket.h"

void setup()
{
  Serial.begin(115200);
  Serial.println("Esteira Pocket ready");

  pinInitialization();
  extIOs.init();
  desligaTodosOutputs();
  delay(1000);

  esteira.setup(velocidade, rampa);
  desligaEsteira();

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

    if (STOP.check(true))
    {
      desligaEsteira();
      changeFsmState(ESTADO_EMERGENCIA);
      Serial.println("STOP");
      break;
    }

    if (fsm_substate == fase1)
    {
      ligaEsteira();
      fsm_substate = fase2;
    }
    else if (fsm_substate == fase2)
    {
      if (SP.check())
      {
        timer_fechamento = millis();
        Serial.println("produto detectado.");
        timer_atrasoProduto = millis();
        changeFsmState(ESTADO_CICLO);
      }
    }

    break;
  }
  case ESTADO_CICLO:
  {
    static uint32_t timer_atrasoPistao = 0;
    static uint32_t timer_saida = 0;
    static uint32_t timer_duracaoPistao = 0;
    static bool flag_stop = false;

    if (evento == EVT_PARADA_EMERGENCIA)
    {
      desligaEsteira();
      desacionaPistao();
      changeFsmState(ESTADO_EMERGENCIA);
      Serial.println("EMERGENCIA");
      break;
    }

    if (STOP.check(true))
    {
      flag_stop = true;
      Serial.println("STOP");
      break;
    }

    if (fsm_substate == fase1)
    {
      if (millis() - timer_atrasoProduto >= atrasoProduto)
      {
        desligaEsteira();
        timer_atrasoPistao = millis();
        fsm_substate = fase2;
      }
    }
    else if (fsm_substate == fase2)
    {
      if (millis() - timer_atrasoPistao >= atrasoPistao)
      {
        Serial.println("aciona pistao");
        acionaPistao();
        timer_duracaoPistao = millis();
        fsm_substate = fase3;
      }
    }
    else if (fsm_substate == fase3)
    {
      if (millis() - timer_duracaoPistao >= duracaoPistao)
      {
        Serial.println("desaciona pistao");
        desacionaPistao();
        timer_saida = millis();
        fsm_substate = fase4;
      }
    }
    else if (fsm_substate == fase4)
    {
      if (millis() - timer_saida >= atrasoSaida)
      {
        contador++;
        Serial.print("contador: "); Serial.print(contador);
        Serial.print("  tempo fechamento: "); Serial.print(millis() - timer_fechamento); Serial.print("ms");
        Serial.print("   prod/min: ");
        Serial.println((float)60 * 1000 / (millis() - timer_ppm));
        timer_ppm = millis();
        if (flag_stop)
        {
          desligaEsteira();
          desacionaPistao();
          changeFsmState(ESTADO_STOP);
        }
        else
        {
          changeFsmState(ESTADO_EM_FUNCIONAMENTO);
        }
      }
    }
    break;
  }
  }
}