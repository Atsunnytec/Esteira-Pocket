#include "detectorDeRotulos.h"

void setup()
{
  Serial.begin(115200);
  Serial.println("Detector de Rotulos ready");

  EEPROM.begin(EEPROM_SIZE);
  loadParametersFromEEPROM();
  // presetEEPROM();

  pinInitialization();
  extIOs.init();
  desligaTodosOutputs();
  enc.setVelocidade(velocidade);
  delay(1000);

  mutex_rs485 = xSemaphoreCreateMutex();
  eventQueue = xQueueCreate(3, sizeof(Evento));

  if (flag_debugEnabled)
  {
    xTaskCreatePinnedToCore(t_debug, "debug task", 2048, NULL, PRIORITY_1, NULL, CORE_0);
  }
  xTaskCreatePinnedToCore(t_blink, "blink task", 1024, NULL, PRIORITY_1, NULL, CORE_0);
  xTaskCreatePinnedToCore(t_ihm, "ihm task", 4096, NULL, PRIORITY_3, NULL, CORE_0);
  xTaskCreatePinnedToCore(t_botoesIhm, "botoesIhm task", 4096, NULL, PRIORITY_3, &h_botoesIhm, CORE_0);
  xTaskCreatePinnedToCore(t_eeprom, "eeprom task", 4096, NULL, PRIORITY_3, &h_eeprom, CORE_0);

  ihm.showStatus2msg("SEGURE PLAY");
}

void loop()
{
  Evento evento = recebeEventos();

  // to do: rotina que reseta o encoder se o buffer ficar vazio.

  switch (fsm)
  {
  case ESTADO_STOP:
  {
    if (evento == EVT_PLAY_PAUSE)
    {
      changeFsmState(ESTADO_EM_FUNCIONAMENTO);
      ihm.goToMenu(&menu_monitor);
      ihm.showStatus2msg("EM FUNCIONAMENTO");
    }
    else if (evento == EVT_HOLD_PLAY_PAUSE)
    {
      changeFsmState(ESTADO_CALIBRACAO);
      ihm.showStatus2msg("CALIBRACAO");
    }
    break;
  }
  case ESTADO_EM_FUNCIONAMENTO:
  {
    // to do: task que controla a rejeicao aqui no "em funcionamento"
    // to do: timeout para o ciclo. Se calibrar errado o ciclo pode ficar infinito.

    if (evento == EVT_PLAY_PAUSE)
    {
      changeFsmState(ESTADO_STOP);
      fsm_bloqueio = fase1; // to do: fragilidade, pode interromper o bloqueio no meio e bugar.
      ihm.showStatus2msg("STOP");
      break;
    }

    taskSensores();
    taskBloqueio();

    if (fsm_substate == fase1)
    {
      if (evento == EVT_SENSOR)
      {
        Serial.print(enc.getPosition());
        Serial.println(" detectou");
        colocaNovoGarrafaoNaFila();
        incrementaContadores();
        fsm_substate = fase2;
      }
    }
    else if (fsm_substate == fase2)
    {
      if (enc.compareDistance(posicaoFimZonaCegaDeEntrada.peek()))
      {
        posicaoFimZonaCegaDeEntrada.pop();
        fsm_substate = fase3;
      }
    }
    else if (fsm_substate == fase3)
    {
      if (enc.compareDistance(posicaoInicioZonaCegaDeSaida.peek()))
      {
        fsm_substate = fase4;
      }
      else if (evento == EVT_SENSOR)
      {
        // preparaBloqueioDoAES();
        nok++;
        flag_bloqueio = true;
        Serial.print(enc.getPosition());
        Serial.println(" bloqueia AES");
        fsm_substate = fase4;
      }
    }
    else if (fsm_substate == fase4)
    {
      if (enc.compareDistance(posicaoInicioZonaCegaDeSaida.peek()))
      {
        posicaoInicioZonaCegaDeSaida.pop();
        noGood.push(flag_bloqueio?true:false);
        flag_bloqueio = false;
        fsm_substate = fase5;
      }
    }
    else if (fsm_substate == fase5)
    {
      if (enc.compareDistance(posicaoTamanhoDoGarrafao.peek()))
      {
        posicaoTamanhoDoGarrafao.pop();
        Serial.print("fim garrafao");
        salvaContadorNaEEPROM();
        ok = contador - nok;
        fsm_substate = fase1;
      }
    }
    break;
  }
  case ESTADO_FALHA:
  {
    break;
  }
  case ESTADO_TESTE_VELOCIDADE:
  {
    static uint32_t timer_velocidade = 0;

    if (fsm_substate == fase1)
    {
      if (digitalRead(PIN_SB1) == LOW)
      {
        timer_velocidade = millis();
        fsm_substate = fase2;
      }
    }
    else if (fsm_substate == fase2)
    {
      if (digitalRead(PIN_SB1) == HIGH)
      {
        // calcule a velocidade da esteira com base no tempo que um produto cilindrico demora para passar na frente do sensor.
        Serial.print(millis() - timer_velocidade);
        Serial.println("ms");
        fsm_substate = fase1;
      }
    }
    break;
  }
  case ESTADO_CALIBRACAO:
  {
    static int16_t productCount = 0;
    static int16_t pulseCount = 0;
    static int32_t *pulse4;
    static int32_t *pulse5;
    static int32_t *pulse8;

    enum
    {
      CALIBRACAO_SETUP = fase1,
      CALIBRACAO_AMOSTRAGEM,
      CALIBRACAO_CALCULOS,
    };

    if (evento == EVT_PLAY_PAUSE)
    {
      changeFsmState(ESTADO_STOP);
      ihm.showStatus2msg("STOP");
      break;
    }

    if (fsm_substate == CALIBRACAO_SETUP)
    {
      pulseCount = 0;
      productCount = 0;
      pulse4 = 0;
      pulse5 = 0;

      pulse4 = (int32_t *)malloc(sizeof(int32_t) * qtdDeProdutosNaCalibracao);
      pulse5 = (int32_t *)malloc(sizeof(int32_t) * qtdDeProdutosNaCalibracao);
      pulse8 = (int32_t *)malloc(sizeof(int32_t) * qtdDeProdutosNaCalibracao);

      fsm_substate = CALIBRACAO_AMOSTRAGEM;
    }
    else if (fsm_substate == CALIBRACAO_AMOSTRAGEM) // to do: alterar o nome das fases.
    {
      taskSensores();

      if (evento == EVT_SENSOR)
      {
        pulseCount++;
        if (pulseCount == 1)
        {
          enc.setCounter(0);
        }

        float pos = enc.getPosition();
        Serial.print("sb: ");
        Serial.print(pos);
        Serial.println("mm");

        if (pulseCount == 4)
        {
          pulse4[productCount] = pos;
        }
        else if (pulseCount == 5)
        {
          pulse5[productCount] = pos;
        }
        else if (pulseCount == 8)
        {
          pulse8[productCount] = pos;
          pulseCount = 0;
          productCount++;
          fsm_substate = CALIBRACAO_CALCULOS;
        }
      }
    }
    else if (fsm_substate == CALIBRACAO_CALCULOS)
    {
      if (productCount == qtdDeProdutosNaCalibracao)
      {
        fimZonaCegaDeEntrada = maximo(pulse4, sizeof(pulse4)) + margemDeSeguranca;
        inicioZonaCegaDeSaida = minimo(pulse5, sizeof(pulse5)) - margemDeSeguranca;
        tamanhoDoGarrafao = maximo(pulse8, sizeof(pulse8)) + margemDeSeguranca;
        Serial.print("Calibrou:");
        Serial.print(" p4: ");
        Serial.print(fimZonaCegaDeEntrada);
        Serial.print("  p5: ");
        Serial.print(inicioZonaCegaDeSaida);
        Serial.print("  p8: ");
        Serial.print(tamanhoDoGarrafao);
        Serial.println();
        changeFsmState(ESTADO_EM_FUNCIONAMENTO);
        ihm.showStatus2msg("EM FUNCIONAMENTO");
        ihm.goToMenu(&menu_monitor);
        break;
      }
      else
      {
        fsm_substate = CALIBRACAO_AMOSTRAGEM;
      }
    }
    break;
  }
  }
}