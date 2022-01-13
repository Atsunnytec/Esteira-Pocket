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
uint16_t fsm_bloqueio = fase1;

// variaveis:
bool flag_debugEnabled = true;
bool flag_bloqueio = false;
bool configuracaoPelaIhm = true;

// objetos
ihmSunnytecMaster ihm{protocoloIhm{PIN_RS485_RX, PIN_RS485_TX, PIN_RS485_EN}};
extendedIOs extIOs = extendedIOs(PIN_IO_CLOCK, PIN_IO_LATCH, PIN_INPUT_DATA, PIN_OUTPUT_DATA);
SemaphoreHandle_t mutex_rs485;
QueueHandle_t eventQueue;
TaskHandle_t h_botoesIhm;
TaskHandle_t h_eeprom;
checkSensorPulse sb1 = checkSensorPulse(PIN_SB1);
checkSensorPulse sb2 = checkSensorPulse(PIN_SB2);

uint32_t ppr = 2000;
uint32_t diametro = 60;
encoderVirtual enc = encoderVirtual(ppr, diametro);

int32_t flag_calibracao = false;

// to do: colocar essas variaveis em FIFOs.

// uint32_t posicaoFimZonaCegaDeEntrada = 0;
// uint32_t posicaoInicioZonaCegaDeSaida = 0;
// uint32_t posicaoTamanhoDoGarrafao = 0;
// uint32_t posicaoDeBloqueio = 0;
// uint32_t posicaoDeDesbloqueio = 0;

FIFO<uint32_t> posicaoFimZonaCegaDeEntrada = FIFO<uint32_t>(LIMITE_DE_PRODUTOS_NA_LINHA);
FIFO<uint32_t> posicaoInicioZonaCegaDeSaida = FIFO<uint32_t>(LIMITE_DE_PRODUTOS_NA_LINHA);
FIFO<uint32_t> posicaoTamanhoDoGarrafao = FIFO<uint32_t>(LIMITE_DE_PRODUTOS_NA_LINHA);
FIFO<uint32_t> posicaoDeBloqueio = FIFO<uint32_t>(LIMITE_DE_PRODUTOS_NA_LINHA);
FIFO<uint32_t> posicaoDeDesbloqueio = FIFO<uint32_t>(LIMITE_DE_PRODUTOS_NA_LINHA);
FIFO<bool> noGood = FIFO<bool>(LIMITE_DE_PRODUTOS_NA_LINHA);

// parametros:
int32_t velocidade = 73; // mm/seg
int32_t contadorAbsoluto = 0;
int32_t produto = 0;
int32_t atrasoBloqueio = 450; //mm
int32_t duracaoBloqueio = 50; //mm
int32_t contador = 0;
int32_t fimZonaCegaDeEntrada = 130;    //mm
int32_t inicioZonaCegaDeSaida = 220;   //mm
int32_t tamanhoDoGarrafao = 340;       //mm
int32_t sombra = 65;                   //mm
int32_t qtdDeProdutosNaCalibracao = 5; //mm
int32_t margemDeSeguranca = 10;        //mm
int32_t ok = 0;
int32_t nok = 0;
int32_t status = 0;

// to do: menu monitor: contador:
// com rotulo:
// sem rotulo:
// status:

//menus:
Menu menu_produto = Menu("Produto", PARAMETRO, &produto, " ", 1u, 0u, (unsigned)(EPR_maxProdutos - 1));
Menu menu_contador = Menu("Contador", READONLY, &contador, " ");
Menu menu_duracaoBloqueio = Menu("Duracao Bloqueio", PARAMETRO, &duracaoBloqueio, "mm", 1u, 0u, 10000, &produto);
Menu menu_atrasoBloqueio = Menu("Atraso Bloqueio", PARAMETRO, &atrasoBloqueio, "mm", 1u, 0u, 10000, &produto);
Menu menu_fimZonaCegaDeEntrada = Menu("Zona Cega Entrada", PARAMETRO, &fimZonaCegaDeEntrada, "mm", 1u, 20u, 10000, &produto);
Menu menu_inicioZonaCegaDeSaida = Menu("Zona Cega Saida", PARAMETRO, &inicioZonaCegaDeSaida, "mm", 1u, 20u, 10000, &produto);
Menu menu_tamanhoDoGarrafao = Menu("Tamanho Garrafao", PARAMETRO, &tamanhoDoGarrafao, "mm", 1u, 50u, 10000, &produto);
Menu menu_velocidade = Menu("Velocidade", PARAMETRO, &velocidade, "mm/s", 1u, 10u, 1000);                                 // to do: salvar na eeprom
Menu menu_margemDeSeguranca = Menu("Margem De Seguranca", PARAMETRO, &margemDeSeguranca, "mm", 1u, 0u, 1000);             // to do: salvar na eeprom
Menu menu_sombra = Menu("Sombra", PARAMETRO, &sombra, "mm", 1u, 0u, 1000);                                                // to do: salvar na eeprom
Menu menu_qtdProdutosNaCalibracao = Menu("Produtos Na Calib", PARAMETRO, &qtdDeProdutosNaCalibracao, "prod", 1u, 1u, 20); // to do: salvar na eeprom

Menu menu_monitor = Menu("Monitor", MONITOR, &contador);
Menu menu_ok = Menu("Sem rotulo", READONLY, &ok, " ");
Menu menu_nok = Menu("Com rotulo", READONLY, &nok, " ");
Menu menu_status = Menu("STATUS", PARAMETRO_STRING, &status); // to do:

// to do: menus de manutencao

// prototypes:
void desligaTodosOutputs();
void liberaMenusDaIhm();
Evento recebeEventos();
void enviaEvento(Evento event);
void changeFsmState(Estado estado);
void saveParametersToEEPROM();
void salvaContadorNaEEPROM();
void loadParametersFromEEPROM();
void presetEEPROM();
void saveProdutoToEEPROM(int32_t _produto);
void loadProdutoFromEEPROM();
void bloqueiaAES();
void desbloqueiaAES();
void taskBloqueio();
void taskSensores();
int32_t maximo(int32_t *, int16_t);
int32_t minimo(int32_t *, int16_t);
void incrementaContadores();
void resetDosContadores();
void colocaNovoGarrafaoNaFila();

// functions:
void taskBloqueio()
{
  if (fsm_bloqueio == fase1)
  {
    if (enc.compareDistance(posicaoDeBloqueio.peek()))
    {
      posicaoDeBloqueio.pop();
      if (noGood.peek() == true)
      {
        Serial.print(enc.getPosition());
        Serial.println(" bloqueia");
        bloqueiaAES();
      }
      fsm_bloqueio = fase2;
    }
  }
  else if (fsm_bloqueio == fase2)
  {
    if (enc.compareDistance(posicaoDeDesbloqueio.peek()))
    {
      posicaoDeDesbloqueio.pop();
      noGood.pop();
      Serial.print(enc.getPosition());
      Serial.println(" desbloqueia");
      desbloqueiaAES();
      fsm_bloqueio = fase1; // to do: resetar para fase 1 quando o ciclo der pause
    }
  }
}

void incrementaContadores()
{
  contador++;
  contadorAbsoluto++;
}

void colocaNovoGarrafaoNaFila()
{
  uint32_t posicaoGarrafao = enc.getPosition();
  posicaoFimZonaCegaDeEntrada.push(posicaoGarrafao + fimZonaCegaDeEntrada);
  posicaoInicioZonaCegaDeSaida.push(posicaoGarrafao + inicioZonaCegaDeSaida);
  posicaoTamanhoDoGarrafao.push(posicaoGarrafao + tamanhoDoGarrafao + sombra);
  posicaoDeBloqueio.push(posicaoTamanhoDoGarrafao.peek() + atrasoBloqueio);
  posicaoDeDesbloqueio.push(posicaoDeBloqueio.peek() + duracaoBloqueio);
}

int32_t maximo(int32_t *data, int16_t size)
{
  int32_t max = data[0];

  for (int i = 1; i < size; i++)
  {
    if (data[i] > max)
    {
      max = data[i];
    }
  }

  return max;
}

int32_t minimo(int32_t *data, int16_t size)
{
  int32_t min = data[0];

  for (int i = 1; i < size; i++)
  {
    if (data[i] < min)
    {
      min = data[i];
    }
  }

  return min;
}

void taskSensores()
{
  // if (sb2.check() || sb1.check())
  if (sb2.onChange() || sb1.onChange())
  {
    enviaEvento(EVT_SENSOR);
  }
}

void bloqueiaAES()
{
  extIOs.desligaOutput(PIN_BLOQUEIO);
}

void desbloqueiaAES()
{
  extIOs.ligaOutput(PIN_BLOQUEIO);
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

void t_eeprom(void *p)
{
  int16_t intervaloDeBackups = 10000; //ms

  while (1)
  {
    delay(intervaloDeBackups);
    saveParametersToEEPROM();
  }
}

void saveParametersToEEPROM()
{
  EEPROM.put(EPR_produto, produto);
  EEPROM.put(EPR_margemDeSeguranca, margemDeSeguranca);
  EEPROM.put(EPR_sombra, sombra);
  EEPROM.put(EPR_qtdDeProdutosNaCalibracao, qtdDeProdutosNaCalibracao);
  EEPROM.put(EPR_velocidade, velocidade);

  salvaContadorNaEEPROM();

  saveProdutoToEEPROM(produto);

  EEPROM.commit();
}

void loadParametersFromEEPROM()
{
  EEPROM.get(EPR_contadorAbsoluto, contadorAbsoluto);
  EEPROM.get(EPR_produto, produto);
  EEPROM.get(EPR_qtdDeProdutosNaCalibracao, qtdDeProdutosNaCalibracao);
  EEPROM.get(EPR_margemDeSeguranca, margemDeSeguranca);
  EEPROM.get(EPR_sombra, sombra);
  EEPROM.get(EPR_velocidade, velocidade);
  // EEPROM.get(EPR_rampa, rampa);
  // EEPROM.get(EPR_velocidadeG0, velocidadeG0);
  // EEPROM.get(EPR_duracaoPulsoDeImpressao, duracaoPulsoDeImpressao);
  // EEPROM.get(EPR_limiteEixoX, limiteEixoX);
  // EEPROM.get(EPR_limiteEixoY, limiteEixoY);
  // EEPROM.get(EPR_habilitarIntertravamento1, habilitarIntertravamento1);
  // EEPROM.get(EPR_habilitarIntertravamento2, habilitarIntertravamento2);
  // EEPROM.get(EPR_habilitarReversao, habilitarReversao);
  // EEPROM.get(EPR_produtoAvancado, flag_produtoAvancado);
  // // EEPROM.get(EPR_linhasDeImpressao, linhasDeImpressao);
  // EEPROM.get(EPR_posicaoDeEmergencia, posicaoDeEmergencia);
  loadProdutoFromEEPROM();
}

void loadProdutoFromEEPROM()
{
  EEPROM.get(EPR_inicioProdutos + produto * EPR_sizeParameters + sizeof(atrasoBloqueio) * EPR_atrasoBloqueio, atrasoBloqueio);
  EEPROM.get(EPR_inicioProdutos + produto * EPR_sizeParameters + sizeof(duracaoBloqueio) * EPR_duracaoBloqueio, duracaoBloqueio);
  EEPROM.get(EPR_inicioProdutos + produto * EPR_sizeParameters + sizeof(fimZonaCegaDeEntrada) * EPR_fimZonaCegaDeEntrada, fimZonaCegaDeEntrada);
  EEPROM.get(EPR_inicioProdutos + produto * EPR_sizeParameters + sizeof(inicioZonaCegaDeSaida) * EPR_inicioZonaCegaDeSaida, inicioZonaCegaDeSaida);
  EEPROM.get(EPR_inicioProdutos + produto * EPR_sizeParameters + sizeof(tamanhoDoGarrafao) * EPR_tamanhoDoGarrafao, tamanhoDoGarrafao);
}

void saveProdutoToEEPROM(int32_t _produto)
{
  EEPROM.put(EPR_inicioProdutos + _produto * EPR_sizeParameters + sizeof(atrasoBloqueio) * EPR_atrasoBloqueio, atrasoBloqueio);
  EEPROM.put(EPR_inicioProdutos + _produto * EPR_sizeParameters + sizeof(duracaoBloqueio) * EPR_duracaoBloqueio, duracaoBloqueio);
  EEPROM.put(EPR_inicioProdutos + _produto * EPR_sizeParameters + sizeof(fimZonaCegaDeEntrada) * EPR_fimZonaCegaDeEntrada, fimZonaCegaDeEntrada);
  EEPROM.put(EPR_inicioProdutos + _produto * EPR_sizeParameters + sizeof(inicioZonaCegaDeSaida) * EPR_inicioZonaCegaDeSaida, inicioZonaCegaDeSaida);
  EEPROM.put(EPR_inicioProdutos + _produto * EPR_sizeParameters + sizeof(tamanhoDoGarrafao) * EPR_tamanhoDoGarrafao, tamanhoDoGarrafao);
}

void presetEEPROM()
{
  for (int i = 0; i < EPR_maxProdutos; i++)
  {
    saveProdutoToEEPROM(i);
  }
}

void salvaContadorNaEEPROM()
{
  const uint16_t intervaloEntreBackups = 100; // ciclos
  if ((contadorAbsoluto % intervaloEntreBackups) == 0)
  {
    // Serial.print("save contador: ");Serial.println(contadorAbsoluto);
    EEPROM.put(EPR_contadorAbsoluto, contadorAbsoluto);
  }
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

  menu_monitor.setMonitorMenus(&menu_contador, &menu_ok, &menu_nok, &menu_status);
  liberaMenusDaIhm();
  ihm.goToMenu(&menu_produto);

  //   atualizaTextoMenuImprimeEixoY();
  //   menu_testeImpressao.setMsgDefault("PRESSIONE CIMA");

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
        if (checkMenu == &menu_produto)
        {
          // saveProdutoToEEPROM(produto); // to do: salvar produto na eeprom antes de trocar. Tem que ser feito antes de incrementar a variavel
          loadProdutoFromEEPROM();
        }
        else if (checkMenu == &menu_contador)
        {
          resetDosContadores();
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
        if (checkMenu == &menu_produto)
        {
          loadProdutoFromEEPROM();
        }
        else if (checkMenu == &menu_contador)
        {
          resetDosContadores();
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
        //   ihm.goToMenu(&menu_falhas);
        //   ihm.showStatus2msg("MANUTENCAO LIBERADA");
        //   flag_manutencao = true;
        // }
      }
    }
    else
    {
      Serial.print("erro> bt=");
      Serial.println(bt);
    }
  }
}

void resetDosContadores()
{
  contador = 0;
  ok = 0;
  nok = 0;
}

void liberaMenusDaIhm()
{
  ihm.addMenuToIndex(&menu_produto);
  ihm.addMenuToIndex(&menu_atrasoBloqueio);
  ihm.addMenuToIndex(&menu_duracaoBloqueio);
  ihm.addMenuToIndex(&menu_fimZonaCegaDeEntrada);
  ihm.addMenuToIndex(&menu_inicioZonaCegaDeSaida);
  ihm.addMenuToIndex(&menu_tamanhoDoGarrafao);
  ihm.addMenuToIndex(&menu_velocidade);
  ihm.addMenuToIndex(&menu_margemDeSeguranca);
  ihm.addMenuToIndex(&menu_qtdProdutosNaCalibracao);
  ihm.addMenuToIndex(&menu_sombra);
  ihm.addMenuToIndex(&menu_contador);
  ihm.addMenuToIndex(&menu_monitor);
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