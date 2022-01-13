#ifndef defines_h
#define defines_h

#define EEPROM_SIZE 512

#define EPR_contadorAbsoluto 0
#define EPR_produto 4
#define EPR_margemDeSeguranca 8
#define EPR_qtdDeProdutosNaCalibracao 12
#define EPR_sombra 16
#define EPR_velocidade 20

#define EPR_maxProdutos 15
#define EPR_inicioProdutos 60

#define EPR_sizeParameters 20

#define EPR_duracaoBloqueio 0
#define EPR_atrasoBloqueio 1
#define EPR_fimZonaCegaDeEntrada 2
#define EPR_inicioZonaCegaDeSaida 3
#define EPR_tamanhoDoGarrafao 4

// IO's:
#define PIN_SP PIN_HSDI1
#define PIN_START PIN_DI1
#define PIN_STOP PIN_DI2
#define PIN_EMERGENCIA DI3

#define PIN_PISTAO PIN_DO1
#define PIN_MOTOR PIN_HSDO1

// outros:
#define LIMITE_DE_PRODUTOS_NA_LINHA 4


#endif