#ifndef defines_h
#define defines_h

// EEPROM:
#define EEPROM_SIZE 512

#define EPR_contadorAbsoluto 0
#define EPR_atrasoProduto 4
#define EPR_duracaoPistao 8

#define EPR_atrasoNovoProduto 12
#define EPR_velocidade 16
#define EPR_atrasoSaida 20
#define EPR_atrasoPistao 24
#define EPR_rampa 28

// IO's:
#define PIN_SP PIN_HSDI1  // sensor de produto
#define PIN_START PIN_DI1
#define PIN_STOP PIN_DI2
#define PIN_EMERGENCIA DI3

#define PIN_PISTAO PIN_DO1
#define PIN_MOTOR PIN_HSDO1

#endif