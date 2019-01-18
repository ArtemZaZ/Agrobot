#pragma once 
/* Файл фиксированных параметров */

// для работы с EEPROM (адреса положений серв, записываемых в EEPROM)
#define EEPROM_ADDR_SERV_PLANT_MIN    0   // идем с нуля, дальше прибавляем адреса
#define EEPROM_ADDR_SERV_PLANT_MAX    EEPROM_ADDR_SERV_PLANT_MIN + sizeof(int)

#define EEPROM_ADDR_SERV_PLOW_MIN     EEPROM_ADDR_SERV_PLANT_MAX + sizeof(int)
#define EEPROM_ADDR_SERV_PLOW_MAX     EEPROM_ADDR_SERV_PLOW_MIN + sizeof(int)

#define EEPROM_ADDR_SERV_BUCKET_GRAB_MIN  EEPROM_ADDR_SERV_PLOW_MAX + sizeof(int)
#define EEPROM_ADDR_SERV_BUCKET_GRAB_MAX  EEPROM_ADDR_SERV_BUCKET_GRAB_MIN + sizeof(int)

#define EEPROM_ADDR_SERV_BUCKET_MIN   EEPROM_ADDR_SERV_BUCKET_GRAB_MAX + sizeof(int)
#define EEPROM_ADDR_SERV_BUCKET_MAX   EEPROM_ADDR_SERV_BUCKET_MIN + sizeof(int)


#define SERVO_CENTRAL_POSITION  350  // центральное положение серв (1500 мкс)


// выводы джойстика
#define JOY_DAT_CH  5
#define JOY_CMD_CH  6
#define JOY_SEL_CH  7
#define JOY_CLK_CH  8

// вывод дисплея
#define DISPLAY_RESET_CH  4

// работа со звуком
#define BUZZER_CH 11  // вывод, подключённый к динамику

// ноты
#define NOTE_C 261
#define NOTE_D 294
#define NOTE_E 329
#define NOTE_F 349
#define NOTE_G 391
#define NOTE_A 440
#define NOTE_B 466


// работа с АЦП
#define ADC_VOLTAGE_CH  A1  // канал считывания напряжения
#define ADC_CURRENT_CH  A0  // канал считывания тока
#define ADC_UAREF   5.0     // опорное напряжение
#define ADC_MAX     1024    // максимальная разрядность
#define ADC_VOLT_DIV_CONST  1 // константа делителя напряжения
#define ADC_MAXCOUNT    15  // задержка преобразования АЦП

/*
#define MAX_MCU_CURRENT 5 //максимальный ток, при превышении которого срабатывает защита (5А)
#define MIN_MCU_VOLTAGE 3.3
#define ADC_CURR_CONST 0.47
*/
