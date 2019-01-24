#pragma once 
#include "stdint.h"
/* Файл фиксированных параметров */

// для работы с EEPROM (адреса положений серв, записываемых в EEPROM)
#define EEPROM_ADDR_SERV_PLANT_MIN    0   // идем с нуля, дальше прибавляем адреса
#define EEPROM_ADDR_SERV_PLANT_MAX    EEPROM_ADDR_SERV_PLANT_MIN + sizeof(uint32_t)

#define EEPROM_ADDR_SERV_PLOW_MIN     EEPROM_ADDR_SERV_PLANT_MAX + sizeof(uint32_t)
#define EEPROM_ADDR_SERV_PLOW_MAX     EEPROM_ADDR_SERV_PLOW_MIN + sizeof(uint32_t)

#define EEPROM_ADDR_SERV_BUCKET_GRAB_MIN  EEPROM_ADDR_SERV_PLOW_MAX + sizeof(uint32_t)
#define EEPROM_ADDR_SERV_BUCKET_GRAB_MAX  EEPROM_ADDR_SERV_BUCKET_GRAB_MIN + sizeof(uint32_t)

#define EEPROM_ADDR_SERV_BUCKET_MIN   EEPROM_ADDR_SERV_BUCKET_GRAB_MAX + sizeof(uint32_t)
#define EEPROM_ADDR_SERV_BUCKET_MAX   EEPROM_ADDR_SERV_BUCKET_MIN + sizeof(uint32_t)


#define SERVO_CENTRAL_POSITION  350  // центральное положение серв (1500 мкс)
#define SERVO_FREQ  60  // частота ШИМ (~57Гц)
#define SERVO_CALIBRATE_DELAY 1   // зедержка сервы при калибровке - влияет на скорость при калибровке
#define SERVO_DELAY 3 // задержка сервы при движении - влияет на скорость в рабочем режиме

// выводы драйвера моторов
#define MOTOR_ENABLE_A_CH     10  // пин разрешающий работу мотора A
#define MOTOR_ENABLE_B_CH     9   // пин разрешающий работу мотора B
#define MOTOR_PWM_A_CH        A2  // канал ШИМа мотора А
#define MOTOR_PWM_B_CH        A3  // канал ШИМа мотора B
#define MOTOR_PWM_INVERSE_A_CH   13   // пин инвертирующий ШИМ на канале мотора А
#define MOTOR_PWM_INVERSE_B_CH   4  // пин инвертирующий ШИМ на канале мотора B

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
#define ADC_MAX_COUNT    15  // задержка преобразования АЦП

#define MAX_MCU_CURRENT 5 //максимальный ток, при превышении которого срабатывает защита (5А)
#define MIN_MCU_VOLTAGE 3.3
#define ADC_CURR_CONST 0.47

// Итерируемы объекты - каналы серв и их имена для режима калибровки - нуль-терминальные строки
const unsigned char SERVO_ITERATED[5] = {SERVO_PLANT_CH, SERVO_PLOW_CH, SERVO_BUCKET_GRAB_CH, SERVO_BUCKET_CH, '\0'};
const char * SERVO_NAMES_ITERATED[5] = {"Plant", "Plow", "BucketGrab", "Bucket", '\0'};
const unsigned int EEPROM_ADDR_SERV_MIN[5] = {EEPROM_ADDR_SERV_PLANT_MIN, EEPROM_ADDR_SERV_PLOW_MIN,    // адреса максимальных позиций серв в епроме
                                              EEPROM_ADDR_SERV_BUCKET_GRAB_MIN, EEPROM_ADDR_SERV_BUCKET_MIN, '\0'};
const unsigned int EEPROM_ADDR_SERV_MAX[5] = {EEPROM_ADDR_SERV_PLANT_MAX, EEPROM_ADDR_SERV_PLOW_MAX,    // адреса максимальных позиций серв в епроме
                                              EEPROM_ADDR_SERV_BUCKET_GRAB_MAX, EEPROM_ADDR_SERV_BUCKET_MAX, '\0'};

