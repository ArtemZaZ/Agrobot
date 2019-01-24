#pragma once
/* Файл переменной конфигурации 
 * файл должен быть подключен в начале скетча 
 */

// определение всех нужных параметров
// определение версии, если что-то координально меняется, т.е. не может быть изменено при попощи
// данного файла - добавляйте новую версию, и в коде несовпадающие эл-ты помещайте в препроцессорные дериктивы ветвления версий
#define VERSION 11 // версия 1.0 - 10 или 1.1 - 11

// регулирование скорости
#define SPEED_MIN   90  // наименьшее допустимое значение 90
#define SPEED_MAX   255 // наибольшее допустимое значение 255
#define SPEED_STEP  30  // размер приращения шага

// временные настройки
#define TIME_STAND_IDLE     30000 // время бездействия, после которого робот перейдет в режим бездействия 
#define TIME_STAND_IDLE_MAX 30000 // максимальное время бездействия, после которого робот перейдет в режим напоминания о бездействии

// режимы работы джойстика
#define JOY_PRESSURES false  // аналоговое считывание кнопок  
#define JOY_RUMBLE    false  // вибромотор

// подключение серв (выводы/каналы pca)
#define SERVO_BUCKET_CH   6 // канал сервы ковша
#define SERVO_BUCKET_GRAB_CH 7  // канал сервы схвата ковша   //+
#define SERVO_PLOW_CH     5 // канал сервы плуга
#define SERVO_PLANT_CH    4 // канал сервы диспенсора

#define SERVO_STEP  5   // шаг изменения положения сервы при движении в рабочем режиме, влияет на скорость движения сервы

#define PLANT_ACTIVE_DELAY  500 // сколько времени диспенсер будет в активном положении

/* проверка параметров - при настройке не трогаем */
#ifndef VERSION // если не продефайнина версия
# error "VERSION is not defined"
#endif

#if (VERSION != 10) && (VERSION != 11)  // проверка на допустимые версии
# error "The selected version does not exist"
#endif

#if !(defined(SPEED_MIN) && defined(SPEED_MAX) && defined(SPEED_STEP))  // если не продефайнены значения скоростей
# error "Speed parameters are not defined" 
#endif

#if !(defined(TIME_STAND_IDLE) && defined(TIME_STAND_IDLE_MAX))   // если не продефайнены значения временных задержек
# error "Time parameters are not defined"
#endif

#if !(defined(JOY_PRESSURES) && defined(JOY_RUMBLE))  // если не продефайнены параметры джойстика
# error "Joystick modes not defined" 
#endif

#if !(defined(SERVO_BUCKET_CH) && defined(SERVO_BUCKET_GRAB_CH) && defined(SERVO_PLOW_CH) && defined(SERVO_PLANT_CH)) // если не продефайнены каналы серв
# error "Servo channels not defined" 
#endif

// подключаем фиксированные параметры, проверку не делаем, т.к. не меняем их
#include "fixed.h"


