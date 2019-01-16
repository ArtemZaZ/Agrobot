#pragma once
/* файл должен быть подключен в начале скетча */

// определение всех нужных параметров
#define VERSION 1 // версия 1 или 2

// регулирование скорости
#define SPEED_MIN   90  // наименьшее допустимое значение 90
#define SPEED_MAX   255 // наибольшее допустимое значение 255
#define SPEED_STEP  30  // размер приращения шага

// временные настройки
#define TIME_STAND_IDLE     30000 // время бездействия, после которого робот перейдет в режим бездействия 
#define TIME_STAND_IDLE_MAX 30000 // максимальное время бездействия, после которого робот перейдет в режим напоминания о бездействии

// режимы работы джойстика
#define JOY_PRESSURES true  // аналоговое считывание кнопок  
#define JOY_RUMBLE    false  // вибромотор

// подключение серв (выводы/каналы pca)
#define SERVO_BUCKET_CH   7 // канал сервы ковша
#define SERVO_BUCKET_GRAB_CH 6  // канал сервы схвата ковша
#define SERVO_PLOW_CH     5 // канал сервы плуга
#define SERVO_PLANT_CH    4 // канал сервы диспенсора




/* проверка параметров - при настройке не трогаем */
#ifndef VERSION // если не продефайнина версия
# error "VERSION is not defined"
#endif

#if (VERSION != 1) && (VERSION != 2)  // проверка на допустимые версии
# error "The selected version does not exist"
#endif

#if !(defined(SPEED_MIN) && defined(SPEED_MAX) && defined(SPEED_STEP))
# error "Speed parameters are not defined" 
#endif

#if !(defined(TIME_STAND_IDLE) && defined(TIME_STAND_IDLE_MAX))
# error "Time parameters are not defined"
#endif

#if !(defined(JOY_PRESSURES) && defined(JOY_RUMBLE))
# error "Joystick modes not defined" 
#endif

#if !(defined(SERVO_BUCKET_CH) && defined(SERVO_BUCKET_GRAB_CH) && defined(SERVO_PLOW_CH) && defined(SERVO_PLANT_CH))
# error "Servo channels not defined" 
#endif



