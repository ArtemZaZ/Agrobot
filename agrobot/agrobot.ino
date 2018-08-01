#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS2X_lib.h>

#include <pictures.h> //массивы изображений для дисплея

//версия платы (раскомментировать нужное)
/*#ifndef version_1.0
  #define version_1.0
  #endif*/

#ifndef version_1.1
#define version_1.1
#endif

//для работы с EEPROM
#define ADDRESS_SERVPLANT_MIN 0
#define ADDRESS_SERVPLANT_MAX ADDRESS_SERVPLANT_MIN + sizeof(int)

#define ADDRESS_SERVPLOW_MIN  ADDRESS_SERVPLANT_MAX + sizeof(int)
#define ADDRESS_SERVPLOW_MAX  ADDRESS_SERVPLOW_MIN + sizeof(int)

#define ADDRESS_SERVBUCKETUD_MIN  ADDRESS_SERVPLOW_MAX + sizeof(int)
#define ADDRESS_SERVBUCKETUD_MAX  ADDRESS_SERVBUCKETUD_MIN + sizeof(int)

#define ADDRESS_SERVBUCKET_MIN  ADDRESS_SERVBUCKETUD_MAX + sizeof(int)
#define ADDRESS_SERVBUCKET_MAX  ADDRESS_SERVBUCKET_MIN + sizeof(int)


//сервы
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); //инициализация i2c для pca с адресом 0x40

#define SERVO_CENTRAL 350  //центральное положение серв (1500 мкс)

//подключение серв (выводы pca)
#define SERVO_BUCKET_CH  7
#define SERVO_BUCKETUD_CH 6
#define SERVO_PLOW_CH  5
#define SERVO_PLANT_CH 4

#define SERVO_MAX_CH  7
#define SERVO_MIN_CH  4

#define DSERVO_const 5 //шаг изменения положения
#define SERVO_FREQ 60 //частота ШИМ (~57Гц)
#define SERVO_DELAY 3 //задержка для правильной работы

//Динамик
#define BUZZER 11

//Выводы драйвера
#define MOTOR_ENA   10
#define MOTOR_ENB   9
#define MOTOR_IN1A  A2
#define MOTOR_IN1B  13
#define MOTOR_IN2A  4
#define MOTOR_IN2B  A3

//Выводы джойстика
#define PS2_DAT        5
#define PS2_CMD        6
#define PS2_SEL        7
#define PS2_CLK        8

//Для дисплея
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2

//ноты
#ifdef version_1.1
#define note_c 261
#define note_d 294
#define note_e 329
#define note_f 349
#define note_g 391
#define note_a 440
#define note_b 466
#endif

//время в миллисекундах
#define TIME_STANDSTILL_MAX 30000 //через которое робот перейдёт в режим бездействия
#define TIME_STANDSTILLLONG_MAX 30000 //промежуток времени, через которое робот напоминает о бездействии
#define TIME_PAUSE_MAX  5000  //на ожидание подтверждения (режим калибровки) 

//Режимы работы джойстика (раскомментировать нужное)
//- pressures = аналоговое считывание нажатия кнопок
//- rumble    = вибромоторы
#define pressures   true
//#define pressures   false
//#define rumble      true
#define rumble      false

//параметры изображения на дисплее
#define imageWidth 128
#define imageHeight 64

//регулирование скорости
#define SPEED_MIN 90
#define SPEED_MAX 255
#define Dspeed_const 30 //константа приращения

//для АЦП
#define UAREF 5.0 //опорное напряжение
#define ADC_MAX 1024 //максимальная разрядность
//выводы считывания
#define ADC_PIN_VOLTAGE A1
#define ADC_PIN_CURRENT A0
#define DEL_CONST 1 //константа делителя напряжения
#define MAXCOUNT_ADC 15 //задержка преобразования АЦП
#define MAX_MCU_CURRENT 5 //максимальный ток (5А)
#define MIN_MCU_VOLTAGE 3.3
#define ADC_CURR_CONST 0.47

//Состояния робота
#define state_tired 0
#define state_go  1
#define state_goback  2
#define state_turnleft  3
#define state_turnright 4
#define state_servoaction  5
#define state_notmove 6
#define state_highcurrent 7
#define state_pause 8
#define state_calibration 9

int motorspeed = SPEED_MIN;
int count_ADC = MAXCOUNT_ADC;
unsigned long time_standstill, time_pause, time_standstill_long;
int pulselen_bucket = SERVO_CENTRAL;
int pulselen_bucketud = SERVO_CENTRAL;
int pulselen_plow, pulselen_plant;
float mcu_voltage, mcu_current;
unsigned char robo_state = state_notmove, flag = 0;
unsigned char state_plow = 0, outstr, servo_ch = 0, address_max, address_min;
unsigned int SERVO_BUCKET_MIN, SERVO_BUCKET_MAX, SERVO_BUCKETUD_MIN, SERVO_BUCKETUD_MAX, SERVO_PLOW_MIN,
         SERVO_PLOW_MAX, SERVO_PLANT_MIN, SERVO_PLANT_MAX, calibration = SERVO_CENTRAL;

//Создание класса для джойстика
PS2X ps2x;
int error = 0;
byte type = 0;
byte vibrate = 0;

void setup() {

  //Настройка выводов
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_ENB, OUTPUT);
  pinMode(MOTOR_IN1A, OUTPUT);
  pinMode(MOTOR_IN1B, OUTPUT);
  pinMode(MOTOR_IN2A, OUTPUT);
  pinMode(MOTOR_IN2B, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  noTone(BUZZER);

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);  // Установка частоты ШИМ

  //Инициализация дисплея
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // Инициализация I2C для дисплея с адресом 0x3D
  display.display();
  delay(2000); //задержка для инициализации дисплея
  display.clearDisplay(); // очистка дисплея

  //установка выводов и настроек: GamePad(clock, command, attention, data, Pressures?, Rumble?) проверка ошибок
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);

  //Значения границ серв считываются из энергонезависимой памяти
  EEPROM.get(ADDRESS_SERVPLANT_MAX, SERVO_PLANT_MAX);
  EEPROM.get(ADDRESS_SERVPLANT_MIN, SERVO_PLANT_MIN);

  EEPROM.get(ADDRESS_SERVPLOW_MAX, SERVO_PLOW_MAX);
  EEPROM.get(ADDRESS_SERVPLOW_MIN, SERVO_PLOW_MIN);

  EEPROM.get(ADDRESS_SERVBUCKET_MAX, SERVO_BUCKET_MAX);
  EEPROM.get(ADDRESS_SERVBUCKET_MIN, SERVO_BUCKET_MIN);

  EEPROM.get(ADDRESS_SERVBUCKETUD_MAX, SERVO_BUCKETUD_MAX);
  EEPROM.get(ADDRESS_SERVBUCKETUD_MIN, SERVO_BUCKETUD_MIN);

#ifdef version_1.1
  //мелодия включения
  beep(note_c, 400);
  beep(note_e, 350);
  beep(note_g, 150);
  beep(note_b, 400);
  noTone(BUZZER);
#endif

#ifdef version_1.0
  beep(1, 500);
#endif

  //Serial.begin(9600);

  //Настройка опорного напряжения для АЦП: внешний источник на выводе AREF
  analogReference(EXTERNAL);

  //Центровка серв
  ServCenter();

  time_standstill = millis(); //запомнить время последнего действия
}


void loop()
{
  //Опрос джойстика
  ps2x.read_gamepad(false, vibrate); //считывание данных с джойстика и установка скорости вибрации

  //Вычисление значения напряжения питания
  if (count_ADC == MAXCOUNT_ADC)
  {
    count_ADC = 0;
    mcu_voltage = DEL_CONST * analogRead(ADC_PIN_VOLTAGE) * UAREF / ADC_MAX; //вычисление напряжения на выходе буффера
    //dtostrf(mcu_voltage, 4, 2, outstr); //преобразование флоат в строку 4 символа в строке, 2 знака после запятой
    mcu_current = analogRead(ADC_PIN_CURRENT) * UAREF / ADC_MAX / ADC_CURR_CONST;
    /*Serial.println(mcu_voltage);
      Serial.println(mcu_current);*/
  }
  else count_ADC++;

  //меньше порога достаточного заряда аккумулятора
  if (mcu_voltage < MIN_MCU_VOLTAGE)
  {
    robo_state = state_tired;

#ifdef version_1.1
    beep(note_b, 400);
    beep(note_g, 350);
    beep(note_e, 150);
    beep(note_c, 400);
    noTone(BUZZER);
#endif

#ifdef version_1.0
    beep(2, 100);
#endif
  }

  //Если ток превышает максимальное значение
  if (mcu_current > MAX_MCU_CURRENT)
  {
    robo_state = state_highcurrent;
  }

  else
  {
    if (robo_state == state_calibration)  //РЕЖИМ КАЛИБРОВКИ
    {
      //Запрос на очистку EEPROM
      if (ps2x.Button(PSB_L3) & ps2x.Button(PSB_R3)& ps2x.Button(PSB_R1)& ps2x.Button(PSB_L1))
      {
        flag = 1;
        time_pause = millis();
      }

      //Очистка EEPROM
      if ((flag == 1) & ((millis() - time_pause) < TIME_PAUSE_MAX))
      {
        beep_not();
        if (ps2x.Button(PSB_SQUARE))
        {
          PrintText("EEPROM Clear", 3);
          for (unsigned int i = 0; i < 1024; i++)
          {
            EEPROM.update(i, 255);
          }
          delay(500);
          flag = 0;

#ifdef version_1.1
          beep(note_g, 300);
          noTone(BUZZER);
#endif
#ifdef version_1.0
          beep(1, 300);
#endif
        }
      }
      else flag = 0;

      if (ps2x.ButtonPressed(PSB_R1)) //переключение сервы
      {
        servo_ch++;
        servo_ch = constrain(servo_ch, SERVO_MIN_CH, SERVO_MAX_CH);
        calibration = SERVO_CENTRAL;
        pwm.setPWM(servo_ch, 0, calibration);
      }

      if (ps2x.ButtonPressed(PSB_L1)) //переключение сервы
      {
        servo_ch--;
        servo_ch = constrain(servo_ch, SERVO_MIN_CH, SERVO_MAX_CH);
        calibration = SERVO_CENTRAL;
        pwm.setPWM(servo_ch, 0, calibration);
      }

      if (ps2x.Button(PSB_L3) | ps2x.Button(PSB_R3)) //центровка выбранной сервы
      {
        calibration = SERVO_CENTRAL;
        pwm.setPWM(servo_ch, 0, calibration);
        delay(SERVO_DELAY);
      }


      if (ps2x.Button(PSB_PAD_DOWN))  //уменьшение скважности управляющего сигнала
      {
        calibration--;
        pwm.setPWM(servo_ch, 0, calibration);
        delay(SERVO_DELAY);
      }


      if (ps2x.Button(PSB_PAD_UP))  //увеличение скважности управляющего сигнала
      {
        calibration++;
        pwm.setPWM(servo_ch, 0, calibration);
        delay(SERVO_DELAY);
      }

      if (ps2x.ButtonPressed(PSB_TRIANGLE)) //найдено максимальное положение сервы
      {
        EEPROM.put(address_max, calibration);

#ifdef version_1.1
        beep(note_c, 200);
        noTone(BUZZER);
#endif
#ifdef version_1.0
        beep(1, 200);
#endif
      }

      if (ps2x.ButtonPressed(PSB_CROSS)) //найдено минимальное положение сервы
      {
        EEPROM.put(address_min, calibration);


#ifdef version_1.1
        beep(note_b, 200);
        noTone(BUZZER);
#endif
#ifdef version_1.0
        beep(1, 200);
#endif
      }

      //выход из режима калибровки
      if (ps2x.Button(PSB_L2) & ps2x.Button(PSB_R2))
      {
        robo_state = state_notmove;
        EEPROM.get(ADDRESS_SERVPLANT_MAX, SERVO_PLANT_MAX);
        EEPROM.get(ADDRESS_SERVPLANT_MIN, SERVO_PLANT_MIN);

        EEPROM.get(ADDRESS_SERVPLOW_MAX, SERVO_PLOW_MAX);
        EEPROM.get(ADDRESS_SERVPLOW_MIN, SERVO_PLOW_MIN);

        EEPROM.get(ADDRESS_SERVBUCKET_MAX, SERVO_BUCKET_MAX);
        EEPROM.get(ADDRESS_SERVBUCKET_MIN, SERVO_BUCKET_MIN);

        EEPROM.get(ADDRESS_SERVBUCKETUD_MAX, SERVO_BUCKETUD_MAX);
        EEPROM.get(ADDRESS_SERVBUCKETUD_MIN, SERVO_BUCKETUD_MIN);

#ifdef version_1.1
        beep(note_c, 300);
        beep(note_g, 300);
        beep(note_b, 300);
        noTone(BUZZER);
#endif

#ifdef version_1.0
        beep(1, 500);
#endif
        ServCenter();
        time_standstill = millis();
      }

      switch (servo_ch)
      {
        case 0:
          {
            PrintText("Calibration", 3);
            break;
          }
        case SERVO_PLANT_CH:
          {
            PrintText("PLANT", 3);
            address_max = ADDRESS_SERVPLANT_MAX;
            address_min = ADDRESS_SERVPLANT_MIN;
            break;
          }
        case SERVO_PLOW_CH:
          {
            PrintText("PLOW", 3);
            address_max = ADDRESS_SERVPLOW_MAX;
            address_min = ADDRESS_SERVPLOW_MIN;
            break;
          }
        case SERVO_BUCKETUD_CH:
          {
            PrintText("BUCKET UP/DOWN", 3);
            address_max = ADDRESS_SERVBUCKETUD_MAX;
            address_min = ADDRESS_SERVBUCKETUD_MIN;
            break;
          }
        case SERVO_BUCKET_CH:
          {
            PrintText("BUCKET", 3);
            address_max = ADDRESS_SERVBUCKET_MAX;
            address_min = ADDRESS_SERVBUCKET_MIN;
            break;
          }
      }
    }

    else
    {
      //РАБОЧИЙ РЕЖИМ
      //Вход в режим калибровки
      if (ps2x.Button(PSB_L3)&ps2x.Button(PSB_R3))
      {
        robo_state = state_calibration;
        servo_ch = 0;

#ifdef version_1.1
        beep(note_f, 200);
        beep(note_f, 200);
        beep(note_c, 350);
        noTone(BUZZER);
#endif
#ifdef version_1.0
        beep(2, 200);
        beep(1, 350);
#endif
        time_standstill = millis();
      }

      // ВВЕРХ нажато (движение вперёд)
      if (ps2x.Button(PSB_PAD_UP))
      {
        SetSpeedRight(motorspeed);
        SetSpeedLeft(motorspeed);
        robo_state = state_go;
        time_standstill = millis();
      }

      //ВНИЗ нажато (движение назад)
      if (ps2x.Button(PSB_PAD_DOWN))
      {
        robo_state = state_goback;

        SetSpeedRight(-motorspeed);
        SetSpeedLeft(-motorspeed);
        time_standstill = millis();
      }

      // Крестовина отпущена и не запущен режим калибровки
      if ((ps2x.Button(PSB_PAD_UP) == false) & (ps2x.Button(PSB_PAD_DOWN) == false) &
          (ps2x.Button(PSB_PAD_LEFT) == false) & (ps2x.Button(PSB_PAD_RIGHT) == false)
          & ((robo_state == state_calibration) == false))
      {

        StopMotors();
        if (millis() - time_standstill < TIME_STANDSTILL_MAX)
          robo_state = state_notmove;

        //не нажата ни одна кнопка действия
        if ((ps2x.Button(PSB_TRIANGLE) == false) & (ps2x.Button(PSB_CROSS) == false) &
            (ps2x.Button(PSB_CIRCLE) == false) & (ps2x.Button(PSB_SQUARE) == false))
        {

          if (millis() - time_standstill >= TIME_STANDSTILL_MAX)  //бездействие
          {
            robo_state = state_pause;

            if (millis() - time_standstill_long >= TIME_STANDSTILLLONG_MAX) //напоминание о бездействии
            {

#ifdef version_1.1
              //мелодия
              beep(note_g, 300);
              beep(note_g, 150);
              beep(note_f, 150);
              beep(note_e, 150);
              beep(note_a, 300);
              noTone(BUZZER);
#endif
#ifdef version_1.0
              beep(3, 100);
#endif
              time_standstill_long = millis();
            }
          }
        }
      }

      // ВПРАВО нажато (поворот)
      if (ps2x.Button(PSB_PAD_RIGHT))
      {
        robo_state = state_turnright;
        SetSpeedRight(-motorspeed);
        SetSpeedLeft(motorspeed);
        time_standstill = millis();
      }

      //ВЛЕВО нажато (поворот)
      if (ps2x.Button(PSB_PAD_LEFT))
      {
        robo_state = state_turnleft;
        SetSpeedRight(motorspeed);
        SetSpeedLeft(-motorspeed);
        time_standstill = millis();
      }

      //vibrate = ps2x.Analog(PSAB_CROSS);  //Скорость вибрации устанавливаеться в зависимости от силы нажатия кнопки (X)

      // L2 нажата (плуг)
      if (ps2x.ButtonPressed(PSB_L2))
      {
        time_standstill = millis();
        robo_state = state_servoaction;
        if ((SERVO_PLOW_MAX == 65535) | (SERVO_PLOW_MIN == 65535))
        {
          beep_not();
        }
        else
        {
          switch (state_plow)
          {
            case 0:
              {
                state_plow++;
                for (pulselen_plow = SERVO_PLOW_MIN; pulselen_plow < SERVO_PLOW_MAX; pulselen_plow ++)
                {
                  pwm.setPWM(SERVO_PLOW_CH, 0, pulselen_plow);
                  delay(SERVO_DELAY);
                }
                break;
              }
            case 1:
              {
                state_plow--;
                for (pulselen_plow = SERVO_PLOW_MAX; pulselen_plow > SERVO_PLOW_MIN; pulselen_plow --)
                {
                  pwm.setPWM(SERVO_PLOW_CH, 0, pulselen_plow);
                  delay(SERVO_DELAY);
                }
                break;
              }
          }
        }
      }

      //L1 (диспенсер)
      if (ps2x.ButtonPressed(PSB_L1))
      {
        time_standstill = millis();

        //проверка значений границ сервы диспенсера
        if ((SERVO_PLANT_MAX == 65535) | (SERVO_PLANT_MIN == 65535))
        {
          beep_not();
        }
        else
        {
          display.clearDisplay();
          display.drawBitmap(0, 0,  eyes_difficult, imageWidth, imageHeight, 1);
          display.display();

          for (pulselen_plant = SERVO_PLANT_MIN; pulselen_plant < SERVO_PLANT_MAX; pulselen_plant ++)
          {
            pwm.setPWM(SERVO_PLANT_CH, 0, pulselen_plant);
            delay(2);
          }
          delay(500);
          for (pulselen_plant = SERVO_PLANT_MAX; pulselen_plant > SERVO_PLANT_MIN; pulselen_plant --)
          {
            pwm.setPWM(SERVO_PLANT_CH, 0, pulselen_plant);
            delay(2);
          }
        }
      }

      // R1 нажата (увеличение скорости)
      if (ps2x.ButtonPressed(PSB_R1))
      {
        time_standstill = millis();
        if (motorspeed < (SPEED_MAX - Dspeed_const))
        {
          motorspeed = motorspeed + Dspeed_const;


#ifdef version_1.1
          beep(note_f, 50);
          beep(note_a, 50);
          noTone(BUZZER);
#endif
#ifdef version_1.0
          beep(2, 50);
#endif
        }
        else
        {
          motorspeed = SPEED_MAX;

#ifdef version_1.1
          beep(note_a, 100);
          beep(note_a, 50);
          beep(note_a, 100);
          noTone(BUZZER);
#endif
#ifdef version_1.0
          beep(1, 100);
          beep(1, 50);
          beep(1, 100);
#endif
        }
      }

      // R2 нажата (уменьшение скорости)
      if (ps2x.ButtonPressed(PSB_R2))
      {
        time_standstill = millis();
        if (motorspeed > (SPEED_MIN + Dspeed_const))
        {
          motorspeed = motorspeed - Dspeed_const;


#ifdef version_1.1
          beep(note_a, 50);
          beep(note_f, 50);
          noTone(BUZZER);
#endif
#ifdef version_1.0
          beep(1, 50);
#endif
        }
        else
        {
          motorspeed = SPEED_MIN;

#ifdef version_1.1
          beep(note_f, 100);
          beep(note_f, 50);
          beep(note_f, 100);
          noTone(BUZZER);
#endif
#ifdef version_1.0
          beep(1, 100);
          beep(1, 50);
          beep(1, 100);
#endif
        }
      }

      // Треугольник нажат (ковш вверх)
      if (ps2x.Button(PSB_TRIANGLE))
      {
        time_standstill = millis();
        if (SERVO_BUCKETUD_MIN == 65535)
        {
          beep_not();
        }
        else
        {
          if (pulselen_bucketud > (SERVO_BUCKETUD_MIN + DSERVO_const))
          {
            pulselen_bucketud = pulselen_bucketud - DSERVO_const;
            pwm.setPWM(SERVO_BUCKETUD_CH, 0, pulselen_bucketud);
          }
          else pulselen_bucketud = SERVO_BUCKETUD_MIN;
          robo_state = state_servoaction;
          delay(SERVO_DELAY);
        }
      }


      //Х нажат (ковш вниз)
      if (ps2x.Button(PSB_CROSS))
      {
        time_standstill = millis();
        if (SERVO_BUCKETUD_MAX == 65535)
        {
          beep_not();
        }
        else
        {
          if (pulselen_bucketud < (SERVO_BUCKETUD_MAX - DSERVO_const))
          {
            pulselen_bucketud = pulselen_bucketud + DSERVO_const;
            pwm.setPWM(SERVO_BUCKETUD_CH, 0, pulselen_bucketud);
          }
          else pulselen_bucketud = SERVO_BUCKETUD_MAX;

          robo_state = state_servoaction;
          delay(SERVO_DELAY);
        }
      }

      //Круг нажат (захват)
      if (ps2x.Button(PSB_CIRCLE))
      {
        time_standstill = millis();

        if (SERVO_BUCKET_MAX == 65535)
        {
          beep_not();
        }
        else
        {
          if (pulselen_bucket < (SERVO_BUCKET_MAX - DSERVO_const))
          {
            pulselen_bucket = pulselen_bucket + DSERVO_const;
            pwm.setPWM(SERVO_BUCKET_CH, 0, pulselen_bucket);
          }
          else pulselen_bucket = SERVO_BUCKET_MAX;
          robo_state = state_servoaction;
          delay(SERVO_DELAY);
        }
      }

      //Квадрат нажат (захват)
      if (ps2x.Button(PSB_SQUARE))
      {
        time_standstill = millis();
        if (SERVO_BUCKET_MIN == 65535)
        {
          beep_not();
        }
        else
        {
          if (pulselen_bucket > (SERVO_BUCKET_MIN + DSERVO_const))
          {
            pulselen_bucket = pulselen_bucket - DSERVO_const;
            pwm.setPWM(SERVO_BUCKET_CH, 0, pulselen_bucket);
          }
          else pulselen_bucket = SERVO_BUCKET_MIN;
          robo_state = state_servoaction;
          delay(SERVO_DELAY);
        }
      }



      //Проверка состояния робота
      switch (robo_state)
      {
        case state_tired: //недостаточный заряд
          {
            display.clearDisplay();
            display.drawBitmap(0, 0,  eyes_tired, imageWidth, imageHeight, 1);
            display.display();

            while (mcu_voltage < MIN_MCU_VOLTAGE)
              mcu_voltage = DEL_CONST * analogRead(ADC_PIN_VOLTAGE) * UAREF / ADC_MAX;
            robo_state = state_notmove;
            break;
          }
        case state_highcurrent: //превышение тока
          {
            display.clearDisplay();
            display.drawBitmap(0, 0,  eyes_tired, imageWidth, imageHeight, 1);
            display.display();

            StopMotors();

            pwm.setPWM(SERVO_PLANT_CH, 0, 0);
            pwm.setPWM(SERVO_PLOW_CH, 0, 0);
            pwm.setPWM(SERVO_BUCKET_CH, 0, 0);
            pwm.setPWM(SERVO_BUCKETUD_CH, 0, 0);

            while (mcu_current > MAX_MCU_CURRENT)
              mcu_current = analogRead(ADC_PIN_CURRENT) * UAREF / ADC_MAX / ADC_CURR_CONST;
            robo_state = state_notmove;
            break;
          }
        case state_go:  //движение вперёд
          {
            display.clearDisplay();
            display.drawBitmap(0, 0,  eyes_up, imageWidth, imageHeight, 1);
            display.display();
            break;
          }
        case state_goback:  //движение назад
          {
            display.clearDisplay();
            display.drawBitmap(0, 0,  eyes_down, imageWidth, imageHeight, 1);
            display.display();
            break;
          }
        case state_turnleft:  //поворот влево
          {
            display.clearDisplay();
            display.drawBitmap(0, 0,  eyes_left, imageWidth, imageHeight, 1);
            display.display();
            break;
          }
        case state_turnright: //поворот вправо
          {
            display.clearDisplay();
            display.drawBitmap(0, 0,  eyes_right, imageWidth, imageHeight, 1);
            display.display();
            break;
          }
        case state_servoaction: //действие механизмов
          {
            display.clearDisplay();
            display.drawBitmap(0, 0,  eyes_difficult, imageWidth, imageHeight, 1);
            display.display();
            break;
          }
        case state_pause: //длительное бездействие
          {
            display.clearDisplay();
            display.drawBitmap(0, 0,  eyes_wow, imageWidth, imageHeight, 1);
            display.display();
            break;
          }
        case state_notmove: //нет движения
          {
            display.clearDisplay();
            display.drawBitmap(0, 0,  eyes_cute, imageWidth, imageHeight, 1);
            display.display();
            break;
          }
      }
    }
  }
}


//Вывод слова на дисплей
void PrintText(char STR[255], char textsize)
{
  display.clearDisplay();
  display.setTextSize(textsize);
  display.setTextColor(WHITE);
  display.setCursor(0, 16);
  display.println(STR);
  display.display();

}

//Запуск двигателей
//Первый
void SetSpeedRight(int mspeed)
{
  if (mspeed > 0)
  {
    analogWrite(MOTOR_IN1A, 255);
    digitalWrite(MOTOR_IN1B, LOW);
  }
  else
  {
    digitalWrite(MOTOR_IN1B, HIGH);
    analogWrite(MOTOR_IN1A, 0);
  }
  analogWrite(MOTOR_ENA, abs(mspeed));
}

//Второй
void SetSpeedLeft(int mspeed)
{
  if (mspeed > 0)
  {
    digitalWrite(MOTOR_IN2A, HIGH);
    analogWrite(MOTOR_IN2B, 0);
  }
  else
  {
    analogWrite(MOTOR_IN2B, 255);
    digitalWrite(MOTOR_IN2A, LOW);
  }
  analogWrite(MOTOR_ENB, abs(mspeed));
}


//Остановка двигателей
void StopMotors()
{
  analogWrite(MOTOR_IN1A, 0);
  digitalWrite(MOTOR_IN1B, LOW);
  digitalWrite(MOTOR_IN2A, LOW);
  analogWrite(MOTOR_IN2B, 0);
}

//центовка серв
void ServCenter()
{
  pwm.setPWM(SERVO_PLANT_CH, 0, SERVO_CENTRAL);
  pwm.setPWM(SERVO_PLOW_CH, 0, SERVO_CENTRAL);
  pwm.setPWM(SERVO_BUCKET_CH, 0, SERVO_CENTRAL);
  pwm.setPWM(SERVO_BUCKETUD_CH, 0, SERVO_CENTRAL);

  pulselen_bucket = SERVO_CENTRAL;
  pulselen_bucketud = SERVO_CENTRAL;
}


#ifdef version_1.1
//для проигрывания тона
void beep(int ton, int tim)
{
  tone(BUZZER, ton, tim);
  delay(tim + 20);
}
#endif

#ifdef version_1.0
void beep(unsigned char num, unsigned int tim)
{
  for (unsigned char num_i = 0; num_i < num; num_i++)
  {
    digitalWrite(BUZZER, HIGH);
    delay(tim);
    digitalWrite(BUZZER, LOW);
    delay(50);
  }
}
#endif

void beep_not() //мелодия предупреждения
{
#ifdef version_1.1
  //мелодия
  beep(note_g, 50);
  beep(note_e, 50);
  beep(note_c, 50);
  noTone(BUZZER);
#endif
#ifdef version_1.0
  beep(3, 100);
#endif
}

