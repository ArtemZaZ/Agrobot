#include "config.h"
#include "pictures.h"
#include "stdint.h"
/*#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS2X_lib.h>

#define SERVO_MAX_CH  7 //самый большой по счёту занятый канал
#define SERVO_MIN_CH  4 //самый меньший по счёту занятый канал

#define DSERVO_const 5 //шаг изменения положения сервы
#define SERVO_DELAY 3 //задержка для правильной работы

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2
*/


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); //инициализация i2c для pca с адресом 0x40
Adafruit_SSD1306 display(OLED_RESET); // инициализация дисплея
PS2X ps2x;  // cоздание экземпляра класса для джойстика


uint8_t motorSpeed = SPEED_MIN;   // скорость мотора, текущая
uint32_t servoPlantMin = SERVO_CENTRAL_POSITION;   // минимальное положение диспенсора
uint32_t servoPlantMax = SERVO_CENTRAL_POSITION;   // максимальное
uint32_t servoPlowMin = SERVO_CENTRAL_POSITION;    // минимальное полпжение плуга
uint32_t servoPlowMax = SERVO_CENTRAL_POSITION;    // максимальное
uint32_t servoBucketMin = SERVO_CENTRAL_POSITION;  // минимальное положение ковша
uint32_t servoBucketMax = SERVO_CENTRAL_POSITION;  // максимальное
uint32_t servoBucketGrabMin = SERVO_CENTRAL_POSITION;  // минимальное положение схвата ковша
uint32_t servoBucketGrabMax = SERVO_CENTRAL_POSITION;  // максимальное

uint32_t bucketPulseLen = SERVO_CENTRAL_POSITION;  // ???
uint32_t bucketGrabPulseLen = SERVO_CENTRAL_POSITION;
uint32_t plowPulseLen = SERVO_CENTRAL_POSITION;
uint32_t plantPulseLen = SERVO_CENTRAL_POSITION;

uint64_t standIdleTimer; // таймер отсчета времени бездействия

//float mcuVoltage;  // текущее напряжение
//float mcuVurrent;  // текущий ток

/*
unsigned long time_pause;
unsigned long time_standstill_long;

unsigned char flag = 0;
unsigned char state_plow = 0, outstr, servo_ch = 0, address_max, address_min;
unsigned int calibration = SERVO_CENTRAL;
         
int error = 0;
byte type = 0;
byte vibrate = 0;
*/


void motorSetup()   // инициализация моторов
{
  pinMode(MOTOR_ENABLE_A_CH, OUTPUT);
  pinMode(MOTOR_ENABLE_B_CH, OUTPUT);
  pinMode(MOTOR_PWM_A_CH, OUTPUT);
  pinMode(MOTOR_PWM_B_CH, OUTPUT);
  pinMode(MOTOR_PWM_INVERSE_A_CH, OUTPUT);
  pinMode(MOTOR_PWM_INVERSE_B_CH, OUTPUT);
}


void buzzerSetup()  // инициализация пищалки
{
  pinMode(BUZZER_CH, OUTPUT);
  noTone(BUZZER_CH);
}


void displaySetup() // инициализация дисплея
{
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // Инициализация I2C для дисплея с адресом 0x3D
  display.display();
  delay(2000); //задержка для инициализации дисплея
  display.clearDisplay(); // очистка дисплея
}


void servoSetup() // инициализация серв
{
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);  // Установка частоты ШИМ
  servoCentering();
}


void servoCentering() // центрирование серв
{
  pwm.setPWM(SERVO_PLANT_CH, 0, SERVO_CENTRAL);
  pwm.setPWM(SERVO_PLOW_CH, 0, SERVO_CENTRAL);
  pwm.setPWM(SERVO_BUCKET_CH, 0, SERVO_CENTRAL);
  pwm.setPWM(SERVO_BUCKETUD_CH, 0, SERVO_CENTRAL);
}


void printText(uint8_t* str, uint8_t textsize)  //Вывод строки на дисплей
{
  display.clearDisplay();
  display.setTextSize(textsize);
  display.setTextColor(WHITE);
  display.setCursor(0, 16);
  display.println(str);
  display.display();
}


//Запуск двигателей 
void setSpeedRight(int mspeed)  // первый двигатель - А
{
  if (mspeed > 0)   // если заданная скорость больше нуля, то задаем Прямой ШИМ без инвертирования
  {
    analogWrite(MOTOR_PWM_A_CH, 255);
    digitalWrite(MOTOR_PWM_INVERSE_A_CH, LOW);
  }
  else    // если меньше, то инвертируем ШИМ !!!( по идее еще бы и значения инвертировать нужно)
  {
    digitalWrite(MOTOR_PWM_INVERSE_A_CH, HIGH);
    analogWrite(MOTOR_PWM_A_CH, 0);
  }
  analogWrite(MOTOR_ENABLE_A_CH, abs(mspeed));    //!!! тут еще мб придется поиграть с заполнением, т.к. сейчас скорость в обратную сторону движения не будет совпадать с прямой
}


void SetSpeedLeft(int mspeed) // второй двигатель - B
{
  if (mspeed > 0)
  {
    digitalWrite(MOTOR_PWM_INVERSE_B_CH, HIGH);
    analogWrite(MOTOR_PWM_B_CH, 0);
  }
  else
  {
    analogWrite(MOTOR_PWM_B_CH, 255);
    digitalWrite(MOTOR_PWM_INVERSE_B_CH, LOW);
  }
  analogWrite(MOTOR_ENABLE_B_CH, abs(mspeed));    //!!! аналогично
}


void StopMotors()   // остановка двигателей
{
  analogWrite(MOTOR_PWM_A_CH, 0);
  digitalWrite(MOTOR_PWM_INVERSE_A_CH, LOW);
  digitalWrite(MOTOR_PWM_INVERSE_B_CH, LOW);
  analogWrite(MOTOR_PWM_B_CH, 0);
}


#ifdef VERSION == 11
void beep(uint32_t ton, uint32_t tim) // для проигрывания тона
{
  tone(BUZZER, ton, tim);
  delay(tim + 20);
}
#endif


#ifdef VERSION == 10
void beep(uint8_t num, uint32_t tim)
{
  for (uint32_t i = 0; i < num; i++)
  {
    digitalWrite(BUZZER, HIGH);
    delay(tim);
    digitalWrite(BUZZER, LOW);
    delay(50);
  }
}
#endif


void beepAlarm() // мелодия предупреждения
{
#ifdef VERSION == 11
  //мелодия
  beep(note_g, 50);
  beep(note_e, 50);
  beep(note_c, 50);
  noTone(BUZZER);
#endif
#ifdef VERSION == 10
  beep(3, 100);
#endif
}


void adcDataCounter(float* voltage, float* current)   // вычисление значения напряжения питания и тока, запись в параметры
{
  static uint8_t adcCount = ADC_MAX_COUNT;  // ограничение по частоте считывания данных с АЦП
  if (adcCount >= ADC_MAX_COUNT)
  {
    adcCount = 0;
    *mcu_voltage = ADC_VOLT_DIV_CONST * analogRead(ADC_VOLTAGE_CH) * ADC_UAREF / ADC_MAX; //вычисление напряжения на выходе буффера
    *mcu_current = analogRead(ADC_CURRENT_CH) * ADC_UAREF / ADC_MAX / ADC_CURR_CONST;
  }
  adcCount++;
}


bool calibrationFSM()   // режим калибровки, нетривиальный конечный автомат, где состояния управляются от одного лидера, состояния переключаются по нажатию кнопок джойстика 
{
  static enum   
  {
    LEAD,   // главный режим, отсюда идет переход ко всем остальным состояниям
    EEPROM_CLEAR,  // тут происходит очистка EEPROM
    NEXT_SERVO, // переход к следующей серве
    PREV_SERVO, // переход к предыдущей серве
    SERVO_MOVE_UP,  // увеличение скважности ШИМа на серве
    SERVO_MOVE_DOWN,  // уменьшение скважности ШИМа на серве
    SERVO_FIND_MAX,   // находим максимальную скважность сервы, в одном из крайних положений
    SERVO_FIND_MIN,   // находим минимальную скважность, в одном из крайних положений
    EXIT   // выход из конечного автомата, переход к другому режиму
  } state;

  switch(state)
  {
    case LEAD:

      return;

    case EEPROM_CLEAR:

      return;

    case NEXT_SERVO:

      return;

    case PREV_SERVO:

      return;

    case SERVO_MOVE_UP:

      return;

    case SERVO_MOVE_DOWN:

      return;

    case SERVO_FIND_MAX:

      return;

    case SERVO_FIND_MIN:
      
      return;

    case EXIT:
      return true;
  }
}


bool workFSM()    // рабочий режим
{
  static enum
  {
    LEAD,  // управляющий режим
    FORWARD,  // вперед
    BACKWARD, // назад 
    LEFT,   // влево
    RIGHT,  // вправо
    SPEED_UP, // уменьшение скорости
    SPEED_DOWN,  // увеличение скорости
    PLOW_SWITCH   // переключает состояние плуга
    PLANT_ACTIVATION, // активация диспенсора 
    BUCKET_UP,  // поднять ковш
    BUCKET_DOWN,  // опустить ковш
    BUCKET_GRAB_CLAMP,  // сжать схват 
    BUCKET_GRAB_LOOSE,  // ослабить схват
    EXIT  // переход к другому режиму 
  } state;

  switch(state)
  {
    case LEAD:

      return;

    case FORWARD:

      return;

    case BACKWARD:

      return;

    case LEFT:

      return;

    case RIGHT:

      return;

    case SPEED_UP:

      return;

    case SPEED_DOWN:

      return;

    case PLOW_SWITCH:
      
      return;

     case PLANT_ACTIVATION:

      return;

    case BUCKET_UP:

      return;

    case BUCKET_DOWN:

      return;

    case BUCKET_GRAB_CLAMP:

      return;

    case BUCKET_GRAB_LOOSE:

      return;

    case EXIT:
      return true;
  }
}


void setup() {
  
  pinMode(A4, INPUT_PULLUP);    // подтяжка линий I2C к питанию, мб и не надо 
  pinMode(A5, INPUT_PULLUP);

  //установка выводов и настроек: GamePad(clock, command, attention, data, Pressures?, Rumble?) проверка ошибок
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble); // что выдает error ????

  //Значения границ серв считываются из энергонезависимой памяти
  EEPROM.get(EEPROM_ADDR_SERV_PLANT_MAX, servoPlantMax);
  EEPROM.get(EEPROM_ADDR_SERV_PLANT_MIN, servoPlantMin);

  EEPROM.get(EEPROM_ADDR_SERV_PLOW_MAX, servoPlowMax);
  EEPROM.get(EEPROM_ADDR_SERV_PLOW_MIN, servoPlowMin);

  EEPROM.get(EEPROM_ADDR_SERV_BUCKET_MAX, servoBucketMax);
  EEPROM.get(EEPROM_ADDR_SERV_BUCKET_MIN, servoBucketMin);

  EEPROM.get(EEPROM_ADDR_SERV_BUCKET_GRAB_MAX, servoBucketGrabMax);
  EEPROM.get(EEPROM_ADDR_SERV_BUCKET_GRAB_MIN, servoBucketGrabMin);

#if VERSION == 11
  //мелодия включения
  beep(note_c, 400);
  beep(note_e, 350);
  beep(note_g, 150);
  beep(note_b, 400);
  noTone(BUZZER);
#endif

#if VERSION == 10
  beep(1, 500);
#endif

  analogReference(EXTERNAL);  // настройка опорного напряжения для АЦП: внешний источник на выводе AREF

  servoSetup();

  standIdleTimer = millis(); // запомнить время последнего действия
}

void loop()
{
  static bool m_exit = false;
  static enum 
  {
    WORK,
    CALIBRATION  
  } state;
  
  switch(state)
  {
    case WORK:
      m_exit = workFSM();
      if(m_exit)  state = CALIBRATION
      m_exit = false;
      return;

    case CALIBRATION:
      m_exit = calibrationFSM();
      if(m_exit)  state = WORK;
      m_exit = false;
      return;    
  }
}

/*
void loop()
{
  // опрос джойстика
  ps2x.read_gamepad(false, vibrate); // считывание данных с джойстика и установка скорости вибрации

  else
  {
    //РЕЖИМ КАЛИБРОВКИ
    if (robo_state == state_calibration)  
    {
      //Запрос на очистку EEPROM
      if (ps2x.Button(PSB_L3) & ps2x.Button(PSB_R3) & ps2x.Button(PSB_R1) & ps2x.Button(PSB_L1))
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

#ifdef version_1_1
          beep(note_g, 300);
          noTone(BUZZER);
#endif
#ifdef version_1_0
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

#ifdef version_1_1
        beep(note_c, 200);
        noTone(BUZZER);
#endif
#ifdef version_1_0
        beep(1, 200);
#endif
      }

      if (ps2x.ButtonPressed(PSB_CROSS)) //найдено минимальное положение сервы
      {
        EEPROM.put(address_min, calibration);


#ifdef version_1_1
        beep(note_b, 200);
        noTone(BUZZER);
#endif
#ifdef version_1_0
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

#ifdef version_1_1
        beep(note_c, 300);
        beep(note_g, 300);
        beep(note_b, 300);
        noTone(BUZZER);
#endif

#ifdef version_1_0
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

#ifdef version_1_1
        beep(note_f, 200);
        beep(note_f, 200);
        beep(note_c, 350);
        noTone(BUZZER);
#endif
#ifdef version_1_0
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

#ifdef version_1_1
              //мелодия
              beep(note_g, 300);
              beep(note_g, 150);
              beep(note_f, 150);
              beep(note_e, 150);
              beep(note_a, 300);
              noTone(BUZZER);
#endif
#ifdef version_1_0
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


#ifdef version_1_1
          beep(note_f, 50);
          beep(note_a, 50);
          noTone(BUZZER);
#endif
#ifdef version_1_0
          beep(2, 50);
#endif
        }
        else
        {
          motorspeed = SPEED_MAX;

#ifdef version_1_1
          beep(note_a, 100);
          beep(note_a, 50);
          beep(note_a, 100);
          noTone(BUZZER);
#endif
#ifdef version_1_0
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


#ifdef version_1_1
          beep(note_a, 50);
          beep(note_f, 50);
          noTone(BUZZER);
#endif
#ifdef version_1_0
          beep(1, 50);
#endif
        }
        else
        {
          motorspeed = SPEED_MIN;

#ifdef version_1_1
          beep(note_f, 100);
          beep(note_f, 50);
          beep(note_f, 100);
          noTone(BUZZER);
#endif
#ifdef version_1_0
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



}
*/
