#include "config.h"
#include "pictures.h"
#include "stdint.h"
#include <SPI.h>
#include <EEPROM.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS2X_lib.h>

/*
 * Прошивка для агробота прошедшая небольшое ревью, возможно некоторые моменты следовало бы запихнуть в классы,
 * но для одного cpp файла смысла от этого особо не вижу. Сделать большее количество cpp файлов данная среда разработки не представляет возможным. 
 * Были вынесены некоторые дефайны и константы в файлы config.h, fixed.h, pictures.h из-за большого их обилия и усложнения читабельности кода.
 * В файле config.h находятся вещи, которые можно/нужно менять в зависимости от версии или параметров робота.
 * В файле fixed.h находятся вещи, которые не надо трогать, они там только чтобы упростить читаемость кода.
 * В файле pictures.h находятся изображения выводимые на дисплей.
 * Также не вышло избавиться от всех глобальных переменных, но я старался. 
 * Исправлена путаница с названием некоторых моторов(теперь есть только мотор А и B, а не моторы 1 и 2 и выходы A и B).
 * Дефайны, константы и названия были отрефакторены и преведены к некоторому условному стандарту.
 * Функционал был разложен по идемпотентным(относительно) функциям, т.е. просто разложил задачи на более мелкие и структуризировал. 
 * Была немного пересмотрена логика опроса джойстика и управления - создан конечный автомат с состояниями в виде действий, которые 
 * должны происходить, когды будет нажата кнопка или др., и лидирующим состоянием(LEAD), в которое переходят все действия, после отработки,
 * в этом состоянии как раз и производится опрос кнопок. Такая система хорошо расширяется, изменяется и читается, проверено на кадете. 
 * И не надо бегать по коду искать в каком if производится опрос конкретной кнопки.
 * Такой же конечный автомат добавлен и для режима калибровки.
 * В режим калибровки добавлен вывод на дисплей текущего состояния сервы - максимального и минимального положения, а также вывод текущего значения.
 * Выбор серв в режиме калибровки закольцован. 
 * Произведена оптимизация - в рабочем режиме ограничен вывод на дисплей повторных состояний, это многократно увеличивает скорость работы кода, но 
 * уменьшает читаемость и увеличивает размер кода, что является необходимым
 */

volatile Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); //инициализация i2c для pca с адресом 0x40
volatile Adafruit_SSD1306 display(DISPLAY_RESET_CH); // инициализация дисплея
volatile PS2X ps2x;  // cоздание экземпляра класса для джойстика

// глобальное зло
uint8_t motorSpeed = SPEED_MIN;   // скорость мотора, текущая
uint16_t servoPlantMin = SERVO_CENTRAL_POSITION;   // минимальное положение диспенсора    
uint16_t servoPlantMax = SERVO_CENTRAL_POSITION;   // максимальное
uint16_t servoPlowMin = SERVO_CENTRAL_POSITION;    // минимальное полпжение плуга
uint16_t servoPlowMax = SERVO_CENTRAL_POSITION;    // максимальное
uint16_t servoBucketMin = SERVO_CENTRAL_POSITION;  // минимальное положение ковша
uint16_t servoBucketMax = SERVO_CENTRAL_POSITION;  // максимальное
uint16_t servoBucketGrabMin = SERVO_CENTRAL_POSITION;  // минимальное положение схвата ковша
uint16_t servoBucketGrabMax = SERVO_CENTRAL_POSITION;  // максимальное

uint32_t standIdleTimer; // таймер отсчета времени бездействия

typedef enum
{
  EYE_UP,
  EYE_DOWN,
  EYE_LEFT,
  EYE_RIGHT,
  EYE_TIRED,
  EYE_DIFFICULT,
  //EYE_CUTE,   // эти два режима не используются, но если их использовать, они жрут много памяти
  EYE_WOW,
  CLEAR
} workDisplayState;   // перечисление состояния экрана в рабочем режиме (все ради оптимизации)

int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

// сами ф-ии особо не смотрел

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
}


void servoCentering() // центрирование серв
{
  pwm.setPWM(SERVO_PLANT_CH, 0, SERVO_CENTRAL_POSITION);
  pwm.setPWM(SERVO_PLOW_CH, 0, SERVO_CENTRAL_POSITION);
  pwm.setPWM(SERVO_BUCKET_CH, 0, SERVO_CENTRAL_POSITION);
  pwm.setPWM(SERVO_BUCKET_GRAB_CH, 0, SERVO_CENTRAL_POSITION);
}


void readServoRange() // читает значения крайних положений серв из епрома и записывает их в глобальные переменные
{
  //Значения границ серв считываются из энергонезависимой памяти
  EEPROM.get(EEPROM_ADDR_SERV_PLANT_MAX, servoPlantMax);
  EEPROM.get(EEPROM_ADDR_SERV_PLANT_MIN, servoPlantMin);

  EEPROM.get(EEPROM_ADDR_SERV_PLOW_MAX, servoPlowMax);
  EEPROM.get(EEPROM_ADDR_SERV_PLOW_MIN, servoPlowMin);

  EEPROM.get(EEPROM_ADDR_SERV_BUCKET_MAX, servoBucketMax);
  EEPROM.get(EEPROM_ADDR_SERV_BUCKET_MIN, servoBucketMin);

  EEPROM.get(EEPROM_ADDR_SERV_BUCKET_GRAB_MAX, servoBucketGrabMax);
  EEPROM.get(EEPROM_ADDR_SERV_BUCKET_GRAB_MIN, servoBucketGrabMin);
}


void displayImage(const unsigned char* image)   // вывод на дисплей изображения
{
  display.clearDisplay();
  display.drawBitmap(0, 0,  image, IMAGE_WIDTH, IMAGE_HEIGHT, 1);
  display.display();
}


void servoCalibrateDisplay(char* servoName, uint16_t servoPosition)   // вывод на дисплей название сервы и ее текущую позицию
{
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println(servoName);
  display.setCursor(0, 16);
  display.println(servoPosition);
  display.display();  
}


void servoInfoDisplay(char* servoName, uint16_t servoPositionMin, uint16_t servoPositionMax)    // вывод на дисплей имя сервы и ее максимальное и минимальную позицию
{
  display.clearDisplay();
  display.setTextSize(1.5);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println(servoName);
  display.setCursor(0, 10);
  display.println("MIN: ");
  display.setCursor(30, 8);
  display.println(servoPositionMin);
  display.setCursor(0, 20);
  display.println("MAX: ");
  display.setCursor(30, 20);
  display.println(servoPositionMax);
  display.display();    
}


void setDisplayState(workDisplayState state) // изменение состояния дисплея (все ради оптимизации)
{
  static workDisplayState m_state = CLEAR;
  if(m_state == state) return;  // если состояние на дисплее такое же, то можно не перерисовывать
  m_state = state;
  switch(m_state)
  {
    case EYE_UP:
      displayImage(eyes_up);  
      break;

    case EYE_DOWN:
      displayImage(eyes_down);  
      break;

    case EYE_LEFT:
      displayImage(eyes_left);  
      break;

    case EYE_RIGHT:
      displayImage(eyes_right); 
      break;

    case EYE_TIRED:
      displayImage(eyes_tired); 
      break;

    case EYE_DIFFICULT:
      displayImage(eyes_difficult); 
      break;
/*
    case EYE_CUTE:      // эти два режима не используются, но если их использовать, они жрут много памяти
      displayImage(eyes_cute); 
      break;
*/
    case EYE_WOW:
      displayImage(eyes_wow); 
      break;
      
    case CLEAR:
      display.clearDisplay();
      display.display();
      break;
  }
}

uint16_t rerangeSpeed(int16_t mspeed)  // проверка и корректировка скорости (параметр знаковый, чтоб переполнения не было) 
{
  if (mspeed > SPEED_MAX) return SPEED_MAX;
  if (mspeed < SPEED_MIN) return SPEED_MIN;
  return mspeed;
}


//Запуск двигателей 
void setSpeedRight(int16_t mspeed)  // первый двигатель - А
{
  if (mspeed > 0)   // если заданная скорость больше нуля, то задаем Прямой ШИМ без инвертирования
  {
    analogWrite(MOTOR_PWM_A_CH, 255);
    digitalWrite(MOTOR_PWM_INVERSE_A_CH, LOW);
  }
  else    // если меньше, то инвертируем направление
  {
    digitalWrite(MOTOR_PWM_INVERSE_A_CH, HIGH);
    analogWrite(MOTOR_PWM_A_CH, 0);
  }
  analogWrite(MOTOR_ENABLE_A_CH, abs(mspeed));    
}


void setSpeedLeft(int16_t mspeed) // второй двигатель - B
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
  analogWrite(MOTOR_ENABLE_B_CH, abs(mspeed));   
}


void stopMotors()   // остановка двигателей
{
  analogWrite(MOTOR_PWM_A_CH, 0);
  digitalWrite(MOTOR_PWM_INVERSE_A_CH, LOW);
  digitalWrite(MOTOR_PWM_INVERSE_B_CH, LOW);
  analogWrite(MOTOR_PWM_B_CH, 0);
}


#if VERSION == 11
void beep(uint16_t ton, uint16_t tim) // для проигрывания тона
{
  tone(BUZZER_CH, ton, tim);
  delay(tim + 20);
}
#endif


#if VERSION == 10
void beep(uint8_t num, uint16_t tim)
{
  for (uint16_t i = 0; i < num; i++)
  {
    digitalWrite(BUZZER_CH, HIGH);
    delay(tim);
    digitalWrite(BUZZER_CH, LOW);
    delay(50);
  }
}
#endif


void beepAlarm() // мелодия предупреждения
{
#if VERSION == 11
  //мелодия
  beep(NOTE_G, 50);
  beep(NOTE_E, 50);
  beep(NOTE_C, 50);
  noTone(BUZZER_CH);
#endif
#if VERSION == 10
  beep(3, 100);
#endif
}


void plowUp()   // поднять плуг
{
  for (uint16_t plowPulseLen = servoPlowMin; plowPulseLen < servoPlowMax; plowPulseLen++)
  {
    pwm.setPWM(SERVO_PLOW_CH, 0, plowPulseLen);
    delay(SERVO_DELAY);
  }
}


void plowDown() // опустить плуг
{
  for (uint16_t plowPulseLen = servoPlowMax; plowPulseLen > servoPlowMin; plowPulseLen--)
  {
    pwm.setPWM(SERVO_PLOW_CH, 0, plowPulseLen);
    delay(SERVO_DELAY);
  }
}


void plantActivate()  // активировать диспенсер
{
  static uint16_t plantPulseLen = SERVO_CENTRAL_POSITION;
  Serial.println(freeRam());
  for (plantPulseLen = servoPlantMin; plantPulseLen < servoPlantMax; plantPulseLen++)
  {
    pwm.setPWM(SERVO_PLANT_CH, 0, plantPulseLen);
    Serial.println(freeRam());
    delay(SERVO_DELAY);
  }
  delay(PLANT_ACTIVE_DELAY);   
  for (plantPulseLen = servoPlantMax; plantPulseLen > servoPlantMin; plantPulseLen--)
  {
    pwm.setPWM(SERVO_PLANT_CH, 0, plantPulseLen);
    delay(SERVO_DELAY);
  }  
}


void adcDataCounter(float* voltage, float* current)   // вычисление значения напряжения питания и тока, запись в параметры
{
  static uint8_t adcCount = ADC_MAX_COUNT;  // ограничение по частоте считывания данных с АЦП
  static float m_voltage = 0;   // доп переменные для того, чтобы не ждать 14 вызовов ф-ии, если пропустили первый 
  static float m_current = 0;
  if (adcCount >= ADC_MAX_COUNT)    // каждые ADC_MAX_COUNT = 15 раз меняет значения,приходящие в параметрах
  {
    adcCount = 0;
    m_voltage = ADC_VOLT_DIV_CONST * analogRead(ADC_VOLTAGE_CH) * ADC_UAREF / ADC_MAX; //вычисление напряжения на выходе буффера
    m_current = analogRead(ADC_CURRENT_CH) * ADC_UAREF / ADC_MAX / ADC_CURR_CONST;
  }
  *voltage = m_voltage;
  *current = m_current;
  adcCount++;
}


bool calibrationFSM()   // режим калибровки, нетривиальный конечный автомат, где состояния управляются от одного лидера, состояния переключаются по нажатию кнопок джойстика 
{
  static int8_t servoCounter = 0;  // каретка, переключающаяся от сервы к серве (нужен знаковый) - текущая выбранная серва из массива
  static uint16_t servoCalibPos = SERVO_CENTRAL_POSITION;   // текущая позиция выбранной сервы
  static uint16_t tempInfoPositionMin = SERVO_CENTRAL_POSITION;   // доп переменные, для считывания информации из епрома о выбранной серве
  static uint16_t tempInfoPositionMax = SERVO_CENTRAL_POSITION;
  static enum   
  {
    LEAD,   // главный режим, отсюда идет переход ко всем остальным состояниям
    EEPROM_CLEAR,  // тут происходит очистка EEPROM
    SERVO_NEXT, // переход к следующей серве
    SERVO_PREV, // переход к предыдущей серве
    SERVO_CENTERING,  // центровка выбранной сервы
    SERVO_MOVE_UP,    // увеличение скважности ШИМа на серве
    SERVO_MOVE_DOWN,  // уменьшение скважности ШИМа на серве
    SERVO_FIND_MAX,   // находим максимальную скважность сервы, в одном из крайних положений
    SERVO_FIND_MIN,   // находим минимальную скважность, в одном из крайних положений
    EXIT   // выход из конечного автомата, переход к другому режиму
  } state;

  switch(state)
  {
    case LEAD:  // состояние которое переходит в другие состояния, если были нажаты какие-то кнопки
      if (ps2x.Button(PSB_L3) && ps2x.Button(PSB_R3) && ps2x.Button(PSB_R1) && ps2x.Button(PSB_L1)){ state = EEPROM_CLEAR; }  
      if (ps2x.ButtonPressed(PSB_R1)) { state = SERVO_NEXT; }
      if (ps2x.ButtonPressed(PSB_L1)) { state = SERVO_PREV; }
      if (ps2x.Button(PSB_L3) || ps2x.Button(PSB_R3)) { state = SERVO_CENTERING; }
      if (ps2x.Button(PSB_PAD_UP)) { state = SERVO_MOVE_UP; }
      if (ps2x.Button(PSB_PAD_DOWN)) { state = SERVO_MOVE_DOWN; }
      if (ps2x.ButtonPressed(PSB_TRIANGLE)) { state = SERVO_FIND_MAX; }
      if (ps2x.ButtonPressed(PSB_CROSS)) { state = SERVO_FIND_MIN; }
      if (ps2x.Button(PSB_L2) && ps2x.Button(PSB_R2)) { state = EXIT; }
      return false;

    case EEPROM_CLEAR:
      // тут будет очистка епрома
      state = LEAD;
      return false;

    case SERVO_NEXT:  // делаем кольцевой массив, можем ходить от серве к серве по замкнутому кругу
      servoCounter++;  
      if (servoCounter >= (strlen((const char*)SERVO_ITERATED))) servoCounter = 0;
      servoCalibPos = SERVO_CENTRAL_POSITION;   // центровка сервы при переключении
      pwm.setPWM(SERVO_ITERATED[servoCounter], 0, servoCalibPos);   // задаем ей центральное положение
      
      EEPROM.get(EEPROM_ADDR_SERV_MIN[servoCounter], tempInfoPositionMin);  // после выбора сервы считываем о ней информацию из епрома и выводим на дисплей
      EEPROM.get(EEPROM_ADDR_SERV_MAX[servoCounter], tempInfoPositionMax);     
      servoInfoDisplay((char*)SERVO_NAMES_ITERATED[servoCounter], tempInfoPositionMin, tempInfoPositionMax);
      state = LEAD;      
      return false;

    case SERVO_PREV:  // делаем кольцевой массив, можем ходить от серве к серве по замкнутому кругу
      servoCounter--;
      if (servoCounter < 0) servoCounter = strlen((const char*)SERVO_ITERATED) - 1; 
      servoCalibPos = SERVO_CENTRAL_POSITION;   // центровка сервы при переключении
      pwm.setPWM(SERVO_ITERATED[servoCounter], 0, servoCalibPos); // задаем ей центральное положение

      EEPROM.get(EEPROM_ADDR_SERV_MIN[servoCounter], tempInfoPositionMin);  // после выбора сервы считываем о ней информацию из епрома и выводим на дисплей
      EEPROM.get(EEPROM_ADDR_SERV_MAX[servoCounter], tempInfoPositionMax);     
      servoInfoDisplay((char*)SERVO_NAMES_ITERATED[servoCounter], tempInfoPositionMin, tempInfoPositionMax);
      state = LEAD;
      return false;

    case SERVO_CENTERING:   // центрирование выбранной сервы
      servoCalibPos = SERVO_CENTRAL_POSITION; // установка центрального значения для текущей сервы
      pwm.setPWM(SERVO_ITERATED[servoCounter], 0, servoCalibPos);
  
      EEPROM.get(EEPROM_ADDR_SERV_MIN[servoCounter], tempInfoPositionMin);    // после центрирования сервы считываем о ней информацию из епрома и выводим на дисплей
      EEPROM.get(EEPROM_ADDR_SERV_MAX[servoCounter], tempInfoPositionMax);     
      servoInfoDisplay((char*)SERVO_NAMES_ITERATED[servoCounter], tempInfoPositionMin, tempInfoPositionMax);
      state = LEAD;
      return false;

    case SERVO_MOVE_UP:  
      servoCalibPos++;    // увеличиваем положение сервы
      pwm.setPWM(SERVO_ITERATED[servoCounter], 0, servoCalibPos);   // задаем его
      delay(SERVO_CALIBRATE_DELAY); // делаем задержку
      servoCalibrateDisplay((char*)SERVO_NAMES_ITERATED[servoCounter], servoCalibPos);   // выводим имя сервы и текущее положение
      state = LEAD;
      return false;

    case SERVO_MOVE_DOWN:
      servoCalibPos--;    // уменьшаем положение сервы
      pwm.setPWM(SERVO_ITERATED[servoCounter], 0, servoCalibPos); // задаем его
      delay(SERVO_CALIBRATE_DELAY); // делаем задержку
      servoCalibrateDisplay((char*)SERVO_NAMES_ITERATED[servoCounter], servoCalibPos); // выводим имя сервы и текущее положение
      state = LEAD;
      return false;

    case SERVO_FIND_MAX:  // если найдено максимальное положение сервы
      EEPROM.put(EEPROM_ADDR_SERV_MAX[servoCounter], servoCalibPos);  // записываем его в епром по адресу из массива и сигналим
#if VERSION == 11
      beep(NOTE_C, 200);  
      noTone(BUZZER_CH);
#endif
#if VERSION == 10
      beep(1, 200);
#endif
      EEPROM.get(EEPROM_ADDR_SERV_MIN[servoCounter], tempInfoPositionMin);  // после записи, считываем информацию из епрома и выводим на дисплей
      EEPROM.get(EEPROM_ADDR_SERV_MAX[servoCounter], tempInfoPositionMax);     
      servoInfoDisplay((char*)SERVO_NAMES_ITERATED[servoCounter], tempInfoPositionMin, tempInfoPositionMax);
      state = LEAD;
      return false;

    case SERVO_FIND_MIN:  // если найдено максимальное положение сервы
      EEPROM.put(EEPROM_ADDR_SERV_MIN[servoCounter], servoCalibPos);  // записываем его в епром по адресу из массива и сигналим
#if VERSION == 11
      beep(NOTE_C, 200);
      noTone(BUZZER_CH);
#endif
#if VERSION == 10
      beep(1, 200);
#endif
      EEPROM.get(EEPROM_ADDR_SERV_MIN[servoCounter], tempInfoPositionMin);  // после записи, считываем информацию из епрома и выводим на дисплей
      EEPROM.get(EEPROM_ADDR_SERV_MAX[servoCounter], tempInfoPositionMax);     
      servoInfoDisplay((char*)SERVO_NAMES_ITERATED[servoCounter], tempInfoPositionMin, tempInfoPositionMax);
      state = LEAD;      
      return false;

    case EXIT:    // выход из режима калибровки
      readServoRange(); // чтение границ серв из епрома и запись в глобальные переменные
#if VERSION == 11
      beep(NOTE_C, 300);
      beep(NOTE_G, 300);
      beep(NOTE_B, 300);
      noTone(BUZZER_CH);
#endif
#if VERSION == 10
      beep(1, 500);
#endif
      servoCentering();   // центровка серв
      servoCounter = 0;   // сбрасываем каретку
      servoCalibPos = SERVO_CENTRAL_POSITION;   // сбрасываем положение
      display.clearDisplay();   // чистим дисплей
      display.display();  
      state = LEAD;  
      return true;
  }
}


bool workFSM()    // рабочий режим
{
  static bool isPlowDown = false;  // опущен ли плуг
  static uint16_t bucketPulseLen = SERVO_CENTRAL_POSITION;    // текущие положения серв ковша
  static uint16_t bucketGrabPulseLen = SERVO_CENTRAL_POSITION;  // схвата ковша
  static uint32_t lastBeepTime = 0; // время последнего пищания 
  static enum
  {
    LEAD,  // управляющий режим
    FORWARD,  // вперед
    BACKWARD, // назад 
    LEFT,   // влево
    RIGHT,  // вправо
    NOTHING, // остановка + ничего не делать
    SPEED_UP, // уменьшение скорости
    SPEED_DOWN,  // увеличение скорости
    PLOW_SWITCH,   // переключает состояние плуга
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
      if (ps2x.Button(PSB_PAD_UP)) { state = FORWARD; }
      if (ps2x.Button(PSB_PAD_DOWN)) { state = BACKWARD; }
      if (ps2x.Button(PSB_PAD_LEFT)) { state = LEFT; }
      if (ps2x.Button(PSB_PAD_RIGHT)) { state = RIGHT; }
      if (!((ps2x.Button(PSB_PAD_UP) || ps2x.Button(PSB_PAD_DOWN) ||    \
             ps2x.Button(PSB_PAD_LEFT) || ps2x.Button(PSB_PAD_RIGHT)))) { state = NOTHING; }
      if (ps2x.ButtonPressed(PSB_R1)) { state = SPEED_UP; }
      if (ps2x.ButtonPressed(PSB_R2)) { state = SPEED_DOWN; }
      if (ps2x.ButtonPressed(PSB_L2)) { state = PLOW_SWITCH; }
      if (ps2x.ButtonPressed(PSB_L1)) { state = PLANT_ACTIVATION; }
      if (ps2x.Button(PSB_TRIANGLE)) { state = BUCKET_UP; }
      if (ps2x.Button(PSB_CROSS)) { state = BUCKET_DOWN; }
      if (ps2x.Button(PSB_CIRCLE)) { state = BUCKET_GRAB_CLAMP; }
      if (ps2x.Button(PSB_SQUARE)) { state = BUCKET_GRAB_LOOSE; }
      if (ps2x.Button(PSB_L3) && ps2x.Button(PSB_R3)) { state = EXIT; }
      return false;

    case FORWARD:
      setSpeedRight(motorSpeed);    // задаем скорости бортам моторов
      setSpeedLeft(motorSpeed);
      setDisplayState(EYE_UP);  
      standIdleTimer = millis();  // запомнить время последнего действия
      state = LEAD;
      return false;

    case BACKWARD:
      setSpeedRight(-motorSpeed);
      setSpeedLeft(-motorSpeed);
      setDisplayState(EYE_DOWN);   
      standIdleTimer = millis();  // запомнить время последнего действия
      state = LEAD;
      return false;

    case LEFT:
      setSpeedRight(motorSpeed);
      setSpeedLeft(-motorSpeed);
      setDisplayState(EYE_LEFT);  
      standIdleTimer = millis();
      state = LEAD;
      return false;

    case RIGHT:
      setSpeedRight(-motorSpeed);
      setSpeedLeft(motorSpeed);
      setDisplayState(EYE_RIGHT);  
      standIdleTimer = millis();
      state = LEAD;
      return false;

    case NOTHING:
      stopMotors();
      // робот переходит в режим бездействия, 
      // нет доп проверок на действия, т.к. NOTHING идет в конечном автомате перед действиями и, если они происходят, то сюда не зайдет
      if(millis() - standIdleTimer > TIME_STAND_IDLE) // робот напоминает о режиме бездействия
      {
        if(millis() - lastBeepTime > TIME_BEEP_CYCLE)
        {
          lastBeepTime = millis();
          setDisplayState(EYE_WOW);
#if VERSION == 11
          beep(NOTE_G, 300);
          beep(NOTE_G, 150);
          beep(NOTE_F, 150);
          beep(NOTE_E, 150);
          beep(NOTE_A, 300);
          noTone(BUZZER_CH);
#endif
#if VERSION == 10
          beep(3, 100);
#endif  
        }  
      }      
      state = LEAD;      
      return false;

    case SPEED_UP:
      motorSpeed = rerangeSpeed(motorSpeed + SPEED_STEP);   // переоценка скоростей, если они выходят из диапазона 
      standIdleTimer = millis();
      state = LEAD;
      return false;

    case SPEED_DOWN:
      motorSpeed = rerangeSpeed(motorSpeed - SPEED_STEP);   // переоценка скоростей, если они выходят из диапазона 
      standIdleTimer = millis();
      state = LEAD;
      return false;

    case PLOW_SWITCH:
      if (isPlowDown) 
      {
        plowUp();   // поднимаем плуг
        isPlowDown = false;
      }
      else
      {
        plowDown();    // опускаем плуг
        isPlowDown = true;
      }
      setDisplayState(EYE_DIFFICULT);    
      standIdleTimer = millis();
      state = LEAD;
      return false;

     case PLANT_ACTIVATION:
      setDisplayState(EYE_DIFFICULT); 
      plantActivate();
      standIdleTimer = millis();
      Serial.println(freeRam());
      state = LEAD;
      return false;

    case BUCKET_UP:
      bucketPulseLen = bucketPulseLen - SERVO_STEP;
      if (bucketPulseLen > servoBucketMax) bucketPulseLen = servoBucketMax;
      if (bucketPulseLen < servoBucketMin) bucketPulseLen = servoBucketMin;
      pwm.setPWM(SERVO_BUCKET_CH, 0, bucketPulseLen);
      setDisplayState(EYE_DIFFICULT);  
      standIdleTimer = millis();
      state = LEAD;
      return false;

    case BUCKET_DOWN:
      bucketPulseLen = bucketPulseLen + SERVO_STEP;
      if (bucketPulseLen > servoBucketMax) bucketPulseLen = servoBucketMax;
      if (bucketPulseLen < servoBucketMin) bucketPulseLen = servoBucketMin;
      pwm.setPWM(SERVO_BUCKET_CH, 0, bucketPulseLen);
      setDisplayState(EYE_DIFFICULT);   
      standIdleTimer = millis();
      state = LEAD;
      return false;

    case BUCKET_GRAB_CLAMP:
      bucketGrabPulseLen = bucketGrabPulseLen + SERVO_STEP;
      if (bucketGrabPulseLen > servoBucketGrabMax) bucketGrabPulseLen = servoBucketGrabMax;
      if (bucketGrabPulseLen < servoBucketGrabMin) bucketGrabPulseLen = servoBucketGrabMin;
      pwm.setPWM(SERVO_BUCKET_GRAB_CH, 0, bucketGrabPulseLen);
      setDisplayState(EYE_DIFFICULT);  
      standIdleTimer = millis();
      state = LEAD;
      return false;

    case BUCKET_GRAB_LOOSE:
      bucketGrabPulseLen = bucketGrabPulseLen - SERVO_STEP;
      if (bucketGrabPulseLen > servoBucketGrabMax) bucketGrabPulseLen = servoBucketGrabMax;
      if (bucketGrabPulseLen < servoBucketGrabMin) bucketGrabPulseLen = servoBucketGrabMin;
      pwm.setPWM(SERVO_BUCKET_GRAB_CH, 0, bucketGrabPulseLen);
      setDisplayState(EYE_DIFFICULT);   
      standIdleTimer = millis();
      state = LEAD;
      return false;

    case EXIT:
      isPlowDown = false;   // сбрасываем метку о состоянии плуга
      bucketPulseLen = SERVO_CENTRAL_POSITION;
      bucketGrabPulseLen = SERVO_CENTRAL_POSITION;
      setDisplayState(CLEAR);  
#if VERSION == 11
      beep(NOTE_B, 300);
      beep(NOTE_G, 300);
      beep(NOTE_C, 300);      
      noTone(BUZZER_CH);
#endif
#if VERSION == 10
      beep(1, 500);
#endif
      standIdleTimer = millis();
      state = LEAD;
      return true;
  }
}



void setup() 
{
  Serial.begin(9600);
  Serial.println(freeRam());

  displaySetup(); // инициализация дисплея
  motorSetup();   // инициализация моторов
  servoSetup();   // инициализация серв
  servoCentering();   // центрирование серв
  
  pinMode(A4, INPUT_PULLUP);    // подтяжка линий I2C к питанию, мб и не надо 
  pinMode(A5, INPUT_PULLUP);

  //установка выводов и настроек: GamePad(clock, command, attention, data, Pressures?, Rumble?) проверка ошибок
  ps2x.config_gamepad(JOY_CLK_CH, JOY_CMD_CH, JOY_SEL_CH, JOY_DAT_CH, JOY_PRESSURES, JOY_RUMBLE); 

  readServoRange();   // чтение границ серв из епрома и запись в глобальные переменные

  //мелодия включения
#if VERSION == 11
  beep(NOTE_C, 400);
  beep(NOTE_E, 350);
  beep(NOTE_G, 150);
  beep(NOTE_B, 400);
  noTone(BUZZER_CH);
#endif
#if VERSION == 10
  beep(1, 500);
#endif

  analogReference(EXTERNAL);  // настройка опорного напряжения для АЦП: внешний источник на выводе AREF
    
  standIdleTimer = millis(); // запомнить время последнего действия
}



void loop()
{
  static bool m_exit = false; // доп переменная для хранения данных о выходе из некоторых конечных автоматов
  static float voltage = 0;
  static float current = 0;
  static uint32_t lastBeepTime = 0; // время последнего пищания об отсутствии заряда
  static enum 
  {
    WORK,   // рабочий режим - ездит, кривляется
    CALIBRATION,  // режим калибровки
    TIRED,  // если робот устал - низкое напряжение
  } state;

  ps2x.read_gamepad(false, 0); // считывание данных с джойстика и установка скорости вибрации
  adcDataCounter(&voltage, &current); // обновляем донные с АЦП
  if (voltage < MIN_MCU_VOLTAGE) state = TIRED;   // если напряжение маленькое
  switch(state)
  {
    case WORK:
      m_exit = workFSM();   // крутимся в рабочем режиме, пока не придет флаг о выходе из него - вернется true
      if(m_exit)  state = CALIBRATION;
      m_exit = false;
      break;

    case CALIBRATION:
      m_exit = calibrationFSM();  // крутимся в режиме калибровки, пока не придет флаг о выходе из него - вернется true
      if(m_exit)  state = WORK;
      m_exit = false;
      standIdleTimer = millis();  // запомнить время последнего действия
      break;    

    case TIRED:
      if (voltage > MIN_MCU_VOLTAGE)
      {
        state = WORK;  // если напряжение нормальное
      }
      else
      {
        setDisplayState(EYE_TIRED);  
        if((millis() - lastBeepTime) > TIME_BEEP_CYCLE) // если прошло достаточно времени
        {        
          lastBeepTime = millis();
#ifdef VERSION == 11
          beep(NOTE_B, 400);
          beep(NOTE_G, 350);
          beep(NOTE_E, 150);
          beep(NOTE_C, 400);
          noTone(BUZZER_CH);
#endif
#ifdef VERSION == 10
          beep(2, 100);
#endif
        }
      }
      delay(10);
      break;
  }
  delay(15);
}

