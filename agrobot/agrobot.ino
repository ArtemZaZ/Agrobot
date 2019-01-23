#include "config.h"
#include "pictures.h"
#include "stdint.h"
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS2X_lib.h>

// TODO: везде запилить проверку границ

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); //инициализация i2c для pca с адресом 0x40
Adafruit_SSD1306 display(DISPLAY_RESET_CH); // инициализация дисплея
PS2X ps2x;  // cоздание экземпляра класса для джойстика


uint8_t motorSpeed = SPEED_MIN;   // скорость мотора, текущая
// пока глобальные мб потом сделаю локальные
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
uint32_t plantPulseLen = SERVO_CENTRAL_POSITION;

uint64_t standIdleTimer; // таймер отсчета времени бездействия

/// Ф-ии пока особо несмотрел, буду пересматривать после перестройки логики

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


void printText(uint8_t* str, uint8_t textsize)  //Вывод строки на дисплей
{
  display.clearDisplay();
  display.setTextSize(textsize);
  display.setTextColor(WHITE);
  display.setCursor(0, 16);
  display.println((char*)str);
  display.display();
}


void servoCalibrateDisplay(char* servoName, uint32_t servoPosition)
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


void servoInfoDisplay(char* servoName, uint32_t servoPositionMin, uint32_t servoPositionMax)
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


uint32_t rerangeSpeed(uint32_t mspeed)  // проверка и корректировка скорости
{
  if (mspeed > SPEED_MAX) return SPEED_MAX;
  if (mspeed < SPEED_MIN) return SPEED_MIN;
  return mspeed;
}


//Запуск двигателей 
void setSpeedRight(int32_t mspeed)  // первый двигатель - А
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


void setSpeedLeft(int32_t mspeed) // второй двигатель - B
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


void stopMotors()   // остановка двигателей
{
  analogWrite(MOTOR_PWM_A_CH, 0);
  digitalWrite(MOTOR_PWM_INVERSE_A_CH, LOW);
  digitalWrite(MOTOR_PWM_INVERSE_B_CH, LOW);
  analogWrite(MOTOR_PWM_B_CH, 0);
}


#if VERSION == 11
void beep(uint32_t ton, uint32_t tim) // для проигрывания тона
{
  tone(BUZZER_CH, ton, tim);
  delay(tim + 20);
}
#endif


#if VERSION == 10
void beep(uint8_t num, uint32_t tim)
{
  for (uint32_t i = 0; i < num; i++)
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


void debug()    // пищалка отладки
{
#if VERSION == 11
  beep(NOTE_G, 50);
  noTone(BUZZER_CH);
  delay(50);
#endif  
}


void plowUp()   // поднять плуг
{
  for (uint32_t plowPulseLen = servoPlowMin; plowPulseLen < servoPlowMax; plowPulseLen++)
  {
    pwm.setPWM(SERVO_PLOW_CH, 0, plowPulseLen);
    delay(SERVO_DELAY);
  }
}


void plowDown() // опустить плуг
{
  for (uint32_t plowPulseLen = servoPlowMax; plowPulseLen > servoPlowMin; plowPulseLen--)
  {
    pwm.setPWM(SERVO_PLOW_CH, 0, plowPulseLen);
    delay(SERVO_DELAY);
  }
}


void plantActivate()  // активировать диспенсер
{
  static uint32_t plantPulseLen = SERVO_CENTRAL_POSITION;
  for (plantPulseLen = servoPlantMin; plantPulseLen < servoPlantMax; plantPulseLen++)
  {
    pwm.setPWM(SERVO_PLANT_CH, 0, plantPulseLen);
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
  if (adcCount >= ADC_MAX_COUNT)
  {
    adcCount = 0;
    *voltage = ADC_VOLT_DIV_CONST * analogRead(ADC_VOLTAGE_CH) * ADC_UAREF / ADC_MAX; //вычисление напряжения на выходе буффера
    *current = analogRead(ADC_CURRENT_CH) * ADC_UAREF / ADC_MAX / ADC_CURR_CONST;
  }
  adcCount++;
}


bool calibrationFSM()   // режим калибровки, нетривиальный конечный автомат, где состояния управляются от одного лидера, состояния переключаются по нажатию кнопок джойстика 
{
  static int8_t servoCounter = 0;  // каретка, переключающаяся от сервы к серве (нужен знаковый)
  static uint32_t servoCalibPos = SERVO_CENTRAL_POSITION;
  static uint32_t tempInfoPositionMin = SERVO_CENTRAL_POSITION;
  static uint32_t tempInfoPositionMax = SERVO_CENTRAL_POSITION;
  static enum   
  {
    LEAD,   // главный режим, отсюда идет переход ко всем остальным состояниям
    EEPROM_CLEAR,  // тут происходит очистка EEPROM
    SERVO_NEXT, // переход к следующей серве
    SERVO_PREV, // переход к предыдущей серве
    SERVO_CENTERING,  // центровка выбранной сервы
    SERVO_MOVE_UP,  // увеличение скважности ШИМа на серве
    SERVO_MOVE_DOWN,  // уменьшение скважности ШИМа на серве
    SERVO_FIND_MAX,   // находим максимальную скважность сервы, в одном из крайних положений
    SERVO_FIND_MIN,   // находим минимальную скважность, в одном из крайних положений
    EXIT   // выход из конечного автомата, переход к другому режиму
  } state;

  switch(state)
  {
    case LEAD:
      if (ps2x.Button(PSB_L3) && ps2x.Button(PSB_R3) && ps2x.Button(PSB_R1) && ps2x.Button(PSB_L1)){ state = EEPROM_CLEAR; }  // очистка епрома 
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

    case SERVO_NEXT:  // делаем кольцевой массив
      servoCounter++;  
      if (servoCounter >= (strlen((const char*)SERVO_ITERATED))) servoCounter = 0;
      servoCalibPos = SERVO_CENTRAL_POSITION;   // центровка сервы при переключении
      pwm.setPWM(SERVO_ITERATED[servoCounter], 0, servoCalibPos);
      
      EEPROM.get(EEPROM_ADDR_SERV_MIN[servoCounter], tempInfoPositionMin);
      EEPROM.get(EEPROM_ADDR_SERV_MAX[servoCounter], tempInfoPositionMax);     
      servoInfoDisplay(SERVO_NAMES_ITERATED[servoCounter], tempInfoPositionMin, tempInfoPositionMax);
      state = LEAD;      
      return false;

    case SERVO_PREV:
      servoCounter--;
      if (servoCounter < 0) servoCounter = strlen((const char*)SERVO_ITERATED) - 1; 
      servoCalibPos = SERVO_CENTRAL_POSITION;   // центровка сервы при переключении
      pwm.setPWM(SERVO_ITERATED[servoCounter], 0, servoCalibPos);

      EEPROM.get(EEPROM_ADDR_SERV_MIN[servoCounter], tempInfoPositionMin);
      EEPROM.get(EEPROM_ADDR_SERV_MAX[servoCounter], tempInfoPositionMax);     
      servoInfoDisplay(SERVO_NAMES_ITERATED[servoCounter], tempInfoPositionMin, tempInfoPositionMax);
      state = LEAD;
      return false;

    case SERVO_CENTERING:
      servoCalibPos = SERVO_CENTRAL_POSITION; // установка центрального значения для текущей сервы
      pwm.setPWM(SERVO_ITERATED[servoCounter], 0, servoCalibPos);
      
      EEPROM.get(EEPROM_ADDR_SERV_MIN[servoCounter], tempInfoPositionMin);
      EEPROM.get(EEPROM_ADDR_SERV_MAX[servoCounter], tempInfoPositionMax);     
      servoInfoDisplay(SERVO_NAMES_ITERATED[servoCounter], tempInfoPositionMin, tempInfoPositionMax);
      state = LEAD;
      return false;

    case SERVO_MOVE_UP:
      servoCalibPos++;
      pwm.setPWM(SERVO_ITERATED[servoCounter], 0, servoCalibPos);
      delay(SERVO_CALIBRATE_DELAY);
      servoCalibrateDisplay(SERVO_NAMES_ITERATED[servoCounter], servoCalibPos);
      state = LEAD;
      return false;

    case SERVO_MOVE_DOWN:
      servoCalibPos--;
      pwm.setPWM(SERVO_ITERATED[servoCounter], 0, servoCalibPos);
      delay(SERVO_CALIBRATE_DELAY);
      servoCalibrateDisplay(SERVO_NAMES_ITERATED[servoCounter], servoCalibPos);
      state = LEAD;
      return false;

    case SERVO_FIND_MAX:
      EEPROM.put(EEPROM_ADDR_SERV_MAX[servoCounter], servoCalibPos);
#if VERSION == 11
      beep(NOTE_C, 200);
      noTone(BUZZER_CH);
#endif
#if VERSION == 10
      beep(1, 200);
#endif

      EEPROM.get(EEPROM_ADDR_SERV_MIN[servoCounter], tempInfoPositionMin);
      EEPROM.get(EEPROM_ADDR_SERV_MAX[servoCounter], tempInfoPositionMax);     
      servoInfoDisplay(SERVO_NAMES_ITERATED[servoCounter], tempInfoPositionMin, tempInfoPositionMax);
      state = LEAD;
      return false;

    case SERVO_FIND_MIN:
      EEPROM.put(EEPROM_ADDR_SERV_MIN[servoCounter], servoCalibPos);
#if VERSION == 11
      beep(NOTE_C, 200);
      noTone(BUZZER_CH);
#endif
#if VERSION == 10
      beep(1, 200);
#endif

      EEPROM.get(EEPROM_ADDR_SERV_MIN[servoCounter], tempInfoPositionMin);
      EEPROM.get(EEPROM_ADDR_SERV_MAX[servoCounter], tempInfoPositionMax);     
      servoInfoDisplay(SERVO_NAMES_ITERATED[servoCounter], tempInfoPositionMin, tempInfoPositionMax);
      state = LEAD;      
      return false;

    case EXIT:
      readServoRange(); // чтение границ серв из епрома и запись в глобальные переменные
#if VERSION == 11
      beep(NOTE_C, 50);
      //beep(NOTE_G, 300);
      //beep(NOTE_B, 300);
      noTone(BUZZER_CH);
#endif
#if VERSION == 10
      beep(1, 500);
#endif
      servoCentering();   // центровка серв
      servoCounter = 0;
      servoCalibPos = SERVO_CENTRAL_POSITION;
      display.clearDisplay();   // чистим дисплей
      display.display();  
      standIdleTimer = millis();  // запомнить время последнего действия
      state = LEAD;  
      return true;
  }
}


bool workFSM()    // рабочий режим
{
  static bool isPlowDown = false;  // опущен ли плуг
  static enum
  {
    LEAD,  // управляющий режим
    FORWARD,  // вперед
    BACKWARD, // назад 
    LEFT,   // влево
    RIGHT,  // вправо
    STOP, // остановка
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
             ps2x.Button(PSB_PAD_LEFT) || ps2x.Button(PSB_PAD_RIGHT)))) { state = STOP; }
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
      setSpeedRight(motorSpeed);
      setSpeedLeft(motorSpeed);
      standIdleTimer = millis();  // запомнить время последнего действия
      state = LEAD;
      return false;

    case BACKWARD:
      setSpeedRight(-motorSpeed);
      setSpeedLeft(-motorSpeed);
      standIdleTimer = millis();  // запомнить время последнего действия
      state = LEAD;
      return false;

    case LEFT:
      setSpeedRight(motorSpeed);
      setSpeedLeft(-motorSpeed);
      standIdleTimer = millis();
      state = LEAD;
      return false;

    case RIGHT:
      setSpeedRight(-motorSpeed);
      setSpeedLeft(motorSpeed);
      standIdleTimer = millis();
      state = LEAD;
      return false;

    case STOP:
      stopMotors();
      state = LEAD;      
      return false;

    case SPEED_UP:
      motorSpeed = rerangeSpeed(motorSpeed + SPEED_STEP);
      standIdleTimer = millis();
      state = LEAD;
      return false;

    case SPEED_DOWN:
      motorSpeed = rerangeSpeed(motorSpeed - SPEED_STEP);
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
      standIdleTimer = millis();
      state = LEAD;
      return false;

     case PLANT_ACTIVATION:
      plantActivate();
      standIdleTimer = millis();
      state = LEAD;
      return false;

    case BUCKET_UP:

      standIdleTimer = millis();
      state = LEAD;
      return false;

    case BUCKET_DOWN:

      standIdleTimer = millis();
      state = LEAD;
      return false;

    case BUCKET_GRAB_CLAMP:

      standIdleTimer = millis();
      state = LEAD;
      return false;

    case BUCKET_GRAB_LOOSE:

      standIdleTimer = millis();
      state = LEAD;
      return false;

    case EXIT:

      standIdleTimer = millis();
      state = LEAD;
      return true;
  }
}




void setup() 
{
  displaySetup();
  motorSetup();
  servoSetup();   // инициализация серв
  servoCentering();   // центрирование серв
  
  pinMode(A4, INPUT_PULLUP);    // подтяжка линий I2C к питанию, мб и не надо 
  pinMode(A5, INPUT_PULLUP);

  //установка выводов и настроек: GamePad(clock, command, attention, data, Pressures?, Rumble?) проверка ошибок
  ps2x.config_gamepad(JOY_CLK_CH, JOY_CMD_CH, JOY_SEL_CH, JOY_DAT_CH, JOY_PRESSURES, JOY_RUMBLE); // что выдает error ????

  readServoRange();   // чтение границ серв из епрома и запись в глобальные переменные

#if VERSION == 11
  //мелодия включения
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
  
  Serial.begin(9600);
  
  standIdleTimer = millis(); // запомнить время последнего действия
}




void loop()
{
  static bool m_exit = false; // доп переменная для хранения данных о выходе из некоторых конечных автоматов
  static enum 
  {
    WORK,   // рабочий режим - ездит, кривляется
    CALIBRATION  // режим калибровки
  } state;

  ps2x.read_gamepad(false, 0); // считывание данных с джойстика и установка скорости вибрации !!! (пока так)
  switch(state)
  {
    case WORK:
      //m_exit = workFSM();   // крутимся в рабочем режиме, пока не придет флаг о выходе из него - вернется true
      if(m_exit)  state = CALIBRATION;
      m_exit = true;
      break;

    case CALIBRATION:
      m_exit = calibrationFSM();  // крутимся в режиме калибровки, пока не придет флаг о выходе из него - вернется true
      if(m_exit)  state = WORK;
      m_exit = false;
      break;    
  }
  delay(5);
}

