#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_PWMServoDriver.h>

#include <PS2X_lib.h>

#include <pictures.h> //массивы изображений для дисплея

//сервы
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // called this way, it uses the default address 0x40
#define SERVO_BUCKET_MIN  281
#define SERVO_BUCKET_MAX  445
#define SERVO_BUCKETUD_MIN  187
#define SERVO_BUCKETUD_MAX  421
#define SERVO_PLOW_MIN  320
#define SERVO_PLOW_MAX  410
#define SERVO_PLANT_MIN  320
#define SERVO_PLANT_MAX  428

#define SERVO_BUCKET_CH  7
#define SERVO_BUCKETUD_CH 6
#define SERVO_PLOW_CH  5
#define SERVO_PLANT_CH 4

#define DSERVO_const 5
#define SERVO_FREQ 60
#define SERVO_DELAY 7

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
#define note_c 261
#define note_d 294
#define note_e 329
#define note_f 349
#define note_g 391
#define note_a 440
#define note_b 466

//пауза
#define MAXCOUNT_PAUSE 350*2  //350 = 30c

//Режимы работы джойстика
//- pressures = аналоговое считывание нажатия кнопок
//- rumble    = вибромоторы

#define pressures   true
//#define pressures   false
#define rumble      true
//#define rumble      false

//изображение
#define imageWidth 128
#define imageHeight 64

//регулирование скорости
#define SPEED_MIN 90
#define SPEED_MAX 255
#define Dspeed_const 5

//для АЦП
#define UAREF 5.0
#define ADC_MAX 1024
#define ADC_PIN_VOLTAGE A1
#define ADC_PIN_CURRENT A0
#define DEL_CONST 1 //константа делителя напряжения
#define MAXCOUNT_ADC 15 //задержка преобразования АЦП
//#define MIN_MCU_CURRENT
#define MIN_MCU_VOLTAGE 3.3

//Состояния робота
#define state_tired 0
#define state_go  1
#define state_goback  2
#define state_turnleft  3
#define state_turnright 4
#define state_servoaction  5
#define state_notmove 6
#define state_pause 10

int motorspeed = SPEED_MIN;
int count_ADC = MAXCOUNT_ADC;
unsigned int count_pause  = 0;
int pulselen_bucket = 350;
int pulselen_bucketud = 300;
int pulselen_plow = SERVO_PLOW_MIN;
int pulselen_plant = SERVO_PLANT_MIN;
//char outstr[4];
float mcu_voltage;
unsigned char robo_state = state_notmove;
unsigned char state_plow = 0;


//Создание класса для джойстика
PS2X ps2x;
int error = 0;
byte type = 0;
byte vibrate = 0;

void setup() {

  //Настройка выводов под драйвер
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_ENB, OUTPUT);
  pinMode(MOTOR_IN1A, OUTPUT);
  pinMode(MOTOR_IN1B, OUTPUT);
  pinMode(MOTOR_IN2A, OUTPUT);
  pinMode(MOTOR_IN2B, OUTPUT);

  pinMode(BUZZER, OUTPUT);
  noTone(BUZZER);

  //Установка частоты ШИМ
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~60 Hz updates



  //Инициализация дисплея
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  // init done
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(2000);
  // Clear the buffer.
  display.clearDisplay();

  //установка выводов и настроек: GamePad(clock, command, attention, data, Pressures?, Rumble?) проверка ошибок
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);

  //мелодия включения
    beep(note_c, 400);
    beep(note_e, 350);
    beep(note_g, 150);
    beep(note_b, 400);
    noTone(BUZZER);

  //Serial.begin(9600);

  //Настройка опорного напряжения для АЦП: внешний источник на выводе AREF
  analogReference(EXTERNAL);

  pwm.setPWM(SERVO_PLANT_CH, 0, SERVO_PLANT_MAX);
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
    //    dtostrf(mcu_voltage, 4, 2, outstr); //преобразование флоат в строку 4 символа в строке, 2 знака после запятой
  }
  else count_ADC++;

  //меньше порога достаточного заряда аккумулятора
  if (mcu_voltage < MIN_MCU_VOLTAGE)
  {
    robo_state = state_tired;
      beep(note_b, 400);
      beep(note_g, 350);
      beep(note_e, 150);
      beep(note_c, 400);
      noTone(BUZZER);

  }
  else
  {
    // ВВЕРХ нажато
    if (ps2x.Button(PSB_PAD_UP))
    {
      SetSpeedRight(motorspeed);
      SetSpeedLeft(motorspeed);
      robo_state = state_go;
      count_pause = 0;
    }

    //ВНИЗ нажато
    if (ps2x.Button(PSB_PAD_DOWN))
    {
      robo_state = state_goback;

      SetSpeedRight(-motorspeed);
      SetSpeedLeft(-motorspeed);
      count_pause = 0;
    }

    // Крестовина отпущена
    if ((ps2x.Button(PSB_PAD_UP) == false) & (ps2x.Button(PSB_PAD_DOWN) == false) &
        (ps2x.Button(PSB_PAD_LEFT) == false) & (ps2x.Button(PSB_PAD_RIGHT) == false))
    {

      StopMotors();
      if (count_pause < MAXCOUNT_PAUSE)
        robo_state = state_notmove;
      //не нажата ни одна кнопка действия
      if ((ps2x.Button(PSB_TRIANGLE) == false) & (ps2x.Button(PSB_CROSS) == false) &
          (ps2x.Button(PSB_CIRCLE) == false) & (ps2x.Button(PSB_SQUARE) == false))
      {
        if (count_pause < MAXCOUNT_PAUSE)
          count_pause++;

        if (count_pause == MAXCOUNT_PAUSE)
        {
          robo_state = state_pause;
          //мелодия
            beep(note_g, 300);
            beep(note_g, 150);
            beep(note_f, 150);
            beep(note_e, 150);
            beep(note_a, 300);
            noTone(BUZZER);
          count_pause++;
        }
      }
    }

    // ВПРАВО нажато
    if (ps2x.Button(PSB_PAD_RIGHT))
    {
      robo_state = state_turnright;
      SetSpeedRight(-motorspeed);
      SetSpeedLeft(motorspeed);
      count_pause = 0;
    }

    //ВЛЕВО нажато
    if (ps2x.Button(PSB_PAD_LEFT))
    {
      robo_state = state_turnleft;
      SetSpeedRight(motorspeed);
      SetSpeedLeft(-motorspeed);
      count_pause = 0;
    }


    //vibrate = ps2x.Analog(PSAB_CROSS);  //Скорость вибрации устанавливаеться в зависимости от силы нажатия кнопки (X)

    // L2 нажата (плуг)
    if (ps2x.ButtonPressed(PSB_L2))
    {
      count_pause = 0;
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

    if (ps2x.ButtonPressed(PSB_L1))
    {
      count_pause = 0;

        pwm.setPWM(SERVO_PLANT_CH, 0, SERVO_PLANT_MIN);
        delay(300);
      
        pwm.setPWM(SERVO_PLANT_CH, 0, SERVO_PLANT_MAX);

      }

    
    // R1 нажата
    if (ps2x.Button(PSB_R1))
    {
      count_pause = 0;
      if (motorspeed < (SPEED_MAX - Dspeed_const))
      {
        motorspeed = motorspeed + Dspeed_const;
      }
      else motorspeed = SPEED_MAX;
    }

    // R2 нажата
    if (ps2x.Button(PSB_R2))
    {
      count_pause = 0;
      if (motorspeed > (SPEED_MIN + Dspeed_const))
      {
        motorspeed = motorspeed - Dspeed_const;
      }
      else motorspeed = SPEED_MIN;
    }






    // Треугольник нажат
    if (ps2x.Button(PSB_TRIANGLE))
    {
      count_pause = 0;
      if (pulselen_bucketud > (SERVO_BUCKETUD_MIN + DSERVO_const))
      {
        pulselen_bucketud = pulselen_bucketud - DSERVO_const;
        pwm.setPWM(SERVO_BUCKETUD_CH, 0, pulselen_bucketud);
      }
      else pulselen_bucketud = SERVO_BUCKETUD_MIN;
      robo_state = state_servoaction;
      delay(SERVO_DELAY);
    }


    //Х нажат
    if (ps2x.Button(PSB_CROSS))
    {
      count_pause = 0;
      if (pulselen_bucketud < (SERVO_BUCKETUD_MAX - DSERVO_const))
      {
        pulselen_bucketud = pulselen_bucketud + DSERVO_const;
        pwm.setPWM(SERVO_BUCKETUD_CH, 0, pulselen_bucketud);
      }
      else pulselen_bucketud = SERVO_BUCKETUD_MAX;

      robo_state = state_servoaction;
      delay(SERVO_DELAY);
    }

    //Круг нажат
    if (ps2x.Button(PSB_CIRCLE))
    {
      count_pause = 0;
      if (pulselen_bucket < (SERVO_BUCKET_MAX - DSERVO_const))
      {
        pulselen_bucket = pulselen_bucket + DSERVO_const;
        pwm.setPWM(SERVO_BUCKET_CH, 0, pulselen_bucket);
      }
      else pulselen_bucket = SERVO_BUCKET_MAX;
      robo_state = state_servoaction;
      delay(SERVO_DELAY);
    }

    //Квадрат нажат
    if (ps2x.Button(PSB_SQUARE))
    {
      count_pause = 0;
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
    case state_tired:
      {
        display.clearDisplay();
        display.drawBitmap(0, 0,  eyes_tired, imageWidth, imageHeight, 1);
        display.display();

        while (mcu_voltage < MIN_MCU_VOLTAGE)
          mcu_voltage = DEL_CONST * analogRead(ADC_PIN_VOLTAGE) * UAREF / ADC_MAX;
        robo_state = state_notmove;
        break;
      }
    case state_go:
      {
        display.clearDisplay();
        display.drawBitmap(0, 0,  eyes_up, imageWidth, imageHeight, 1);
        display.display();
        break;
      }
    case state_goback:
      {
        display.clearDisplay();
        display.drawBitmap(0, 0,  eyes_down, imageWidth, imageHeight, 1);
        display.display();
        break;
      }
    case state_turnleft:
      {
        display.clearDisplay();
        display.drawBitmap(0, 0,  eyes_left, imageWidth, imageHeight, 1);
        display.display();
        break;
      }
    case state_turnright:
      {
        display.clearDisplay();
        display.drawBitmap(0, 0,  eyes_right, imageWidth, imageHeight, 1);
        display.display();
        break;
      }
    case state_servoaction:
      {
        display.clearDisplay();
        display.drawBitmap(0, 0,  eyes_difficult, imageWidth, imageHeight, 1);
        display.display();
        break;
      }
    case state_pause:
      {
        display.clearDisplay();
        display.drawBitmap(0, 0,  eyes_wow, imageWidth, imageHeight, 1);
        display.display();
        break;
      }
    case state_notmove:
      {
        display.clearDisplay();
        display.drawBitmap(0, 0,  eyes_cute, imageWidth, imageHeight, 1);
        display.display();
        break;
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
//для проигрывания тона
void beep(int ton, int tim)
{
  tone(BUZZER, ton, tim);
  delay(tim + 20);
}


