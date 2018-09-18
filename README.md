# Agrobot


## Описание
Прошивка Агробота, скетч для Arduino IDE. 

## Порядок установки необходимых библиотек
1. Все библиотеки необходимо скачать по ссылкам и распаковать в домашнем каталоге ArduinoIDE Arduino/libraries _(при отсутствии папки libraries в каталоге Arduino необходимо создать её)_, каждую в соответствующую ей папку (например, Arduino/libraries/Adafruit-GFX-Library).

[Adafruit-GFX-Library](https://github.com/adafruit/Adafruit-GFX-Library)

[Adafruit-PWM-Servo-Driver-Library-master](https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library)

[Adafruit_SSD1306](https://github.com/adafruit/Adafruit_SSD1306)

[PS2X_lib](https://github.com/madsci1016/Arduino-PS2X/tree/master/PS2X_lib)

2. Скачать архив Agrobot.zip, распаковать его в домашнюю папку Arduino IDE. Файл pictures.h уже размещён в каталоге проекта в папке libraries/pictures. **Переносить его не нужно.**

## Управление
Управление роботом наглядно представлено в файле ```instructions.md```

**Внимание!** Если при включении питания робот не подаёт звуковой сигнал, соответствующий успешной инициализации, а его дисплей не активизируется, следует:
1. Отключить питание.
2. Вынуть из разъёма приёмник джойстика.
3. Включить питание робота и дождаться успешной инициализации.
4. После подачи звукового сигнала установить приёмник джойстика в соответствующий ему разъём.

