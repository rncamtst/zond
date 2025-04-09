#include <SPI.h> //библиотека для работы с SPI
#include <SoftwareSerial.h>
#include <LoRa.h> //библиотека для работы с LoRa
#include <Streaming.h> //хз что, но работает с часами
#include <DS3232RTC.h> //библиотека для работы с часами
#include <Wire.h> //библиотека i2c
#include <Sleep_n0m1.h> //библиотека для сна
#include <Adafruit_BMP280.h> //библиотека для работы с bmp280
#include <Adafruit_Sensor.h>// Подключаем библиотеку Adafruit_Sensor
#include "Adafruit_HTU21DF.h" //библиотека для работы с htu21
#include <OneWire.h> //библиотека для работы c шиной 1Wire
#include <DallasTemperature.h> //библиотека для работы с датчиком ds18b20
#include <EEPROM.h> //библиотека для работы с энергонезависимой памятью
//#include <SD.h> //библиотека для работы с SD-картой
#include <Adafruit_ADS1X15.h>// внещний АЦП
#include <Eeprom24C128_256.h>// работа с внешней EEPROM памятью

#define batt_pin A7 //для считывания заряда АКБ
#define SD_CS 9 //Chip select для карты памяти
#define NSS 10 //пин Chip select (slave select) lora
#define RST 3 //пин сброса lora
#define DIO0 4 // цифровой пин для lora
#define TXEN 8 //режим на передачу (не используется)
#define RXEN 7 //режим на прием (не используется)
#define DS18B20_PIN A0 //пин data датчика ds18b20
#define INT0_PIN 2 //пин для пробуждения
#define LED_PIN 1 //пин для светодиода
#define EEPROM_ADDRESS 0x50 // адрес внешней еепром на шине i2c

SoftwareSerial mySerial(5, 6); // RX, TX
Sleep sleep; //объявляем класс для сна
DS3232RTC myRTC; //объявляем класс для часов
Adafruit_BMP280 bmp; //объявляем класс для bmp280
Adafruit_HTU21DF htu = Adafruit_HTU21DF(); //объявляем класс для htu21
OneWire oneWire_in(DS18B20_PIN); //объявляем класс для onewire
DallasTemperature sensor_inhouse(&oneWire_in); //объявляем класс для ds18b20
Adafruit_ADS1115 ads; //class for ADS1115
static Eeprom24C128_256 eeprom(EEPROM_ADDRESS); // class for eeprom at24c256

float value, voltage, k, c, h, p; //переменные для работы АЦП
volatile boolean flag = false; //сбрасываем флаг прерываний
boolean flag1 = true; //сбрасываем флаг прерываний
const uint8_t alarmInput(INT0_PIN); //2 пин для прерываний
uint16_t i; //счетчик
char str[200];
uint8_t st, st_prev;
unsigned long t1 = 0;
unsigned long t2 = 0;
int8_t max_pwr = 20;
int8_t min_pwr = -4;
int8_t crnt_pwr = 8; //мощность передатчика
uint32_t id; //идентификатор
uint8_t time_start_day, time_start_hour, time_start_min;
uint8_t period_min = 1; //период пробуждения датчика в минутах (задавать в диапазоне от 1 до 59)
uint8_t period_hour = 1; //период пробуждения датчика в часах (задвать в диапазоне от 1 до 23)
unsigned char Sec,Min,Hour,Day,date,Month,Year; //переменные для хранения времени
static uint8_t mydata[32]; //переменная для хранения всех данных побайтно
static uint8_t PROTOCOL = 0x7F; //номер первого протокола
static uint8_t rxbuf[1]; //приемный буфер
uint8_t chrg;
int packetSize = 0;
String namefile; //имя файла на карте памяти
int current_day = 0; //текущий день (для обновления шапки в csv файле)
union //для хранения данных в разного типа (объединение)
 {
  unsigned char x; 
  signed short sgnshrt;
 } itob;






//---------------------------SETUP--------------------------------
void setup() 
{
  
  pinMode(LED_PIN, OUTPUT); //вывод для индикатороного светодиода
  digitalWrite(LED_PIN, HIGH); //индикация включения зонда
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(500);
  digitalWrite(LED_PIN, HIGH); //индикация включения зонда
  delay(500);
  digitalWrite(LED_PIN, LOW);
  
  k = 0.00085217391; //коэффициент преобразования "попугаев" в вольты для АКБ
  c = 5.058; //коэффициент деления для делителя 4х300кОм
  h = 1.1399; //коэффициент преобразования "попугаев" в вольты для солнечной панели
  p = 43; //верхнее значение АКБ * 10; 4.2В.
  
  //detachInterrupt(0); //запрещаем прерывания
  Wire.begin(); //запускаем twi
  analogReference(INTERNAL); //опорное напряжение для АЦП 1.1В
  mySerial.begin(9600); //последовательный порт 9600 бит/с  
  
  if (EEPROM.read(7) != 0x03) //проверят была ли настройка часов ранее
  {
    RTC.set(compileTime()); //устанавливаем время компиляции
    mySerial.println("Set compile time");
    EEPROM.update(7, 0x03);
  }
  for (i = 0; i <= sizeof(mydata); i++) //обнуляем переменную для хранения данных
    mydata[i] = 0x00;

  LoRa.setPins(NSS, RST, DIO0); //переопределение роли каждого пина
  if (!LoRa.begin(868E6)) {
    mySerial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.setSignalBandwidth(125E3);
  LoRa.setSpreadingFactor(12); 
  for (i = 1; i <= 4; i++) mydata[i] = EEPROM.read(i-1); //2-5 байт - читаем из EEPROM адрес устойства
 // set_power(); //АРМ
  mySerial.print("Power adjusting is successful. Transmitter power: ");
  mySerial.print(crnt_pwr);
  mySerial.println(" dBm");

       
//________________________________________________________________________________________________

  mydata[0] = PROTOCOL; //1 байт - записываем номер протокола
  mySerial.print("Probe's ID: ");
  id = *reinterpret_cast<uint32_t*>(mydata+1); //записываем id в одну 4х байтную переменную
  mySerial.println(id);
  pinMode(INT0_PIN, INPUT_PULLUP); //подтягиваем 2 пин к питанию (чекнуть работают ли часы на +5 в, а не на землю по прерыванию)
  RTC.squareWave(SQWAVE_NONE); //конфигурируем пин SQW (DS3231) на прерывания
  RTC.alarmInterrupt(ALARM_1, true); //разрешаем прерывания по 1 будильнику
  }
//---------------------------LOOP---------------------------------
void loop() 
{
      read_RTC(); //читаем и записываем в mydata температуру и время  
      read_bmp280_temp(); //считывание температуры с bmp
      read_bmp280_press(); //считывание давления с bmp
      read_htu21_temp(); //считывание температуры с htu
      read_htu21_hum(); //считывание влажности с htu
      read_ds18b20(); //считывание температуры с ds18b20
      charge(); //считывание показаниц АЦП (напряжение батареи)
      spanel(); //считывание и расчет тока заряда солнечной панели
      //SD_write(); //считываем все данные и записываем на карту памяти 
      read_humADC();
      write_at24c256(); // запись массива во внешнуюю еепром ат24с256
      crc(); //контрольная сумма
      lora_send_data(); //отправляем данные
     
      mySerial.print("Data packet: ");
      for (int i = 0; i < sizeof(mydata); i++) {
        mySerial.print(i);mySerial.print(" "); mySerial.print(mydata[i], DEC); mySerial.print(" ");    //вывод данных

      }
      mySerial.println();
      
    delay(500); //задержка нужна для корректного вывода данных в порт
    set_alarm_min();
   // set_alarm_hour();
}
//---------------------------SET POWER---------------------------------
void set_power()
{
  mySerial.println("Auto power adjusting");
  sprintf(str, "Transmitter power = %d dBm.", crnt_pwr);
  mySerial.println(str);
  rxbuf[0] = 0xFF; //для корректной отправки данных
  mydata[0] = 0x9B; //протокол для автоматической регулировки мощности
  crnt_pwr = 8; //привожу мощность к опорному минимальному уровню crnt_pwr == 8 дБм
  st = 12;
  while (st > 0 && crnt_pwr < 20 && crnt_pwr > -4) //пока шаг подстройки мощности > 1 и мощность не максимальная и не минимальная - выполняется автоподстройка
  {
    LoRa.setTxPower(crnt_pwr); //устанавливаем мощность передатчика
    LoRa.beginPacket(); //отправляем команду базовой станции о том, что сейчас будет выполняться автоподстройка мощности 
    LoRa.write(mydata, sizeof(mydata));
    LoRa.endPacket();
    t1 = millis();
    t2 = millis();
    
    while (packetSize == 0 && (t2 - t1 < 3000)) //пока не получили ответ от БС или не вышел таймаут
    {
      packetSize = LoRa.parsePacket(); //ожидание приема
      t2 = millis();
      if (t2 - t1 >= 3000) 
      {
        rxbuf[0] = 0x9C; //ответа нет, выходим по таймауту 
        mySerial.println("------***------\nNo response was received from base station!\nBase station is power on?\n------***------");
      }
    }
    if (LoRa.available()) 
    {
      rxbuf[0] = LoRa.read(); //ответ получен, понижаем мощность
    }
    if (rxbuf[0] == 0x9A) //ответ получен, мощности хватает
    {
      if (st > 0) st = (max_pwr - min_pwr) / 2; //половина текущей мощности (+ учитываем что динамический диапазон от -4) 
      max_pwr = crnt_pwr;
      if (st != 1) crnt_pwr = crnt_pwr - st/2; else crnt_pwr = crnt_pwr - st;
      sprintf(str, "Transmitter power decreasing. Power = %d dBm.", crnt_pwr);
      mySerial.println(str);
      slow2_blink();
    } else 
      if (rxbuf[0] == 0x9C) //ответ не получен, мощности не хватает либо БС отключена
      {
        if (st > 0) st = (max_pwr - min_pwr) / 2;
        min_pwr = crnt_pwr;
        if (st != 1) crnt_pwr = crnt_pwr + st/2; else crnt_pwr = crnt_pwr + st;
        sprintf(str, "Transmitter power increasing. Power = %d dBm.", crnt_pwr);
        mySerial.println(str);
        fast4_blink();
      }
    rxbuf[0] = 0xFF;
    packetSize = 0;
  }
  mydata[0] = 0x7F;
}
//---------------------------SET ALARM MINUTE---------------------------------
void set_alarm_min()
{
//  if (flag1) 
//  {
//    time_start_min = Min + period_min;
//    flag1 = false;
//    //mySerial.println(String(Min) + " " + String(period));
//  } else 
  mySerial.print("Send data period: ");
  mySerial.print(period_min);
  mySerial.println(" minute(s).");
  delay(10);
  time_start_min = Min + period_min;
  if (time_start_min > 59) 
  {
    time_start_min = time_start_min - 60;
    time_start_hour = Hour + 1;
  } else time_start_hour = Hour;
  sprintf(str, "Now: %u.%u.%u %u:%u:%u", date, Month, Year, Hour, Min, Sec);
  mySerial.println(str);
  delay(10);
  sprintf(str, "Next time start: %u.%u.%u %u:%u:%u", date, Month, Year, time_start_hour, time_start_min, 0);
  mySerial.println(str);
  delay(500);
  RTC.setAlarm(ALM1_MATCH_MINUTES, 0, time_start_min, 0, 0); //ставим будильник каждую секунду ALM1_MATCH_MINUTES ALM1_EVERY_SECOND time_start
  RTC.alarm(ALARM_1); //очищаем флаг будильника ALARM_1
  sleep.sleepInterrupt(0,FALLING); //отправляем в сон и будим по прерыванию на 2 пине   
}
//---------------------------SET ALARM HOUR---------------------------------
void set_alarm_hour()
{
//  if (flag1) 
//  {
//    time_start_hour = Hour + period_hour;
//    flag1 = false;
//    mySerial.println(String(Hour) + " " + String(period));
//  } else 
  mySerial.print("Send data period: ");
  mySerial.print(period_hour);
  mySerial.println(" hour(s)");
  delay(10);
  time_start_hour = Hour + period_hour;
  if (time_start_hour >= 24) 
  {
    time_start_hour = time_start_hour - 24;
    time_start_day = date + 1;
  } else time_start_day = date;
  time_start_min = id % 60;

  sprintf(str, "Now: %u.%u.%u %u:%u:%u", date, Month, Year, Hour, Min, Sec);
  mySerial.println(str);
  delay(10);
  sprintf(str, "Next time start: %u.%u.%u %u:%u:%u", time_start_day, Month, Year, time_start_hour, time_start_min, 0);
  mySerial.println(str);
  delay(100);
  RTC.setAlarm(ALM1_MATCH_HOURS, 0, time_start_min, time_start_hour, 0); //ставим будильник каждую секунду ALM1_MATCH_MINUTES ALM1_EVERY_SECOND time_start
  RTC.alarm(ALARM_1); //очищаем флаг будильника ALARM_1
  sleep.sleepInterrupt(0,FALLING); //отправляем в сон и будим по прерыванию на 2 пине   
}
//---------------------------CHARGE------------------------------------
void charge(void)
{
  for (i = 0; i<=1000; i++) //усреднение
    {
      value = analogRead(A7);
      voltage = voltage + value;
    }
  voltage = voltage / (i+1) * k * c * 100; // * k * c * 100; //делим на 1002, умножаем на коэффициенты
  if (voltage < 2.1) chrg = 0; else chrg = map(round(voltage / 10), 21, p, 0, 1000) / 10; //84%
  itob.sgnshrt = voltage;
  mydata[22] = itob.sgnshrt;
  mydata[23] = itob.sgnshrt >> 8;
  mydata[24] = chrg;
//  mySerial.print("Напряжение АКБ: ");
//  mySerial.print(voltage);
//  mySerial.println(" В ");
//  mySerial.print(mydata[22], HEX);
//  mySerial.println(mydata[23], HEX);
//  mySerial.print("Заряд: ");
//  mySerial.print(chrg);
//  mySerial.print("% ");
//  mySerial.println(mydata[24], HEX);
}
//---------------------------SOLAR------------------------------------
void spanel(void)
{
  voltage = 0;
  for (i = 0; i<=1000; i++) //усреднение
    {
      value = analogRead(A3) * h;
      voltage = voltage + value;
    }
  voltage = voltage / (i+1) / 22.2 * 100; //делим на 1002, считаем ток поделив на 22.2 Ом
  voltage = 0;
  itob.sgnshrt = voltage;
  mydata[25] = itob.sgnshrt;
  mydata[26] = itob.sgnshrt >> 8;
//  mySerial.print("Ток заряда: ");
//  mySerial.print(voltage/100);
//  mySerial.print(" мА ");
//  mySerial.print(mydata[25], HEX);
//  mySerial.println(mydata[26], HEX);
}
//---------------------------RTC---------------------------------------
void read_RTC(void)
{
  I2C_SendByteByADDR(0,0b11010000); //переходим на адрес 0
  I2C_StartCondition(); //Отправим условие START
  I2C_SendByte(0b11010001); //отправим в устройство бит чтения
  Sec = I2C_ReadByte();
  Min = I2C_ReadByte();
  Hour = I2C_ReadByte();
  Day = I2C_ReadByte();
  date = I2C_ReadByte();
  Month = I2C_ReadByte();
  Year = I2C_ReadLastByte();
  I2C_StopCondition(); //Отправим условие STOP
  
  Sec = bcd2dec(Sec); //Преобразуем в десятичный формат
  Min = bcd2dec(Min); //Преобразуем в десятичный формат
  Hour = bcd2dec(Hour); //Преобразуем в десятичный формат
  Day = bcd2dec(Day); //Преобразуем в десятичный формат
  Year = bcd2dec(Year); //Преобразуем в десятичный формат
  Month = bcd2dec(Month); //Преобразуем в десятичный формат
  date = bcd2dec(date); //Преобразуем в десятичный формат
  
  namefile = 'D'+String(date)+'M'+String(Month)+'Y'+String(Year);
//  mySerial.println(namefile);
  mydata[5] = date;
  mydata[6] = Month;
  mydata[7] = Year;
  mydata[8] = Hour;
  mydata[9] = Min;
  voltage = myRTC.temperature() / 4.0;
  itob.sgnshrt = voltage*100.0;
  mydata[10] = itob.sgnshrt;
  mydata[11] = itob.sgnshrt >> 8;
//  mySerial.print(date);
//  mySerial.print(".");
//  mySerial.print(Month);
//  mySerial.print(".");
//  mySerial.print(Year);
//  mySerial.print(" ");
//  mySerial.print(Hour);
//  mySerial.print(":");
//  mySerial.print(Min);
//  mySerial.print(":");
//  mySerial.print(Sec);
//  mySerial.println();
}
//------------------------------------------------------------------------
float read_bmp280_temp(void)
{
  bmp.begin(0x76);
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X1,      /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_OFF,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  voltage = bmp.readTemperature();
  itob.sgnshrt = voltage*100;
  mydata[12] = itob.sgnshrt;
  mydata[13] = itob.sgnshrt >> 8;
  return voltage;
}
//------------------------------------------------------------------------
float read_bmp280_press(void)
{
  voltage = bmp.readPressure()/133.332;
  long int result = round(voltage*10);
  itob.sgnshrt = result;
  mydata[14] = itob.sgnshrt;
  mydata[15] = itob.sgnshrt >> 8;
  return voltage;
}
//----------------------------------------------------------------------
float read_htu21_temp(void)
{
  htu.begin();
  voltage = htu.readTemperature();
  long int result = voltage*100;
  itob.sgnshrt = result;
  mydata[16] = itob.sgnshrt;
  mydata[17] = itob.sgnshrt >> 8;
  return voltage;
}
//----------------------------------------------------------------------
float read_htu21_hum(void)
{
  voltage = htu.readHumidity();
  long int result = voltage*100;
  itob.sgnshrt = result;
  mydata[18] = itob.sgnshrt;
  mydata[19] = itob.sgnshrt >> 8;
  return voltage;
}
//----------------------------------------------------------------------
float read_ds18b20(void)
{
  sensor_inhouse.requestTemperatures(); //запрашиваем температуру с датчика
  voltage = sensor_inhouse.getTempCByIndex(0); //считываем температуру
//  mySerial.println(voltage);
  long int result = voltage*100;
  itob.sgnshrt = result;
  mydata[20] = itob.sgnshrt;
  mydata[21] = itob.sgnshrt >> 8;
  return voltage;
}

int crc(void)
{
  int crc = 0; // обнуляем переменную для хранения контрольной суммы
  for(int i = 0 ; i <= sizeof(mydata)-2; i++){
   
    
    crc = crc + mydata[i]; // к тому, что было в переменной crc добавить очередной элемент массива mydata
    
    }
  mySerial.println(crc);
  mydata[30] = crc;  // младший бит контр.суммы
  mydata[31] = crc >> 8; // старший бит контр.суммы
  }


  void read_humADC(void){
    uint8_t temp_points[7] = {3, 6, 9, 12, 15, 17, 19};
    static bool measurements_done[7] = {false};
    int64_t adc0 = 0, adc1 = 0, adc2 = 0; 
    uint16_t adc0_sr = 0, adc1_sr = 0, adc2_sr = 0;
    uint8_t h0 = 0, h1 = 0, h2 = 0; 
    uint16_t n = 1000;                                                // обьём выборки
    float k_5 = -0.0059; float k_10 = -0.0059; float k_15 = -0.0059;  //        коэфы функции 
    float b_5 = 118.65; float b_10 = 118.65;float b_15 = 118.65;      //    y = kx + b (H = k*ADC + b)
    uint16_t temp_ds18b20;
  ads.setGain(GAIN_ONE);
  ads.begin();
        signed short convert = (mydata[23] << 8) | mydata[22];
      temp_ds18b20 = convert;

 for (int i = 0; i < 7; i++) {
        if (temp_ds18b20 >= temp_points[i] && !measurements_done[i]) {
            // Обнуляем накопители
            adc0 = 0; adc1 = 0; adc2 = 0;
            
            // Выполняем измерения
            for (uint16_t k = 0; k < n; k++) {
                adc0 += ads.readADC_SingleEnded(0);
                adc1 += ads.readADC_SingleEnded(1);
                adc2 += ads.readADC_SingleEnded(2);
            }
            
            // Рассчитываем средние значения
            adc0_sr = adc0 / n;
            adc1_sr = adc1 / n;
            adc2_sr = adc2 / n;
            
            // Выбираем коэффициенты в зависимости от температуры
            float k, b;
           
            if(temp_ds18b20 < 5)
            {
              k = k_5; b = b_5;             //расчет H% по линейной калибровочной функции
              }
              
            if(temp_ds18b20 >= 5 && temp_ds18b20 <= 10)
            {
              k = k_5; b = b_5;
              } 
              
            if(temp_ds18b20 > 10 && temp_ds18b20 <= 15){
              k = k_10; b = b_10;
              }
                 
            if(temp_ds18b20 > 15){
              k = k_15; b = b_15;
              }
            
            // Рассчитываем влажность
            h0 = uint8_t(k * adc0_sr + b);
            h1 = uint8_t(k * adc1_sr + b);
            h2 = uint8_t(k * adc2_sr + b);
            
            // Сохраняем результаты
            mydata[27] = h0;
            mydata[28] = h1;
            mydata[29] = h2;
            
            // Выводим результаты
            mySerial.print("T:"); mySerial.println(temp_points[i]);
            mySerial.print("ADC0:"); mySerial.print(adc0_sr); mySerial.print(":H0:"); mySerial.println(h0);
            mySerial.print("ADC1:"); mySerial.print(adc1_sr); mySerial.print(":H1:"); mySerial.println(h1);
            mySerial.print("ADC2:"); mySerial.print(adc2_sr); mySerial.print(":H2:"); mySerial.println(h2);
            
            // Помечаем точку как измеренную
            measurements_done[i] = true;
            
            // Прерываем цикл после первого найденного измерения
            break;
        }
  
  }
  }






  void write_at24c256(void){
    unsigned long numWC = 0; // Счетчик циклов записи eeprom
    const unsigned long maxAddress = 32768; // Максимальный адрес для AT24C256 (32KB)
    
    numWC = read_numWCFromEEPROM();
    unsigned long startAddress = numWC * sizeof(mydata);

  // Проверяем, не вышли ли за пределы EEPROM 
  
    if (startAddress + sizeof(mydata) > maxAddress) {
      Serial.println("EEPROM is full! No more writing.");
      delay(1000);
      return;
    }

    for (int i = 0; i < sizeof(mydata); i++) {
      eeprom.writeByte(startAddress + i, mydata[i]);
      delay(10); // Небольшая задержка для надежности записи
    }

  numWC++;

  saveCounterToEEPROM(numWC);
  }

  
unsigned long read_numWCFromEEPROM() {
      const unsigned long maxAddress = 32768; // Максимальный адрес для AT24C256 (32KB)
// Функция для чтения счетчика из EEPROM (опционально)
  byte bytes[4];
  for (int i = 0; i < 4; i++) {
    bytes[i] = eeprom.readByte(maxAddress - 4 + i);
    delay(5);
  }
  
  return ((unsigned long)bytes[0] << 24) | 
         ((unsigned long)bytes[1] << 16) | 
         ((unsigned long)bytes[2] << 8) | 
         (unsigned long)bytes[3];
}



void saveCounterToEEPROM(unsigned long counter) {
      const unsigned long maxAddress = 32768; // Максимальный адрес для AT24C256 (32KB)
// Функция для сохранения счетчика в EEPROM (опционально)
  // Записываем 4 байта счетчика в последние ячейки EEPROM
  byte bytes[4];
  bytes[0] = (counter >> 24) & 0xFF;
  bytes[1] = (counter >> 16) & 0xFF;
  bytes[2] = (counter >> 8) & 0xFF;
  bytes[3] = counter & 0xFF;
  
  for (int i = 0; i < 4; i++) {
    eeprom.writeByte(maxAddress - 4 + i, bytes[i]);
    delay(5);
  }
}
//---------------------------BCD_TO_DEC---------------------------------
uint8_t bcd2dec(uint8_t n)
{
    return n - 6 * (n >> 4);
}
//---------------------------I2C----------------------------------------
void I2C_StartCondition(void)
{
  TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
  while(!(TWCR&(1<<TWINT)));//подождем пока установится TWIN
}
//----------------------------------------------------------------------
void I2C_StopCondition(void)
{
  TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
}
//----------------------------------------------------------------------
void I2C_SendByte(unsigned char c)
{
  TWDR = c;//запишем байт в регистр данных
  TWCR = (1<<TWINT)|(1<<TWEN);//включим передачу байта
  while (!(TWCR & (1<<TWINT)));//подождем пока установится TWIN
}
//----------------------------------------------------------------------
void I2C_SendByteByADDR(unsigned char c,unsigned char addr)
{
  I2C_StartCondition(); // Отправим условие START
  I2C_SendByte(addr); // Отправим в шину адрес устройства + бит чтения-записи
  I2C_SendByte(c);// Отправим байт данных
  I2C_StopCondition();// Отправим условие STOP
}
//----------------------------------------------------------------------
unsigned char I2C_ReadByte(void)
{
  TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
  while (!(TWCR & (1<<TWINT)));//ожидание установки бита TWIN
  return TWDR;//читаем регистр данных
}
//----------------------------------------------------------------------
unsigned char I2C_ReadLastByte(void)
{
  TWCR = (1<<TWINT)|(1<<TWEN);
  while (!(TWCR & (1<<TWINT)));//ожидание установки бита TWIN
  return TWDR;//читаем регистр данных
}
//----------------------------------------------------------------------
void slow2_blink(void)
{
  digitalWrite(LED_PIN, HIGH);
  delay(250);
  digitalWrite(LED_PIN, LOW);
  delay(250);
}
//----------------------------------------------------------------------
void fast4_blink(void)
{
  digitalWrite(LED_PIN, HIGH);
  delay(70);
  digitalWrite(LED_PIN, LOW);
  delay(70);
  digitalWrite(LED_PIN, HIGH); 
  delay(70);
  digitalWrite(LED_PIN, LOW);
  delay(70);
}
//----------------------------------------------------------------------
time_t compileTime()
{
    const time_t FUDGE(10);    //fudge factor to allow for upload time, etc. (seconds, YMMV)
    const char *compDate = __DATE__, *compTime = __TIME__, *months = "JanFebMarAprMayJunJulAugSepOctNovDec";
    char compMon[3], *m;

    strncpy(compMon, compDate, 3);
    compMon[3] = '\0';
    m = strstr(months, compMon);

    tmElements_t tm;
    tm.Month = ((m - months) / 3 + 1);
    tm.Day = atoi(compDate + 4);
    tm.Year = atoi(compDate + 7) - 1970;
    tm.Hour = atoi(compTime);
    tm.Minute = atoi(compTime + 3);
    tm.Second = atoi(compTime + 6);

    time_t t = makeTime(tm);
    return t + FUDGE;        //add fudge factor to allow for compile time
}
//----------------------------------------------------------------------
void lora_send_data(void)
{
    LoRa.setPins(NSS, RST, DIO0);
    if (!LoRa.begin(868E6)) //инициализируем LoRa 868 МГц, максимальная мощность
    {
      mySerial.println("Ошибка инициализации LoRa!");
    } 
    LoRa.setTxPower(crnt_pwr);
    LoRa.setSpreadingFactor(12); 
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    LoRa.beginPacket(); 
    LoRa.write(mydata, sizeof(mydata));
    LoRa.endPacket();
    LoRa.end();
}
////----------------------------------------------------------------------
//void SD_write(void)
//{
//  if (!SD.begin(SD_CS)) mySerial.println("SD failed"); else mySerial.println("SD OK!");
//  File myFile = SD.open(namefile+".csv", FILE_WRITE);
//  if (date != current_day)
//  {
//    if (myFile)
//    {
//      myFile.print("Date time");
//      myFile.print(";");
//      myFile.print("Temp_RTC");
//      myFile.print(";");
//      myFile.print("Temp_bmp");
//      myFile.print(";");
//      myFile.print("temp_htu");
//      myFile.print(";");
//      myFile.print("temp_ds18b20");
//      myFile.print(";");
//      myFile.print("press_bmp");
//      myFile.print(";");
//      myFile.print("hum_htu");
//    } else mySerial.println("error opening");
//  }
//      current_day = date;
//      myFile.println(";");
//      myFile.print(String(date));
//      myFile.print('.');
//      myFile.print(String(Month));
//      myFile.print('.');
//      myFile.print("20");
//      myFile.print(String(Year));
//      myFile.print(' ');
//      myFile.print(String(Hour));
//      myFile.print(':');
//      myFile.print(String(Min));
//      myFile.print(";");
//      myFile.print('='+String(myRTC.temperature() / 4.0));
//      myFile.print(";");
//      myFile.print('='+String(read_bmp280_temp()));
//      myFile.print(";");
//      myFile.print('='+String(read_htu21_temp()));
//      myFile.print(";");
//      myFile.print('='+String(read_ds18b20()));
//      myFile.print(";");
//      myFile.print('='+String(read_bmp280_press()));
//      myFile.print(";");
//      myFile.print('='+String(read_htu21_hum()));
//      myFile.close();
//}
