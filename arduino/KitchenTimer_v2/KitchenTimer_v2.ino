// sudo chmod a+rw /dev/ttyACM0
#include <Arduino.h>
#include <driver/i2s_std.h> // Новый I2S API  //****
#include <SD.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>
//#include "WiFi.h"


// ****************** MAX98357 AND TOUCH start ******************
// I2S пины для MAX98357 (используем gpio_num_t)
#define I2S_BCLK GPIO_NUM_26  // BCLK
#define I2S_LRCLK GPIO_NUM_25 // WS (LRCLK)
#define I2S_DOUT GPIO_NUM_22  // DOUT
#define I2S_PORT I2S_NUM_0    // Можно попробовать I2S_NUM_1, если конфликт сохраняется

// Подключение Touchpad XPT2046
#define T_CS   33 //15   // CS (Chip Select)
#define T_CLK  27 //18   // CLK (SCK)
#define T_DIN  32 //23   // MOSI
#define T_DO   39 //19   // MISO
#define T_IRQ  36 //4    // IRQ (низкий при касании)
// ****************** MAX98357 AND TOUCH *end* ******************


#define SENSOR1_PIN 17  //TX2 Пин для первого DS18B20
#define SENSOR2_PIN 16  //RX2 Пин для второго DS18B20
#define BTN_SBROS 4 //1 // Кнопка сброс
#define BAT_PIN 34 //4  // ADC вход, контроль заряда батареи

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240
#define FONT_SIZE 3

#define COLOR_FRAME 0x7ff3  // цвет рамки выбранной плашки
#define COLOR_ACTIVE 0x03b9    // цвет активной плашки
#define COLOR_PASSIVE 0x9559   // цвет неработающей плашки
#define COLOR_KBRD 0x0548   // цвет кнопок клавиатуры
#define COLOR_TEXT1 0x84d3   // цвет текста 1
#define COLOR_BACK TFT_BLACK

#define BTN_W_L 140   //int BTN_W_L = 110; ширина левых плашек
#define BTN_W_R 150   //int BTN_W_R = 140; ширина правых плашек
#define BTNHIGH 35   //int BTNHIGH = 20; высота плашек
#define BRD 3   // ширина рамки выбранной плашки
#define BLANKLEFTMARGIN 3   //int BLANKLEFTMARGIN = 30; отступ слева
#define BLANKTOPMARGIN 2   //int BLANKTOPMARGIN = 20; отступ сверху
#define VERTICALSPACE 50   //int VERTICALSPACE = 40; вертикальынй шаг между плашками
#define KBBTNH 25   // int KBBTNH = 20; высота кнопки клавиатуры набора
#define KBBTNW 50   //int KBBTNW = 50; ширина кнопки клавиатуры набора
#define KBBTNSP 5   //int KBBTNSP = 5; зазор между кнопками клавы набора
#define KBBTNTOPMARGIN 100   //int KBBTNTOPMARGIN = 100; отступ сверху для клавы набора
#define COLUMN1KB 210   //int COLUMN1KB = 180; отступ слева для 1й колонки клавы набора
#define COLUMN2KB COLUMN1KB + KBBTNW + 5   //int COLUMN2KB = COLUMN1KB + KBBTNW + 5; отступ слева для 2й колонки клавы набора
#define COLUMN2SPACE BLANKLEFTMARGIN + BTN_W_L + 20   //int COLUMN2SPACE = BLANKLEFTMARGIN + BTN_W_L + 20;
//#define ___ ___   //

File wavFile;
struct MyTimer { int32_t segm_ml; int32_t segm_st; };

TFT_eSPI tft = TFT_eSPI();

// Инициализация объектов OneWire
OneWire oneWire1(SENSOR1_PIN);
OneWire oneWire2(SENSOR2_PIN);

// Передача объектов OneWire в библиотеку DallasTemperature
DallasTemperature sensor1(&oneWire1);
DallasTemperature sensor2(&oneWire2);


MyTimer timer_1 = {0,0};
MyTimer timer_2 = {0,0};
MyTimer timer_3 = {0,0};

int activpanel = 1;
bool sbros = false;
bool tea = false;
int added = 0;
float voltageBAT = 4.5;
int Vpercent = 50;
unsigned long counter = 0;
bool lowBattery = false;

int32_t alarmTimeInSeconds_1 = 0;
int32_t alarmTimeInSeconds_2 = 0;
int32_t alarmTimeInSeconds_3 = 0;
int32_t leftTimeInSeconds_1 = 0;
int32_t leftTimeInSeconds_2 = 0;
int32_t leftTimeInSeconds_3 = 0;
int8_t flag_timer_1 = 0;
int8_t flag_timer_2 = 0;
int8_t flag_timer_3 = 0;
int8_t soundOn = 1;
unsigned long lastRunCounter1 = 0;
unsigned long lastRunCounter03 = 0;
unsigned long lastRunCounter5 = 0;
unsigned long lastRunCounter60 = 0;
float temp1 = 20.0;
float temp2 = 20.0;
int porog1 = 95;
int porog2 = 95;

// ****************** ************************ ******************
// ****************** MAX98357 AND TOUCH start ******************
// ****************** ************************ ******************
i2s_chan_handle_t tx_handle; // Хендлер для I2S TX  для MAX98357

// Инициализация I2S с новым API для MAX98357
void i2sInit() {
  // Конфигурация канала
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT, I2S_ROLE_MASTER);
  chan_cfg.auto_clear = true;

  // Создаём TX-канал
  if (esp_err_t err = i2s_new_channel(&chan_cfg, &tx_handle, NULL); err != ESP_OK) {
    Serial.printf("Failed to create I2S channel: %d\n", err);
    return;
  }

  // Конфигурация стандартного режима (44.1 кГц, 16-бит, стерео, Philips)
  i2s_std_config_t std_cfg = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(44100),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED, // Не нужен для MAX98357
      .bclk = I2S_BCLK,        // GPIO_NUM_26
      .ws = I2S_LRCLK,         // GPIO_NUM_25
      .dout = I2S_DOUT,        // GPIO_NUM_22
      .din = I2S_GPIO_UNUSED,
      .invert_flags = {0},
    },
  };

  // Инициализация канала
  if (esp_err_t err = i2s_channel_init_std_mode(tx_handle, &std_cfg); err != ESP_OK) {
    Serial.printf("Failed to init I2S std mode: %d\n", err);
    return;
  }

  // Включение канала
  if (esp_err_t err = i2s_channel_enable(tx_handle); err != ESP_OK) {
    Serial.printf("Failed to enable I2S channel: %d\n", err);
    return;
  }

  Serial.println("I2S initialized successfully");
}

void spiWrite(uint8_t data) {
  for (int i = 7; i >= 0; i--) {
    digitalWrite(T_CLK, LOW);
    digitalWrite(T_DIN, (data >> i) & 0x01);
    digitalWrite(T_CLK, HIGH);
  }
}

uint16_t spiRead12bit() {
  uint16_t value = 0;
  for (int i = 0; i < 12; i++) {
    digitalWrite(T_CLK, HIGH);
    value <<= 1;
    if (digitalRead(T_DO)) value |= 1;
    digitalWrite(T_CLK, LOW);
  }
  return value;
}

uint16_t readAxis(uint8_t command) {
  digitalWrite(T_CS, LOW);
  spiWrite(command);
  uint16_t val = spiRead12bit();
  digitalWrite(T_CS, HIGH);
  return val;
}

void initTouch() {
  pinMode(T_CS, OUTPUT);
  pinMode(T_CLK, OUTPUT);
  pinMode(T_DIN, OUTPUT);
  pinMode(T_DO, INPUT);
  pinMode(T_IRQ, INPUT);

  digitalWrite(T_CS, HIGH);
  digitalWrite(T_CLK, LOW);
}

// ****************** ************************ ******************
// ****************** MAX98357 AND TOUCH *end* ******************
// ****************** ************************ ******************


void printToSerial(int add) {
  Serial.print("add = ");
  Serial.print(add);
  Serial.print(" | activpanel = ");
  Serial.print(activpanel);
  Serial.print(" | sbros = ");
  Serial.print(sbros);
  Serial.println();
}

bool cursorinrect(int x_cur, int y_cur, int x_rect, int y_rect, int w, int h) {
  return (x_cur >= x_rect && x_cur <= x_rect + w && y_cur >= y_rect && y_cur <= y_rect + h);
}

void checktouch(int x, int y) {
  int add = 0;
  int corrected2 = BLANKTOPMARGIN + 10; // ошибка смещения тача, он не откалиброван
  int corrected = KBBTNTOPMARGIN + 8; // ошибка смещения тача, он не откалиброван

  // Panel t1
  if (cursorinrect(x, y, BLANKLEFTMARGIN, corrected2, BTN_W_L, BTNHIGH)) activpanel = 1;
  // Panel t2
  if (cursorinrect(x, y, BLANKLEFTMARGIN, corrected2 + VERTICALSPACE, BTN_W_L, BTNHIGH)) activpanel = 2;
  // Panel t3
  if (cursorinrect(x, y, BLANKLEFTMARGIN, corrected2 + VERTICALSPACE*2, BTN_W_L, BTNHIGH)) activpanel = 3;
  // Panel T1
  if (cursorinrect(x, y, COLUMN2SPACE, corrected2, BTN_W_R, BTNHIGH)) activpanel = 4;
  // Panel T2
  if (cursorinrect(x, y, COLUMN2SPACE, corrected2 + VERTICALSPACE, BTN_W_R, BTNHIGH)) activpanel = 5;

  // Keyboard key [-1]      if cursorinrect(x, y,     ) add-=1;
    if (cursorinrect(x, y, COLUMN1KB, corrected, KBBTNW, KBBTNH)) add-=60;
  // Keyboard key [-5]
  if (cursorinrect(x, y, COLUMN1KB, corrected + (KBBTNH + KBBTNSP)*1, KBBTNW, KBBTNH)) add-=300;
  // Keyboard key [-20]
  if (cursorinrect(x, y, COLUMN1KB, corrected + (KBBTNH + KBBTNSP)*2, KBBTNW, KBBTNH)) add-=1200;
  // Keyboard key [-60]
  if (cursorinrect(x, y, COLUMN1KB, corrected + (KBBTNH + KBBTNSP)*3, KBBTNW, KBBTNH)) add-=3600;
  // Keyboard key [+1]
  if (cursorinrect(x, y, COLUMN2KB, corrected, KBBTNW, KBBTNH)) add+=60;
  // Keyboard key [+5]
  if (cursorinrect(x, y, COLUMN2KB, corrected + (KBBTNH + KBBTNSP)*1, KBBTNW, KBBTNH)) add+=300;
  // Keyboard key [+20]
  if (cursorinrect(x, y, COLUMN2KB, corrected + (KBBTNH + KBBTNSP)*2, KBBTNW, KBBTNH)) add+=1200;
  // Keyboard key [+60]
  if (cursorinrect(x, y, COLUMN2KB, corrected + (KBBTNH + KBBTNSP)*3, KBBTNW, KBBTNH)) add+=3600;

  // SBROS
  if (cursorinrect(x, y, BLANKLEFTMARGIN, BLANKTOPMARGIN + VERTICALSPACE*3 + 10, BTN_W_L, 50)) sbros = true;

  // чай
  if (cursorinrect(x, y, 160, 100, 45, 45)) tea = true;

  printToSerial(add);
  
  added = add;
}

void checkTouchCoord() {
  added = 0;
  if (digitalRead(T_IRQ) == LOW) {   // есть касание
    uint16_t x = readAxis(0x90) >> 3;     // команда для X
    uint16_t y = readAxis(0xD0) >> 3;     // команда для Y
    // немного нормализуем
    x = x*2.66;
    y = y*2;

    Serial.print("X=");
    Serial.print(x);
    Serial.print(" Y=");
    Serial.println(y);

    checktouch(x, y);
  }
}

void drawscreen() {
  // Clear TFT screen
  //tft.fillScreen(COLOR_BACK);
  //tft.setTextColor(TFT_BLACK, TFT_WHITE);

  // Panel t1
  tft.fillRect(BLANKLEFTMARGIN-BRD, BLANKTOPMARGIN-BRD, BTN_W_L+2*BRD, BTNHIGH+2*BRD, (activpanel==1)?COLOR_FRAME:COLOR_PASSIVE);
  tft.fillRect(BLANKLEFTMARGIN, BLANKTOPMARGIN, BTN_W_L, BTNHIGH, (leftTimeInSeconds_1>0)?COLOR_ACTIVE:COLOR_PASSIVE);
  tft.setCursor(BLANKLEFTMARGIN+17, BLANKTOPMARGIN+6);
  tft.setTextColor( (leftTimeInSeconds_1>0)?TFT_WHITE:COLOR_TEXT1, (leftTimeInSeconds_1>0)?COLOR_ACTIVE:COLOR_PASSIVE);
  tft.setTextSize(FONT_SIZE);
  String txt = " " + String(timer_1.segm_st) + ":" + String(timer_1.segm_ml);
  tft.print(txt); // "t1=00:00"

  // Panel t2
  tft.fillRect(BLANKLEFTMARGIN-BRD, BLANKTOPMARGIN + VERTICALSPACE-BRD, BTN_W_L+2*BRD, BTNHIGH+2*BRD, (activpanel==2)?COLOR_FRAME:COLOR_PASSIVE);
  tft.fillRect(BLANKLEFTMARGIN, BLANKTOPMARGIN + VERTICALSPACE, BTN_W_L, BTNHIGH, (leftTimeInSeconds_2>0)?COLOR_ACTIVE:COLOR_PASSIVE);
  tft.setCursor(BLANKLEFTMARGIN+17, BLANKTOPMARGIN + VERTICALSPACE + 6);
  tft.setTextColor( (leftTimeInSeconds_2>0)?TFT_WHITE:COLOR_TEXT1, (leftTimeInSeconds_2>0)?COLOR_ACTIVE:COLOR_PASSIVE);
  tft.setTextSize(FONT_SIZE);
  txt = " " + String(timer_2.segm_st) + ":" + String(timer_2.segm_ml);
  tft.print(txt);

  // Panel t3
  tft.fillRect(BLANKLEFTMARGIN-BRD, BLANKTOPMARGIN + VERTICALSPACE*2-BRD, BTN_W_L+2*BRD, BTNHIGH+2*BRD, (activpanel==3)?COLOR_FRAME:COLOR_PASSIVE);
  tft.fillRect(BLANKLEFTMARGIN, BLANKTOPMARGIN + VERTICALSPACE*2, BTN_W_L, BTNHIGH, (leftTimeInSeconds_3>0)?COLOR_ACTIVE:COLOR_PASSIVE);
  tft.setCursor(BLANKLEFTMARGIN+17, BLANKTOPMARGIN + VERTICALSPACE*2 + 6);
  tft.setTextColor( (leftTimeInSeconds_3>0)?TFT_WHITE:COLOR_TEXT1, (leftTimeInSeconds_3>0)?COLOR_ACTIVE:COLOR_PASSIVE);
  tft.setTextSize(FONT_SIZE);
  txt = " " + String(timer_3.segm_st) + ":" + String(timer_3.segm_ml);
  tft.print(txt);

  
  // Panel T1
  tft.fillRect(COLUMN2SPACE-BRD, BLANKTOPMARGIN-BRD, BTN_W_R+2*BRD, BTNHIGH+2*BRD, (activpanel==4)?COLOR_FRAME:COLOR_PASSIVE);
  tft.fillRect(COLUMN2SPACE, BLANKTOPMARGIN, BTN_W_R, BTNHIGH, COLOR_ACTIVE);
  tft.setCursor(COLUMN2SPACE+7, BLANKTOPMARGIN + 6);
  tft.setTextColor(TFT_WHITE, COLOR_ACTIVE);
  tft.setTextSize(FONT_SIZE);
  txt = " " + String(int(temp1)) + "->" + String(porog1); 
  tft.print(txt);

  // Panel T2
  tft.fillRect(COLUMN2SPACE-BRD, BLANKTOPMARGIN + VERTICALSPACE-BRD, BTN_W_R+2*BRD, BTNHIGH+2*BRD, (activpanel==5)?COLOR_FRAME:COLOR_PASSIVE);
  tft.fillRect(COLUMN2SPACE, BLANKTOPMARGIN + VERTICALSPACE, BTN_W_R, BTNHIGH, COLOR_ACTIVE);
  tft.setCursor(COLUMN2SPACE+7, BLANKTOPMARGIN + VERTICALSPACE + 6);
  tft.setTextColor(TFT_WHITE, COLOR_ACTIVE);
  tft.setTextSize(FONT_SIZE);
  txt = " " + String(int(temp2)) + "->" + String(porog2);
  tft.print(txt);

  // Time
  tft.setCursor(5, 225);
  tft.setTextColor(COLOR_ACTIVE, COLOR_BACK);
  tft.setTextSize(2);
  txt = "10:00";
  tft.print(txt);
  // Battery
  tft.setCursor(220, 225);
  if(lowBattery) tft.setTextColor(TFT_RED, COLOR_BACK);
  else tft.setTextColor(COLOR_ACTIVE, COLOR_BACK);
  tft.setTextSize(2);
  txt = "Bat " + String(Vpercent) + "%  "; // + String(voltageBAT) + " "
  tft.print(txt);
}

void drawCupIcon(int16_t x, int16_t y) {
  // Цвета (16-bit 565)
  uint16_t blue = tft.color565(0, 102, 204);  // фон
  uint16_t white = TFT_WHITE;
  uint16_t darkerBlue = tft.color565(0, 70, 150); // тень внутри чашки

  // --- фон ---
  tft.fillRect(x, y, 45, 45, blue);

  // --- чашка ---
  // ободок
  tft.fillRect(x + 9, y + 9, 24, 4, white);

  // тело чашки
  tft.fillRoundRect(x + 9, y + 13, 24, 18, 5, white);

  // внутренняя тень (показывает, что чашка "пуста")
  tft.fillRect(x + 11, y + 11, 20, 2, darkerBlue);

  // подставка (основание)
  tft.fillRect(x + 15, y + 32, 14, 3, white);

  // ручка — толстая дуга справа
  tft.fillRoundRect(x + 31, y + 14, 8, 16, 4, white);
  // вырезаем внутреннюю часть ручки
  tft.fillRoundRect(x + 33, y + 16, 4, 12, 2, blue);

  // (опционально) тонкая рамка белым вокруг тела — это можно закомментировать
  // tft.drawRoundRect(x + 3, y + 5, 8, 6, 2, white);
}

void drawkeys() {
    // Keyboard key [-1]
    int txtotstup_H = 7;
    int txtotstup_W = 5;
  tft.fillRect(COLUMN1KB, KBBTNTOPMARGIN, KBBTNW, KBBTNH, COLOR_KBRD);
  tft.setCursor(COLUMN1KB+txtotstup_H, KBBTNTOPMARGIN + txtotstup_W);
  tft.setTextColor(TFT_WHITE, COLOR_KBRD);
  tft.setTextSize(2);
  tft.print("- 1");
  // Keyboard key [-5]
  tft.fillRect(COLUMN1KB, KBBTNTOPMARGIN + (KBBTNH + KBBTNSP)*1, KBBTNW, KBBTNH, COLOR_KBRD);
  tft.setCursor(COLUMN1KB+txtotstup_H, KBBTNTOPMARGIN + (KBBTNH + KBBTNSP)*1 + txtotstup_W);
  tft.setTextColor(TFT_WHITE, COLOR_KBRD);
  tft.setTextSize(2);
  tft.print("- 5");
  // Keyboard key [-20]
  tft.fillRect(COLUMN1KB, KBBTNTOPMARGIN + (KBBTNH + KBBTNSP)*2, KBBTNW, KBBTNH, COLOR_KBRD);
  tft.setCursor(COLUMN1KB+txtotstup_H, KBBTNTOPMARGIN + (KBBTNH + KBBTNSP)*2 + txtotstup_W);
  tft.setTextColor(TFT_WHITE, COLOR_KBRD);
  tft.setTextSize(2);
  tft.print("-20");
  // Keyboard key [-60]
  tft.fillRect(COLUMN1KB, KBBTNTOPMARGIN + (KBBTNH + KBBTNSP)*3, KBBTNW, KBBTNH, COLOR_KBRD);
  tft.setCursor(COLUMN1KB+txtotstup_H, KBBTNTOPMARGIN + (KBBTNH + KBBTNSP)*3 + txtotstup_W);
  tft.setTextColor(TFT_WHITE, COLOR_KBRD);
  tft.setTextSize(2);
  tft.print("-60");

  // Keyboard key [+1]
  tft.fillRect(COLUMN2KB, KBBTNTOPMARGIN, KBBTNW, KBBTNH, COLOR_KBRD);
  tft.setCursor(COLUMN2KB+txtotstup_H, KBBTNTOPMARGIN + txtotstup_W);
  tft.setTextColor(TFT_WHITE, COLOR_KBRD);
  tft.setTextSize(2);
  tft.print("+ 1");
  // Keyboard key [+5]
  tft.fillRect(COLUMN2KB, KBBTNTOPMARGIN + (KBBTNH + KBBTNSP)*1, KBBTNW, KBBTNH, COLOR_KBRD);
  tft.setCursor(COLUMN2KB+txtotstup_H, KBBTNTOPMARGIN + (KBBTNH + KBBTNSP)*1 + txtotstup_W);
  tft.setTextColor(TFT_WHITE, COLOR_KBRD);
  tft.setTextSize(2);
  tft.print("+ 5");
  // Keyboard key [+20]
  tft.fillRect(COLUMN2KB, KBBTNTOPMARGIN + (KBBTNH + KBBTNSP)*2, KBBTNW, KBBTNH, COLOR_KBRD);
  tft.setCursor(COLUMN2KB+txtotstup_H, KBBTNTOPMARGIN + (KBBTNH + KBBTNSP)*2 + txtotstup_W);
  tft.setTextColor(TFT_WHITE, COLOR_KBRD);
  tft.setTextSize(2);
  tft.print("+20");
  // Keyboard key [+60]
  tft.fillRect(COLUMN2KB, KBBTNTOPMARGIN + (KBBTNH + KBBTNSP)*3, KBBTNW, KBBTNH, COLOR_KBRD);
  tft.setCursor(COLUMN2KB+txtotstup_H, KBBTNTOPMARGIN + (KBBTNH + KBBTNSP)*3 + txtotstup_W);
  tft.setTextColor(TFT_WHITE, COLOR_KBRD);
  tft.setTextSize(2);
  tft.print("+60");

  // Keyboard SBROS
  tft.fillRect(BLANKLEFTMARGIN, BLANKTOPMARGIN + VERTICALSPACE*3 + 10, BTN_W_L, 50, COLOR_KBRD);
  tft.setCursor(BLANKLEFTMARGIN+22, BLANKTOPMARGIN + VERTICALSPACE*3 + 22);
  tft.setTextColor(TFT_WHITE, COLOR_KBRD);
  tft.setTextSize(3);
  tft.print("SBROS");

  // Line
  tft.fillRect(0, 220, 320, 2, COLOR_ACTIVE);
  tft.setCursor(COLUMN2KB+5, KBBTNTOPMARGIN + (KBBTNH + KBBTNSP)*3 + 3);

  //Icons
  //drawCupIcon(100, 100);
}

MyTimer showtimeleft(int lefttime) {
  MyTimer timer_0 = {0,0};
  int32_t hours, minutes, seconds;

  // Вычисляем компоненты времени
  hours = lefttime / 3600;
  int32_t remainingSeconds = lefttime % 3600;
  minutes = remainingSeconds / 60;
  seconds = remainingSeconds % 60;

  if (hours > 0) { timer_0.segm_st = hours; timer_0.segm_ml = minutes; }
  else { timer_0.segm_st = minutes; timer_0.segm_ml = seconds; }

  return timer_0;
}

void timerBranch() {
  int32_t currentTimeInSeconds = millis()/1000; // текущее время в секундах (с включения прибора) 

  // timer_1
  if(currentTimeInSeconds < alarmTimeInSeconds_1) { flag_timer_1 = 1; }
  else { alarmTimeInSeconds_1 = currentTimeInSeconds; // обнулить разницу, вдруг целевое вермя раньше текущего (при старте контролера)
    if(flag_timer_1 && soundOn==0) { soundOn = 1; flag_timer_1 = 0; }
  }
  if(activpanel==1) alarmTimeInSeconds_1+=added;
  leftTimeInSeconds_1 = alarmTimeInSeconds_1 - currentTimeInSeconds; // осталось секунд до сигнала
  timer_1 = showtimeleft(leftTimeInSeconds_1);

  // timer_2
  if(currentTimeInSeconds < alarmTimeInSeconds_2) { flag_timer_2 = 1; }
  else { alarmTimeInSeconds_2 = currentTimeInSeconds; // обнулить разницу, вдруг целевое вермя раньше текущего (при старте контролера)
    if(flag_timer_2 && soundOn==0) { soundOn = 1; flag_timer_2 = 0; }
  }
  if(activpanel==2) alarmTimeInSeconds_2+=added;
  leftTimeInSeconds_2 = alarmTimeInSeconds_2 - currentTimeInSeconds; // осталось секунд до сигнала
  timer_2 = showtimeleft(leftTimeInSeconds_2);

  // timer_3
  if(currentTimeInSeconds < alarmTimeInSeconds_3) { flag_timer_3 = 1; }
  else { alarmTimeInSeconds_3 = currentTimeInSeconds; // обнулить разницу, вдруг целевое вермя раньше текущего (при старте контролера)
    if(flag_timer_3 && soundOn==0) { soundOn = 1; flag_timer_3 = 0; }
  }
  if(activpanel==3) alarmTimeInSeconds_3+=added;
  leftTimeInSeconds_3 = alarmTimeInSeconds_3 - currentTimeInSeconds; // осталось секунд до сигнала
  timer_3 = showtimeleft(leftTimeInSeconds_3);

  //сброс для всех таймеров или по отдельности?
  //if(нажат сброс) { soundOn = 0; alarmTimeInSeconds_1 = currentTimeInSeconds; }
  if(soundOn == 1) Serial.println("sound ON.");
}

void playsounds() {
  Serial.println("sound start.");
  int c = 0;
  while(c<1) {
    static uint8_t buffer[512];
    if(wavFile.available()) {
      int bytesRead = wavFile.read(buffer, sizeof(buffer));
      size_t bytesWritten;
      //Serial.println(bytesRead);
        //****i2s_write(I2S_NUM_0, buffer, bytesRead, &bytesWritten, portMAX_DELAY);
      i2s_channel_write(tx_handle, buffer, bytesRead, &bytesWritten, portMAX_DELAY);
    } else {
      wavFile.seek(0);  // Перемотать в начало, чтобы проигрывать циклом
      c+=1;
    }
    // проверка сброса чтобы отключить звук по среди дороги
    checkTouchCoord();
    if (btnSbrosPressed() or sbros) {
      wavFile.seek(0);
      break;
    }
  }
}

bool btnSbrosPressed() {
  if(digitalRead(BTN_SBROS)) return false;
  else return true;
}

void batteryHealth() {
  int raw = analogRead(BAT_PIN);
  for (int i = 0; i < 10; i++) {
    raw += analogRead(BAT_PIN);
    delay(2);
  }
  raw = raw/11.0;
  float voltageADC = raw * (3.3 / 4095.0); // напряжение на пине 1234;//
  voltageBAT = voltageADC * 3.3;     // учёт делителя 1:2

  float minV = 2.6;   // напряжение при 0%
  float maxV = 3.91;   // напряжение при 100%

  Vpercent = int((voltageBAT - minV) * 100.0 / (maxV - minV));

}

void setup() {
  Serial.begin(115200);
  pinMode(BTN_SBROS, INPUT);
  pinMode(BTN_SBROS, INPUT_PULLUP);

  pinMode(BAT_PIN, INPUT);

  // Инициализация датчиков
  sensor1.begin();
  sensor2.begin();

  if(!SD.begin(5)){
  Serial.println("SD mount failed");
  while(1) delay(1000);
  }
  Serial.println("SD mounted");

  i2sInit();

  wavFile = SD.open("/bell.wav");
  if(!wavFile) {
    Serial.println("Failed to open WAV");
    while(1) delay(1000);
  }
  Serial.println("The /bell.wav opened.");

  initTouch();

  // Start the tft display
  tft.init();
  // Set the TFT display rotation in landscape mode
  tft.setRotation(1);
  tft.fillScreen(COLOR_BACK);
  drawscreen();
  drawkeys();
  Serial.println("Setup complete.");
}

void temperatureBranch() {
  // Запрос температуры с датчиков
  sensor1.requestTemperatures();
  sensor2.requestTemperatures();
  
  // Чтение температуры с первого датчика
  temp1 = sensor1.getTempCByIndex(0);
  // Чтение температуры со второго датчика
  temp2 = sensor2.getTempCByIndex(0);
  
  if(int(temp1)>porog1) soundOn = 1;
  if(int(temp2)>porog2) soundOn = 1;
  if(activpanel==4) porog1+=added/60;
  if(activpanel==5) porog2+=added/60;

  // Вывод температуры в Serial
  if(false) {
    Serial.print("Температура 1: ");
    Serial.print(int(temp1));
    Serial.print(" ");
    Serial.print(porog1);
    Serial.print(" °C  ***  ");
    
    Serial.print("Температура 2: ");
    Serial.print(int(temp2));
    Serial.print(" ");
    Serial.print(porog2);
    Serial.println(" °C");
  }
}

void loop() {
  
  if(soundOn==1) playsounds();
  timerBranch();
  drawscreen();
  drawCupIcon(160, 100);
  checkTouchCoord();
  
  if(tea) {
    tea = false;
    alarmTimeInSeconds_3 = millis()/1000 + 500;
  }

  // lastRunCounter5

  // раз в секунду
  if (millis() - lastRunCounter1 >= 1000) {
    lastRunCounter1 = millis();
    batteryHealth();
    temperatureBranch();
    if(lowBattery) {
      counter += 1;
      if(counter%2==0) tft.fillRect(0, 217, 320, 5, TFT_RED);
      else tft.fillRect(0, 217, 320, 5, TFT_BLUE);
    }
  } 

  // раз в 0.3 секунды
  if (millis() - lastRunCounter03 >= 300) {
    lastRunCounter03 = millis();
    
    //
  } 

  // раз в минуту
  if (millis() - lastRunCounter60 >= 60000) {
    lastRunCounter60 = millis();
    // мигание если заряд ниже 20%
    if(Vpercent<20) lowBattery = true;
    else lowBattery = false;
  } 

  delay(100);
  if (btnSbrosPressed() or sbros) soundOn = 0;
 
  //Serial.print(".");
  sbros = false;
}







/*

  *** СОХРАНЕНКИ ***

void checktouch() {
  // Touchscreen coordinates: (x, y) and pressure (z)
  int x, y, z;
  int add = 0;
  static bool touched = false;

  if (digitalRead(T_IRQ) == LOW) {
    
    if (!touched) {
      uint16_t x = readAxis(0x90) >> 3;     // команда для X
      uint16_t y = readAxis(0xD0) >> 3;     // команда для Y
      // немного нормализуем
      x = x*2.66;
      y = y*2;

      Serial.print("X=");
      Serial.print(x);
      Serial.print(" Y=");
      Serial.println(y);
      touched = true;
    

      // Panel t1
      if (cursorinrect(x, y, BLANKLEFTMARGIN, BLANKTOPMARGIN, BTN_W_L, BTNHIGH)) activpanel = 1;
      // Panel t2
      if (cursorinrect(x, y, BLANKLEFTMARGIN, BLANKTOPMARGIN + VERTICALSPACE, BTN_W_L, BTNHIGH)) activpanel = 2;
      // Panel t3
      if (cursorinrect(x, y, BLANKLEFTMARGIN, BLANKTOPMARGIN + VERTICALSPACE*2, BTN_W_L, BTNHIGH)) activpanel = 3;
      // Panel T1
      if (cursorinrect(x, y, COLUMN2SPACE, BLANKTOPMARGIN, BTN_W_R, BTNHIGH)) activpanel = 4;
      // Panel T2
      if (cursorinrect(x, y, COLUMN2SPACE, BLANKTOPMARGIN + VERTICALSPACE, BTN_W_R, BTNHIGH)) activpanel = 5;

      // Keyboard key [-1]      if cursorinrect(x, y,     ) add-=1;
      if (cursorinrect(x, y, COLUMN1KB, KBBTNTOPMARGIN, KBBTNW, KBBTNH)) add-=60;
      // Keyboard key [-5]
      if (cursorinrect(x, y, COLUMN1KB, KBBTNTOPMARGIN + (KBBTNH + KBBTNSP)*1, KBBTNW, KBBTNH)) add-=300;
      // Keyboard key [-20]
      if (cursorinrect(x, y, COLUMN1KB, KBBTNTOPMARGIN + (KBBTNH + KBBTNSP)*2, KBBTNW, KBBTNH)) add-=1200;
      // Keyboard key [-60]
      if (cursorinrect(x, y, COLUMN1KB, KBBTNTOPMARGIN + (KBBTNH + KBBTNSP)*3, KBBTNW, KBBTNH)) add-=3600;
      // Keyboard key [+1]
      if (cursorinrect(x, y, COLUMN2KB, KBBTNTOPMARGIN, KBBTNW, KBBTNH)) add+=60;
      // Keyboard key [+5]
      if (cursorinrect(x, y, COLUMN2KB, KBBTNTOPMARGIN + (KBBTNH + KBBTNSP)*1, KBBTNW, KBBTNH)) add+=300;
      // Keyboard key [+20]
      if (cursorinrect(x, y, COLUMN2KB, KBBTNTOPMARGIN + (KBBTNH + KBBTNSP)*2, KBBTNW, KBBTNH)) add+=1200;
      // Keyboard key [+60]
      if (cursorinrect(x, y, COLUMN2KB, KBBTNTOPMARGIN + (KBBTNH + KBBTNSP)*3, KBBTNW, KBBTNH)) add+=3600;

      // SBROS
      if (cursorinrect(x, y, BLANKLEFTMARGIN, BLANKTOPMARGIN + VERTICALSPACE*3 + 3, BTN_W_L, 50)) sbros = true;

      
    }
    printToSerial(add);
    added = add;

  } else {
    touched = false;
    int add = 0;
  }

  
}

void checktouch() {
  // Touchscreen coordinates: (x, y) and pressure (z)
  int x, y, z;
  int add = 0;

  if (touchscreen.tirqTouched() && touchscreen.touched()) {
    // Get Touchscreen points
    TS_Point p = touchscreen.getPoint();
    // Calibrate Touchscreen points with map function to the correct width and height
    x = map(p.x, 200, 3700, 1, SCREEN_WIDTH);
    y = map(p.y, 240, 3800, 1, SCREEN_HEIGHT);
    z = p.z;

    // Panel t1
    if (cursorinrect(x, y, BLANKLEFTMARGIN, BLANKTOPMARGIN, BTN_W_L, BTNHIGH)) activpanel = 1;
    // Panel t2
    if (cursorinrect(x, y, BLANKLEFTMARGIN, BLANKTOPMARGIN + VERTICALSPACE, BTN_W_L, BTNHIGH)) activpanel = 2;
    // Panel t3
    if (cursorinrect(x, y, BLANKLEFTMARGIN, BLANKTOPMARGIN + VERTICALSPACE*2, BTN_W_L, BTNHIGH)) activpanel = 3;
    // Panel T1
    if (cursorinrect(x, y, COLUMN2SPACE, BLANKTOPMARGIN, BTN_W_R, BTNHIGH)) activpanel = 4;
    // Panel T2
    if (cursorinrect(x, y, COLUMN2SPACE, BLANKTOPMARGIN + VERTICALSPACE, BTN_W_R, BTNHIGH)) activpanel = 5;

    // Keyboard key [-1]      if cursorinrect(x, y,     ) add-=1;
    if (cursorinrect(x, y, COLUMN1KB, KBBTNTOPMARGIN, KBBTNW, KBBTNH)) add-=60;
    // Keyboard key [-5]
    if (cursorinrect(x, y, COLUMN1KB, KBBTNTOPMARGIN + (KBBTNH + KBBTNSP)*1, KBBTNW, KBBTNH)) add-=300;
    // Keyboard key [-20]
    if (cursorinrect(x, y, COLUMN1KB, KBBTNTOPMARGIN + (KBBTNH + KBBTNSP)*2, KBBTNW, KBBTNH)) add-=1200;
    // Keyboard key [-60]
    if (cursorinrect(x, y, COLUMN1KB, KBBTNTOPMARGIN + (KBBTNH + KBBTNSP)*3, KBBTNW, KBBTNH)) add-=3600;
    // Keyboard key [+1]
    if (cursorinrect(x, y, COLUMN2KB, KBBTNTOPMARGIN, KBBTNW, KBBTNH)) add+=60;
    // Keyboard key [+5]
    if (cursorinrect(x, y, COLUMN2KB, KBBTNTOPMARGIN + (KBBTNH + KBBTNSP)*1, KBBTNW, KBBTNH)) add+=300;
    // Keyboard key [+20]
    if (cursorinrect(x, y, COLUMN2KB, KBBTNTOPMARGIN + (KBBTNH + KBBTNSP)*2, KBBTNW, KBBTNH)) add+=1200;
    // Keyboard key [+60]
    if (cursorinrect(x, y, COLUMN2KB, KBBTNTOPMARGIN + (KBBTNH + KBBTNSP)*3, KBBTNW, KBBTNH)) add+=3600;

    // SBROS
    if (cursorinrect(x, y, BLANKLEFTMARGIN, BLANKTOPMARGIN + VERTICALSPACE*3 + 3, BTN_W_L, 50)) sbros = true;

    printToSerial(add);
  }

  added = add;
}
*/