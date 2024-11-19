/********************************************************
 * Binary Clock
 * Copyright Tauno Erik 2024
 * Stared 15.11.2024
 * Edited 16
 * 19.11.2024
 ********************************************************/
#include <Arduino.h>
#include <ESP8266WiFi.h>         // https://github.com/esp8266/Arduino
#include <ESPAsyncWebServer.h>   // https://github.com/me-no-dev/ESPAsyncWebServer
#include <ESPAsyncWiFiManager.h> // https://github.com/tzapu/WiFiManager WiFi Configuration Magic

#define SECOND 1000 // ms
#define DATA_BOTTOM 0
#define DATA_MIDLE 1
#define DATA_TOP 2

const uint8_t DATA_PIN = 4;    // GPIO4 - D2
const uint8_t CLOCK_PIN = 5;   // GPIO5 - D1
const uint8_t LATCH_PIN = 14;  // GPIO14 - D5

uint8_t data[3] = {
  0b00000000,  // bottom
  0b00000000,  // midle
  0b00000000,  // top
};

uint32_t bottom_last_time = 0;
uint32_t midle_last_time = 0;
uint32_t top_last_time = 0;
bool data_changed = false;

AsyncWebServer server(80);
DNSServer dns;

/********************************************************
 **  Function declarations:
 ********************************************************/

void write_to_shift_register(int data_pin, int clock_pin, int latch_pin, uint8_t data);
void write_to_shift_register_fast_esp8266(int data_pin, int clock_pin, int latch_pin, uint8_t data);
void update_data(int data_position);
void write_data(int data_pin, int clock_pin, int latch_pin, uint32_t data);
uint32_t count_data();
void init_lights();
void binary_counter();
void kuusepuu();

void setup()
{
  pinMode(DATA_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);

  Serial.begin(115200);

  // first parameter is name of access point, second is the password
  AsyncWiFiManager wifiManager(&server, &dns);

  wifiManager.autoConnect("CLOCK", "tauno");

  // all OFF
  write_to_shift_register_fast_esp8266(DATA_PIN, CLOCK_PIN, LATCH_PIN, 0);  // top
  write_to_shift_register_fast_esp8266(DATA_PIN, CLOCK_PIN, LATCH_PIN, 0);  // midle
  write_to_shift_register_fast_esp8266(DATA_PIN, CLOCK_PIN, LATCH_PIN, 0);  // bottom
  delay(SECOND);

  init_lights();
}


void loop()
{
  // binary_counter();
  kuusepuu();

  /*
  if (now - bottom_last_time >= (SECOND / 2))
  {
    bottom_last_time = now;
    update_data(DATA_BOTTOM);
    data_changed = true;
  }

  if (now - midle_last_time >= (SECOND))
  {
    midle_last_time = now;
    update_data(DATA_MIDLE);
    data_changed = true;
  }

  if (now - top_last_time >= (SECOND * 2))
  {
    top_last_time = now;
    update_data(DATA_TOP);
    data_changed = true;
  }

  if (data_changed)
  {
    write_to_shift_register_fast_esp8266(DATA_PIN, CLOCK_PIN, LATCH_PIN, data[DATA_TOP]);
    write_to_shift_register_fast_esp8266(DATA_PIN, CLOCK_PIN, LATCH_PIN, data[DATA_MIDLE]);
    write_to_shift_register_fast_esp8266(DATA_PIN, CLOCK_PIN, LATCH_PIN, data[DATA_BOTTOM]);

    data_changed = false;
  }
  */
}

/********************************************************
 ** Function definitions:
 ********************************************************/


/********************************************************
 ** Arduino func
 ********************************************************/
void write_to_shift_register(int data_pin, int clock_pin, int latch_pin, uint8_t data)
{
  digitalWrite(latch_pin, LOW);
  shiftOut(data_pin, clock_pin, MSBFIRST, data);
  digitalWrite(latch_pin, HIGH);
}


/********************************************************
 ** ESP8266 fast func
 ** 8-bit data
 ********************************************************/
void write_to_shift_register_fast_esp8266(int data_pin, int clock_pin, int latch_pin, uint8_t data)
{
  // Precompute bitmasks for the GPIO pins
  uint32_t data_mask = (1U << data_pin);
  uint32_t clock_mask = (1U << clock_pin);
  uint32_t latch_mask = (1U << latch_pin);

  // Set latch LOW
  GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, latch_mask);  // Clear latch pin (LOW)

  // Send bits (MSB first)
  for (int i = 7; i >= 0; i--)
  {
    // Set data pin
    if (data & (1 << i))
    {
      GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, data_mask);  // Set data pin HIGH
    }
    else
    {
      GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, data_mask);  // Set data pin LOW
    }

    // Pulse clock pin
    GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, clock_mask);  // Set clock pin HIGH
    GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, clock_mask);  // Set clock pin LOW
  }

  // Set latch HIGH
  GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, latch_mask);  // Set latch pin HIGH
}


/********************************************************
 ** 8-bit data
 ********************************************************/
void update_data(int data_position)
{
  data[data_position]++;

  if (data[data_position] == 0)
  {
    data[data_position] = 0b00000001;
  }
}


/********************************************************
 ** 24-bit data
 ********************************************************/
uint32_t count_data() {
  static uint32_t counter = 0;

  // Increment the counter (modulo 2^24 to avoid overflow)
  counter = (counter + 1) % 16777216; // 2^24 = 16777216

  return counter;
}


/********************************************************
 ** ESP8266 fast func
 ** Input 32-bit data
 ********************************************************/
void write_data(int data_pin, int clock_pin, int latch_pin, uint32_t data)
{
  // Precompute bitmasks for the GPIO pins
  uint32_t data_mask = (1U << data_pin);
  uint32_t clock_mask = (1U << clock_pin);
  uint32_t latch_mask = (1U << latch_pin);

  // Set latch LOW
  GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, latch_mask);  // Clear latch pin (LOW)

  // Send bits (MSB first)
  for (int i = 23; i >= 0; i--)
  {
    // Set data pin
    if (data & (1 << i))
    {
      GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, data_mask);  // Set data pin HIGH
    }
    else
    {
      GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, data_mask);  // Set data pin LOW
    }

    // Pulse clock pin
    GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, clock_mask);  // Set clock pin HIGH
    GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, clock_mask);  // Set clock pin LOW
  }

  // Set latch HIGH
  GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, latch_mask);  // Set latch pin HIGH
}


void init_lights() {
  uint8_t d = 0;
  // top up
  for (int i = 0; i <= 8; i++)
  {
    write_to_shift_register_fast_esp8266(DATA_PIN, CLOCK_PIN, LATCH_PIN, d);  // top
    write_to_shift_register_fast_esp8266(DATA_PIN, CLOCK_PIN, LATCH_PIN, 0);  // midle
    write_to_shift_register_fast_esp8266(DATA_PIN, CLOCK_PIN, LATCH_PIN, 0);  // bottom
    delay(SECOND/4);
    d |= (1 << i);
  }
  // top on
  d = 0b11111111;
  write_to_shift_register_fast_esp8266(DATA_PIN, CLOCK_PIN, LATCH_PIN, d);  // top
  write_to_shift_register_fast_esp8266(DATA_PIN, CLOCK_PIN, LATCH_PIN, 0);  // midle
  write_to_shift_register_fast_esp8266(DATA_PIN, CLOCK_PIN, LATCH_PIN, 0);  // bottom
  // top down
  for (int i = 8; i >= 0; i--)
  {
    write_to_shift_register_fast_esp8266(DATA_PIN, CLOCK_PIN, LATCH_PIN, d);  // top
    write_to_shift_register_fast_esp8266(DATA_PIN, CLOCK_PIN, LATCH_PIN, 0);  // midle
    write_to_shift_register_fast_esp8266(DATA_PIN, CLOCK_PIN, LATCH_PIN, 0);  // bottom
    delay(SECOND/4);
    d &= ~(1 << i);
  }

  // middle up
  d = 0;
  for (int i = 0; i <= 8; i++)
  {
    write_to_shift_register_fast_esp8266(DATA_PIN, CLOCK_PIN, LATCH_PIN, 0);  // top
    write_to_shift_register_fast_esp8266(DATA_PIN, CLOCK_PIN, LATCH_PIN, d);  // middle
    write_to_shift_register_fast_esp8266(DATA_PIN, CLOCK_PIN, LATCH_PIN, 0);  // bottom
    delay(SECOND/4);
    d |= (1 << i);
  }
  // middle on
  d = 0b11111111;
  write_to_shift_register_fast_esp8266(DATA_PIN, CLOCK_PIN, LATCH_PIN, 0);  // top
  write_to_shift_register_fast_esp8266(DATA_PIN, CLOCK_PIN, LATCH_PIN, d);  // middle
  write_to_shift_register_fast_esp8266(DATA_PIN, CLOCK_PIN, LATCH_PIN, 0);  // bottom
  // middle down
  for (int i = 8; i >= 0; i--)
  {
    write_to_shift_register_fast_esp8266(DATA_PIN, CLOCK_PIN, LATCH_PIN, 0);  // top
    write_to_shift_register_fast_esp8266(DATA_PIN, CLOCK_PIN, LATCH_PIN, d);  // middle
    write_to_shift_register_fast_esp8266(DATA_PIN, CLOCK_PIN, LATCH_PIN, 0);  // bottom
    delay(SECOND/4);
    d &= ~(1 << i);
  }


  // bottom up
  d = 0;
  for (int i = 0; i <= 8; i++)
  {
    write_to_shift_register_fast_esp8266(DATA_PIN, CLOCK_PIN, LATCH_PIN, 0);  // top
    write_to_shift_register_fast_esp8266(DATA_PIN, CLOCK_PIN, LATCH_PIN, 0);  // midle
    write_to_shift_register_fast_esp8266(DATA_PIN, CLOCK_PIN, LATCH_PIN, d);  // bottom
    delay(SECOND/4);
    d |= (1 << i);
  }
  // bottom on
  d = 0b11111111;
  write_to_shift_register_fast_esp8266(DATA_PIN, CLOCK_PIN, LATCH_PIN, 0);  // top
  write_to_shift_register_fast_esp8266(DATA_PIN, CLOCK_PIN, LATCH_PIN, 0);  // middle
  write_to_shift_register_fast_esp8266(DATA_PIN, CLOCK_PIN, LATCH_PIN, d);  // bottom
  // bottom down
  for (int i = 8; i >= 0; i--)
  {
    write_to_shift_register_fast_esp8266(DATA_PIN, CLOCK_PIN, LATCH_PIN, 0);  // top
    write_to_shift_register_fast_esp8266(DATA_PIN, CLOCK_PIN, LATCH_PIN, 0);  // middle
    write_to_shift_register_fast_esp8266(DATA_PIN, CLOCK_PIN, LATCH_PIN, d);  // bottom
    delay(SECOND/4);
    d &= ~(1 << i);
  }
}

void binary_counter()
{
  uint32_t now = millis();
  static uint32_t last_time = 0;

  if (now - last_time >= SECOND/10)
  {
    last_time = now;
    uint32 data = count_data();

    // Serial.println(data);
    write_data(DATA_PIN, CLOCK_PIN, LATCH_PIN, data);
    // data_changed = true;
  }
}

void kuusepuu()
{
  uint32_t now = millis();
  static uint32_t last_time = 0;
  static uint32_t kuusk = 0b101010100101010110101010;

  if (now - last_time >= SECOND/2)
  {
    last_time = now;
    uint32 data = count_data();
    // Serial.println(data);
    write_data(DATA_PIN, CLOCK_PIN, LATCH_PIN, kuusk);
    kuusk = ~(kuusk);
  }
}
