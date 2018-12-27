#include <EEPROM.h>

#include "WireSlave.h"
#include "PMW3901.h"
#include "CCITTCrc16.h"
#include "stm32l4_wiring_private.h"
#include <limits.h>

#define  SLAVE_ADDRESS            0x29  //slave address,any number from 0x01 to 0x7F

#define PRODUCT_ID_CMD            0x00
#define REVISION_ID_CMD           0x01

#define CP_PRODUCT_ID_CMD         0x20
#define CP_REVISION_ID_CMD        0x21
#define SPI_PASSTHRU_CMD          0x22
#define LEDSTATUS_CMD             0x23
#define READ_RELATIVE_CMD         0x24
#define REREAD_RELATIVE_CMD       0x25
#define ZERO_ABSOLUTE_CMD         0x26
#define READ_ABSOLUTE_CMD         0x27
#define RESET_CMD                 0x28
#define CHANGE_I2C_ADDRESS_CMD    0x29

#define CP_PRODUCT_ID             0x42
#define CP_REVISION_ID            0x00


#define RED_LED           17
#define GREEN_LED         38
#define BLUE_LED          26
#define WHITE_LED         5

// External LED control pin (active low)
#define PMW_LED           3
#define J3_OUTPUT1        5
#define J3_OUTPUT2        15
#define BUTTON            39

// Motion interrupt (active low)
#define MOTION_INTERRUPT  30

#define LED_ON    LOW
#define LED_OFF   HIGH


#define STATE_WAITING_FOR_REGISTER      1
#define STATE_WAITING_FOR_DATA          2
#define STATE_WAITING_FOR_READ          3
#define STATE_WAITING_FOR_READ_OR_DATA  4

int i2cState = STATE_WAITING_FOR_REGISTER;

/********* Global  Variables  ***********/

struct Config {
  uint16_t  slaveAddress;
  uint8_t   debugOn;
};

struct EEPROM_Config {
  Config settings;
  uint16_t crc;
};

uint8_t productId;
uint8_t revisionId;

PMW3901 sensor(10);
CCITTCrc16 crcCalculator;

Config settings;


void defaultConfig() {
  settings.slaveAddress = SLAVE_ADDRESS;
  settings.debugOn = 0x00;
}

void loadConfig() {
  EEPROM_Config config;
  EEPROM.get(0, config);

  uint16_t crc = crcCalculator.calculate((char *)(&config.settings), sizeof(Config));

  if (crc != config.crc) {
    Serial.print("Saved settings invalid - using defaults ");
    Serial.print(config.crc);
    Serial.print(" <> ");
    Serial.println(crc);
    defaultConfig();
  } else {
    settings = config.settings;
  }
}

void saveConfig() {
  EEPROM_Config oldConfig;
  EEPROM_Config config;
  config.settings = settings;
  config.crc = crcCalculator.calculate((char *)(&config.settings), sizeof(Config));

  EEPROM.get(0, oldConfig);
  if (memcmp(&oldConfig, &config, sizeof(config) != 0)) {
    EEPROM.put(0, config);
  }
}

String inputBuffer = "";
bool inputComplete = false;

void setup()
{
  pinMode(BUTTON, INPUT);

  // Set the pins to output mode
  pinMode(J3_OUTPUT1, OUTPUT);
  digitalWrite(J3_OUTPUT1, LOW);
  pinMode(J3_OUTPUT2, OUTPUT);
  digitalWrite(J3_OUTPUT2, HIGH);
  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED, LED_OFF);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(GREEN_LED, LED_OFF);
  pinMode(BLUE_LED, OUTPUT);
  digitalWrite(BLUE_LED, LED_OFF);
  pinMode(WHITE_LED, OUTPUT);
  digitalWrite(WHITE_LED, LED_OFF);

  inputBuffer.reserve(200);
  
  // Set up the serial communications over the USB connector
  Serial.begin(9600);

  loadConfig();

  delay(1000);

  Serial.println("Initalizing");

  delay(1000);

  if (digitalRead(BUTTON) == HIGH) {
    settings.slaveAddress = SLAVE_ADDRESS;
    settings.debugOn = 0x00;
    Serial.print("Resetting config slave address ");
    Serial.println(settings.slaveAddress, HEX);
    saveConfig();
    for (int i = 0; i < 10; i++) {
      digitalWrite(BLUE_LED, LED_OFF);
      digitalWrite(RED_LED, LED_ON);
      delay(50);
      digitalWrite(RED_LED, LED_OFF);
      digitalWrite(GREEN_LED, LED_ON);
      delay(50);
      digitalWrite(GREEN_LED, LED_OFF);
      digitalWrite(BLUE_LED, LED_ON);
      delay(50);
    }
    digitalWrite(BLUE_LED, LED_OFF);
  }
  
  // Set up the I2C communication callbacks
  WireSlave.onOverrideHandler(ll_callback);
  WireSlave.onRequest(requestEvent);
  WireSlave.onReceive(receiveEvent);

  Serial.print("Starting slave listen on ");
  Serial.println(settings.slaveAddress, HEX);

  // Start listening as a slave
  WireSlave.begin(settings.slaveAddress);

  Serial.println("Starting sensor");
  // start up processing for the optical flow chip
  sensor.begin();
  Serial.println("Reading product id");
  productId = sensor.registerRead(PRODUCT_ID_CMD);
  Serial.println("Reading revision id");
  revisionId = sensor.registerRead(REVISION_ID_CMD);

  // write out some diagnostics
  Serial.print("Init completed productId = ");
  Serial.print(productId, HEX);
  Serial.print(" revisionId = ");
  Serial.println(revisionId, HEX);
}

uint8_t lastValue;
bool lastValueSet = false;
uint16_t heartbeat = 0;
bool ledOverwritten = false;

typedef struct __attribute__((packed)) _RelativeRead
{
  uint32_t timestamp;
  int16_t xOffset;
  int16_t yOffset;
  uint16_t crc;
  uint8_t overflow;

  void incrementOverflowX()
  {
    uint8_t value = overflow & 0x0f;
    value++;
    overflow = (overflow & 0xf0) | (value & 0x0f);
  }

  void incrementOverflowY()
  {
    uint8_t value = (overflow & 0xf0) >> 4;
    value++;
    overflow = (overflow & 0x0f) | ((value & 0xf0) << 4);
  }

  void calcCrc()
  {
    char * me = (char *)this;
    crc = crcCalculator.calculate(me, sizeof(uint32_t) + sizeof(int16_t) + sizeof(int16_t) + sizeof(uint8_t));
  }
} RelativeRead;

typedef struct __attribute__((packed)) _AbsoluteRead
{
  uint32_t timestamp;
  int32_t x;
  int32_t y;
  uint16_t crc;

  void calcCrc()
  {
    char * me = (char *)this;
    crc = crcCalculator.calculate(me, sizeof(uint32_t) + sizeof(int32_t) + sizeof(int32_t));
  }

  void reset() {
    timestamp = x = y = crc = 0;
  }
} AbsoluteRead;

RelativeRead last = { 0 };
RelativeRead cached = { 0 };

AbsoluteRead absolute = { 0 };

uint8_t ledStatus = 0;

int32_t callbackCount = 0;
int32_t requestCount = 0;
int32_t receiveCount = 0;
int32_t readyCount = 0;
int32_t addressMatchSlaveTransmitCount = 0;
int32_t addressMatchSlaveReceiveCount = 0;

void loop() {
  if (lastValueSet)
  {
    Serial.print("    ");
    Serial.println(lastValue, HEX);
    lastValueSet = false;
  }
  heartbeat++;


  if (!ledOverwritten)
  {
    switch (heartbeat)
    {
      case 100:
        digitalWrite(RED_LED, LED_ON);
        digitalWrite(GREEN_LED, LED_OFF);
        digitalWrite(BLUE_LED, LED_OFF);
        break;
      case 200:
        digitalWrite(RED_LED, LED_ON);
        digitalWrite(GREEN_LED, LED_ON);
        digitalWrite(BLUE_LED, LED_OFF);
        break;
      case 300:
        digitalWrite(RED_LED, LED_OFF);
        digitalWrite(GREEN_LED, LED_ON);
        digitalWrite(BLUE_LED, LED_OFF);
        break;
      case 400:
        digitalWrite(RED_LED, LED_OFF);
        digitalWrite(GREEN_LED, LED_ON);
        digitalWrite(BLUE_LED, LED_ON);
        break;
      case 500:
        digitalWrite(RED_LED, LED_OFF);
        digitalWrite(GREEN_LED, LED_OFF);
        digitalWrite(BLUE_LED, LED_ON);
        break;
      case 600:
        digitalWrite(RED_LED, LED_ON);
        digitalWrite(GREEN_LED, LED_OFF);
        digitalWrite(BLUE_LED, LED_ON);
        break;
      case 700:
        digitalWrite(RED_LED, LED_ON);
        digitalWrite(GREEN_LED, LED_ON);
        digitalWrite(BLUE_LED, LED_ON);
        break;
      case 800:
        digitalWrite(RED_LED, LED_OFF);
        digitalWrite(GREEN_LED, LED_OFF);
        digitalWrite(BLUE_LED, LED_OFF);
        break;
    }
  }
  if ((heartbeat % 100 == 0) && settings.debugOn)
  {
    Serial.print("Button = ");
    Serial.println(digitalRead(BUTTON) == HIGH ? "HIGH" : "LOW");
    Serial.print("Counts: ");
    Serial.print(callbackCount);
    Serial.print("/");
    Serial.print(requestCount);
    Serial.print("/");
    Serial.print(receiveCount);
    Serial.print("/");
    Serial.print(readyCount);
    Serial.print("/");
    Serial.print(addressMatchSlaveTransmitCount);
    Serial.print("/");
    Serial.print(addressMatchSlaveReceiveCount);
    Serial.print("/");
    Serial.print(absolute.x);
    Serial.print("/");
    Serial.print(absolute.y);
    Serial.print("/");
    Serial.print(absolute.timestamp);
    Serial.print("/");
    Serial.print(SYSCFG->CFGR1, HEX);
    Serial.print("/");
    Serial.print(WireSlave.get_cr1(), HEX);
    Serial.print("/");
    Serial.print(WireSlave.get_cr2(), HEX);
    Serial.print("/");
    Serial.print(WireSlave.get_oar1(), HEX);
    Serial.print("/");
    Serial.print(WireSlave.get_isr(), HEX);
    Serial.print("/");
    Serial.print(WireSlave.get_sda_pin());
    Serial.print("/");
    Serial.print(WireSlave.get_scl_pin());
    Serial.println("<<<");
  }
  if (heartbeat == 800) {
    heartbeat = 0;
  }

  // Retrieve the values from the optical flow chip
  sensor.registerRead(0x02);
  int16_t deltaX = ((int16_t)sensor.registerRead(0x04) << 8) | sensor.registerRead(0x03);
  int16_t deltaY = ((int16_t)sensor.registerRead(0x06) << 8) | sensor.registerRead(0x05);

  // check for overflow
  if (checked_add_16(deltaX, last.xOffset, &(last.xOffset)))
  {
    last.incrementOverflowX();
  }
  if (checked_add_16(deltaY, last.yOffset, &(last.yOffset)))
  {
    last.incrementOverflowY();
  }

  absolute.x += deltaX;
  absolute.y += deltaY;

  absolute.timestamp = last.timestamp = micros();

  if (inputComplete) {
    if (inputBuffer[0] == 'a') {
      uint8_t high = getHexDigit(inputBuffer[1]);
      uint8_t low = getHexDigit(inputBuffer[2]);
      if (high == 255 || low == 255) {
        Serial.println("Invalid hex address");
      } else {
        settings.slaveAddress = (high << 4 | low) & 0x7f;
        Serial.print("Changing slave address to ");
        Serial.println(settings.slaveAddress, HEX);
        saveConfig();
      }
    } else if (inputBuffer[0] == 'd') {
      settings.debugOn = inputBuffer[1] == '1';
      Serial.print("Debug ");
      Serial.println(settings.debugOn ? "on" : "off");
      saveConfig();
    }
    inputComplete = false;
  }

  delay(50);
}

uint8_t getHexDigit(char value) {
  switch (value) {
    case '0': case '1': case '2': case '3': case '4':
    case '5': case '6': case '7': case '8': case '9':
      return (uint8_t)(value - '0');
    case 'a': case 'b': case 'c': case 'd': case 'e': case 'f':
      return (uint8_t)(value - 'a' + 10);
    case 'A': case 'B': case 'C': case 'D': case 'E': case 'F':
      return (uint8_t)(value - 'A' + 10);
  }

  return 255;
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputBuffer += inChar;
    if (inChar == '\n') {
      inputComplete = true;
    }
  }
}

bool checked_add_32(int32_t a, int32_t b, int32_t *rp) {
  if (b > 0 && a > INT_MAX - b) {
    *rp =  (a + INT_MIN) + (b + INT_MIN);
    return true;
  }
  if (b < 0 && a < INT_MIN - b) {
    *rp =  (a - INT_MIN) + (b - INT_MIN);
    return true;
  }
  *rp = a + b;
  return false;
}

bool checked_add_16(int16_t a, int16_t b, int16_t *rp) {
  if (b > 0 && a > SHRT_MAX - b) {
    *rp =  (a + SHRT_MIN) + (b + SHRT_MIN);
    return true;
  }
  if (b < 0 && a < SHRT_MIN - b) {
    *rp =  (a - SHRT_MIN) + (b - SHRT_MIN);
    return true;
  }
  *rp = a + b;
  return false;
}

uint8_t lastRegister;
uint8_t cachedValue;
bool cacheStale = true;

uint8_t cachedBuffer[20];
uint8_t cachedBufferCount = 0;
uint8_t *cachedBufferPtr = cachedBuffer;

void(* resetFunc) (void) = 0;

extern const stm32l4_i2c_pins_t g_WirePins;

void dumpState(const char * message)
{
  if (!settings.debugOn) {
    return;
  }
  Serial.println(message);
  Serial.print("    State:");
  Serial.print(WireSlave.state(), HEX);
  //Serial.print(" events:");
  //Serial.print(WireSlave.get_events(), HEX);
  //Serial.print(" xf:");
  //Serial.print(WireSlave.xf_count());
  //Serial.print(" tx:");
  //Serial.print(WireSlave.tx_count());
  Serial.print(" CR1:");
  Serial.print(WireSlave.get_cr1(), HEX);
  //Serial.print(" CR2:");
  //Serial.print(WireSlave.get_cr2(), HEX);
  Serial.print(" ISR:");
  Serial.print(WireSlave.get_isr(), HEX);
  //Serial.print(" ICR:");
  //Serial.print(WireSlave.get_icr(), HEX);
  //Serial.print(" scl-GPIO:");
  //Serial.print(stm32l4_gpio_pin_read_configure(g_WirePins.scl), HEX);
  //Serial.print(" sda-GPIO:");
  //Serial.println(stm32l4_gpio_pin_read_configure(g_WirePins.sda), HEX);
  Serial.println();
}



void ll_callback(struct _stm32l4_i2c_t *i2c)
{
  callbackCount++;

  I2C_TypeDef * I2C = i2c->I2C;
  uint32_t i2c_isr = I2C->ISR;

  if (i2c->state == I2C_STATE_READY)
  {
    readyCount++;
    if (i2c_isr & I2C_ISR_ADDR)
    {
      if (i2c_isr & I2C_ISR_DIR)
      {
        addressMatchSlaveTransmitCount++;
        
        dumpState("Address match, slave transmit");
        I2C->CR1 |= (I2C_CR1_TXIE | I2C_CR1_NACKIE | I2C_CR1_STOPIE);
        I2C->ISR |= I2C_ISR_TXE;
        I2C->ICR = I2C_ICR_ADDRCF;
        i2c->state = I2C_STATE_SLAVE_TRANSMIT;
      }
      else
      {
        addressMatchSlaveReceiveCount++;

        dumpState("Address match, slave receive");
        I2C->CR1 |= (I2C_CR1_RXIE | I2C_CR1_STOPIE);
        I2C->ICR = I2C_ICR_ADDRCF;
        i2c->state = I2C_STATE_SLAVE_RECEIVE;
      }
    }
  }
  else if (i2c->state == I2C_STATE_SLAVE_RECEIVE)
  {
    if (i2c_isr & I2C_ISR_RXNE)
    {
      if (i2cState == STATE_WAITING_FOR_REGISTER || i2cState == STATE_WAITING_FOR_READ) {
        debugPrint("Receive data ");
        lastRegister = I2C->RXDR;
        debugPrintln(lastRegister, HEX);
        cacheStale = true;
        i2cState = determineStateFromRegisterReceive(lastRegister);
      } else if (i2cState == STATE_WAITING_FOR_DATA || i2cState == STATE_WAITING_FOR_READ_OR_DATA) {
        uint8_t data = I2C->RXDR;
        switch (lastRegister) {
          case LEDSTATUS_CMD:
            setLEDStatus(data);
            i2cState = STATE_WAITING_FOR_REGISTER;
            break;
          case ZERO_ABSOLUTE_CMD:
            // We don't care about the data, just zero out the absolute data
            absolute.reset();
            i2cState = STATE_WAITING_FOR_REGISTER;
            break;
          case RESET_CMD:
            if (data == 0) {
              Serial.println("Debug off");
              settings.debugOn = 0;
              saveConfig();
            } else if (data == 1) {
              Serial.println("Debug on");
              settings.debugOn = 1;
              saveConfig();
            }
            resetFunc();
            i2cState = STATE_WAITING_FOR_REGISTER;
            break;
          case CHANGE_I2C_ADDRESS_CMD:
            settings.slaveAddress = data & 0x7f;
            Serial.print("Changing slave address to ");
            Serial.println(settings.slaveAddress, HEX);
            saveConfig();
            break;
          default:
            i2cState = STATE_WAITING_FOR_REGISTER;
            break;
        }
      }
    }
    else
    {
      if (i2c_isr & I2C_ISR_ADDR)
      {
        dumpState("Receive terminated by address match; repeated start");
        I2C->CR1 &= ~(I2C_CR1_RXIE | I2C_CR1_STOPIE);
        if (i2c_isr & I2C_ISR_DIR)
        {
          debugPrintln("Address match, slave transmit");
          I2C->CR1 |= (I2C_CR1_TXIE | I2C_CR1_NACKIE | I2C_CR1_STOPIE);
          I2C->ISR |= I2C_ISR_TXE;
          I2C->ICR = I2C_ICR_ADDRCF;
          i2c->state = I2C_STATE_SLAVE_TRANSMIT;
        }
        else
        {
          debugPrintln("Address match, slave receive");
          I2C->CR1 |= (I2C_CR1_RXIE | I2C_CR1_STOPIE);
          I2C->ICR = I2C_ICR_ADDRCF;
          i2c->state = I2C_STATE_SLAVE_RECEIVE;
        }
      }
      else if (i2c_isr & I2C_ISR_STOPF)
      {
        dumpState("Received terminated by stop");
        I2C->CR1 &= ~(I2C_CR1_RXIE | I2C_CR1_STOPIE);
        I2C->ICR = I2C_ICR_STOPCF;
        i2c->state = I2C_STATE_READY;
      }
    }
  }
  else if (i2c->state == I2C_STATE_SLAVE_TRANSMIT)
  {
    if (i2c_isr & I2C_ISR_ADDR)
    {
      dumpState("Transmit terminated by address match");
      I2C->CR1 &= ~(I2C_CR1_TXIE | I2C_CR1_NACKIE | I2C_CR1_STOPIE);
      if (i2c_isr & I2C_ISR_DIR)
      {
        debugPrintln("Address match, slave transmit");
        I2C->CR1 |= (I2C_CR1_TXIE | I2C_CR1_NACKIE | I2C_CR1_STOPIE);
        I2C->ISR |= I2C_ISR_TXE;
        I2C->ICR = I2C_ICR_ADDRCF;
        i2c->state = I2C_STATE_SLAVE_TRANSMIT;
      }
      else
      {
        debugPrintln("Address match, slave receive");
        I2C->CR1 |= (I2C_CR1_RXIE | I2C_CR1_STOPIE);
        I2C->ICR = I2C_ICR_ADDRCF;
        i2c->state = I2C_STATE_SLAVE_RECEIVE;
      }
    }
    else if (i2c_isr & I2C_ISR_NACKF)
    {
      dumpState("Transmit terminated by nack");
      I2C->CR1 &= ~(I2C_CR1_TXIE | I2C_CR1_NACKIE | I2C_CR1_STOPIE);
      I2C->ICR = (I2C_ICR_NACKCF | I2C_ICR_STOPCF);
      i2c->state = I2C_STATE_READY;
    }
    else if (i2c_isr & I2C_ISR_STOPF)
    {
      dumpState("Transmit terminated by stop");
      I2C->CR1 &= ~(I2C_CR1_TXIE | I2C_CR1_NACKIE | I2C_CR1_STOPIE);
      I2C->ICR = (I2C_ICR_NACKCF | I2C_ICR_STOPCF);
      i2c->state = I2C_STATE_READY;
    }
    else if (i2c_isr & I2C_ISR_TXIS)
    {
      debugPrintln("Transmit byte");
      if (cacheStale)
      {
        uint8_t value;
        switch (lastRegister)
        {
          case PRODUCT_ID_CMD:
            value = productId;
            break;
          case CP_PRODUCT_ID_CMD:
            //Serial.println("CP_PRODUCT_ID");
            value = CP_PRODUCT_ID;
            break;
          case REVISION_ID_CMD:
            value = revisionId;
            break;
          case CP_REVISION_ID_CMD:
            value = CP_REVISION_ID;
            break;
          case LEDSTATUS_CMD:
            value = ledStatus;
            break;
          case READ_RELATIVE_CMD:
            cached = last;
            cached.calcCrc();
            cachedBufferCount = sizeof(cached);
            memcpy(cachedBuffer, &cached, cachedBufferCount);
            cachedBufferPtr = cachedBuffer;
            value = *cachedBufferPtr++;
            break;
          case REREAD_RELATIVE_CMD:
            cachedBufferCount = sizeof(cached);
            memcpy(cachedBuffer, &cached, cachedBufferCount);
            cachedBufferPtr = cachedBuffer;
            value = *cachedBufferPtr++;
            break;
          case READ_ABSOLUTE_CMD:
            absolute.calcCrc();
            cachedBufferCount = sizeof(absolute);
            memcpy(cachedBuffer, &absolute, cachedBufferCount);
            cachedBufferPtr = cachedBuffer;
            value = *cachedBufferPtr++;
            break;
          default:
            value = 0x00;
            break;
        }
        cachedValue = value;
        cacheStale = false;
        i2cState = STATE_WAITING_FOR_REGISTER;
      } else {
        uint8_t value;
        switch (lastRegister)
        {
          case READ_RELATIVE_CMD:
          case REREAD_RELATIVE_CMD:
          case READ_ABSOLUTE_CMD:
            if (cachedBufferPtr < (cachedBuffer + cachedBufferCount)) {
              value = *cachedBufferPtr++;
            } else {
              value = 0x00;
            }
            break;
          default:
            value = 0x00;
            break;
        }
        cachedValue = value;
      }
      debugPrintln(cachedValue, HEX);
      I2C->TXDR = cachedValue;
      lastValue = cachedValue;
      lastValueSet = true;
    }
  }
}

int determineStateFromRegisterReceive(int lastRegister)
{
  switch (lastRegister) {
    case LEDSTATUS_CMD:
      return STATE_WAITING_FOR_READ_OR_DATA;
    case ZERO_ABSOLUTE_CMD:
    case RESET_CMD:
      return STATE_WAITING_FOR_DATA;
    default:
      return STATE_WAITING_FOR_READ;
  }
}

void setLEDStatus(uint8_t status)
{
  if (!ledOverwritten) {
    debugPrintln("Overwriting LEDs");
  }
  debugPrintln("setLEDStatus");
  ledOverwritten = true;
  //digitalWrite(WHITE_LED, status & 0x01 ? LED_ON : LED_OFF);
  digitalWrite(BLUE_LED, status & 0x02 ? LED_ON : LED_OFF);
  digitalWrite(GREEN_LED, status & 0x04 ? LED_ON : LED_OFF);
  digitalWrite(RED_LED, status & 0x08 ? LED_ON : LED_OFF);

  ledStatus = status;
}

void requestEvent() {
  requestCount++;
}

void receiveEvent(int bytesReceived) {
  receiveCount++;
}

void debugPrint(const char message[])
{
  if (settings.debugOn) {
    Serial.print(message);
  }
}

void debugPrintln(const char message[])
{
  if (settings.debugOn) {
    Serial.println(message);
  }
}

void debugPrintln(uint8_t v, int radix)
{
  if (settings.debugOn) {
    Serial.println(v, radix);
  }
}

void debugPrintln(char v, int radix)
{
  if (settings.debugOn) {
    Serial.println(v, radix);
  }
}

