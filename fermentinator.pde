/**
* Copyright 2010 Steve Baker <steve@stevebaker.org>
* This program is distributed under the terms of the GNU General Public License
*/
#include <AikoEvents.h>
#include <OneWire.h>

using namespace Aiko;

#define VERSION    "0.1"

// constants
#define SECOND     1000ul    // millis in a second
#define MINUTE     60000ul   // millis in an minute
#define HOUR       3600000ul // millis in an hour
#define MODE_COOL  0       // switch is attached to a fridge
#define MODE_HEAT  1       // switch is attached to a heater

// configuration options
#define MIN_TEMP         (-4 * 100)      // minus 4 deg C
#define MAX_TEMP         (40 * 100)      // 30 deg C
#define COOL_DELAY       (1ul * MINUTE)
// 3 minute delay between turning off cooling and turning it on again
#define DIFF_TEMP         50           // Keep temp +/- 0.5 deg C
#define SWITCH_DURATION   100          // millis to "hold down" the button to toggle a switch
#define TARGET_MULT       (102400l / (MAX_TEMP - MIN_TEMP))
#define TARGET_OFFSET     MIN_TEMP

// Define IO pins
#define PIN_TEMP_SET        1  // Analog knob for setting target temperature
#define PIN_LCD_STROBE      2  // CD4094 8-bit shift/latch
#define PIN_LCD_DATA        3  // CD4094 8-bit shift/latch
#define PIN_LCD_CLOCK       4  // CD4094 8-bit shift/latch
#define PIN_ONE_WIRE        5  // Maxim DS18B20 temperature sensor(s)
#define PIN_SWITCH_1_ON     6  // Pin to turn on  switch 1
#define PIN_SWITCH_1_OFF    7  // Pin to turn off switch 1
//#define PIN_SWITCH_2_ON     8  // Pin to turn on  switch 2
//#define PIN_SWITCH_2_OFF    9  // Pin to turn off switch 2
//#define PIN_SWITCH_3_ON     10 // Pin to turn on  switch 3
//#define PIN_SWITCH_3_OFF    11 // Pin to turn off switch 3

#define ONE_WIRE_COMMAND_READ_SCRATCHPAD  0xBE
#define ONE_WIRE_COMMAND_START_CONVERSION 0x44
#define ONE_WIRE_COMMAND_MATCH_ROM        0x55
#define ONE_WIRE_COMMAND_SKIP_ROM         0xCC

#define ONE_WIRE_DEVICE_18B20  0x28
#define ONE_WIRE_DEVICE_18S20  0x10

OneWire oneWire(PIN_ONE_WIRE);  // Maxim DS18B20 temperature sensor

// state globals
byte switch1Mode = MODE_COOL; // Current mode of switch 1
int currentTemp; // current temperature
int targetTemp;  // target temperature
byte switch1on = false; // whether the switch 1 is currently in the on state
unsigned long switch1Time; // millis since switch 1 changed state

void setup() {
  Serial.begin(38400);
  pinMode(PIN_SWITCH_1_ON, OUTPUT);
  pinMode(PIN_SWITCH_1_OFF, OUTPUT);
  
  showSplashScreen();
  Events.addHandler(readCurrentTemp, 1000, 2000);
  Events.addHandler(readTargetTemp, 250, 3000);
  Events.addHandler(checkSwitchAction, 2000, 3500);
  Events.addHandler(updateLcd, 1000, 3600);
}

void loop() {
  Events.loop();
}

void showSplashScreen(){
  lcdInitialize();
  lcdClear();
  lcdPosition(0, 0);
  lcdWriteString("Fermentinator 3000");
  lcdPosition(1, 0);
  lcdWriteString("v");
  lcdWriteString(VERSION);
}

void updateLcd(void){
  static byte firstRun = true;
  if (firstRun){
    lcdClear();
    firstRun = false;    
  }
    
  lcdPosition(0, 0);
  lcdWriteString("Current:");
  lcdWriteTemperature(currentTemp);

  lcdPosition(1, 1);
  lcdWriteString("Target:");
  lcdWriteTemperature(targetTemp);
  
  lcdPosition(2, 3);
  lcdWriteString("Mode: ");
  if (switch1Mode == MODE_COOL){
    lcdWriteString("COOLING");
  }
  else if (switch1Mode == MODE_HEAT){
    lcdWriteString("HEATING");
  }
  
  lcdPosition(3, 1);
  lcdWriteString("Switch: ");
  if (switch1on){
    lcdWriteString("ON  ");
  }
  else{
    lcdWriteString("OFF ");
  }
  unsigned long s = (millis() - switch1Time) / 1000ul;
  
  int m = s / 60;
  int h = m / 60;
  s = s % 60;
  m = m % 60;

  if (h != 0){
    lcdWriteNumber(h, 2);
    lcdWriteString(":");
  }

  lcdWriteNumber(m, 2);
  if (h == 0){
    lcdWriteString(":");
    lcdWriteNumber(s, 2);
  }
}

void checkSwitchAction(void){
  if (switch1Mode == MODE_COOL){
    if ((currentTemp > targetTemp + DIFF_TEMP) && (millis() - switch1Time > COOL_DELAY)){
      // trigger switch on
      if (!switch1on) {
        switch1Time = millis();
        switch1on = true;
        updateLcd();
      }

    }
    else if (currentTemp <= targetTemp){
      // trigger switch off
      if (switch1on) {
        switch1Time = millis();
        switch1on = false;
        updateLcd();
      }
    }
  }
  //else {
  //  
  //}
  
  if (switch1on){
    digitalWrite(PIN_SWITCH_1_ON, HIGH);
  }
  else {
    digitalWrite(PIN_SWITCH_1_OFF, HIGH);
  }
  Events.addOneShotHandler(switchDone, SWITCH_DURATION);
}

void switchDone(void){
  digitalWrite(PIN_SWITCH_1_ON, LOW);
  digitalWrite(PIN_SWITCH_1_OFF, LOW);
}

void readTargetTemp(void){
  int previousTarget = targetTemp;
  long ts = long(1024 - analogRead(PIN_TEMP_SET)) * 100l;
  targetTemp = (int)(ts / TARGET_MULT)  + TARGET_OFFSET;
  if (previousTarget != targetTemp) updateLcd();
}

void readCurrentTemp(void) {  // total time: 33 milliseconds
  byte address[8];
  byte data[12];
  byte index;
  
  if (! oneWire.search(address)) {  // time: 14 milliseconds
    //Serial.println("(error 'No more one-wire devices')");
    oneWire.reset_search();         // time: <1 millisecond
    return;
  }

  if (OneWire::crc8(address, 7) != address[7]) {
    //Serial.println("(error 'Address CRC is not valid')");
    return;
  }

  if (address[0] != ONE_WIRE_DEVICE_18B20) {
    
    //Serial.println("(error 'Device is not a DS18B20')");
    return;
  }

  static byte oneWireInitialized = false;
  if (oneWireInitialized) {
  
    byte present = oneWire.reset();                   // time: 1 millisecond
    oneWire.select(address);                          // time: 5 milliseconds
    oneWire.write(ONE_WIRE_COMMAND_READ_SCRATCHPAD);  // time: 1 millisecond

    for (index = 0; index < 9; index++) {             // time: 5 milliseconds
      data[index] = oneWire.read();
    }
  
    if (OneWire::crc8(data, 8) != data[8]) {
      //Serial.println("(error 'Data CRC is not valid')");
      return;
    }
  
    int temperature = (data[1] << 8) + data[0];
    int signBit     = temperature & 0x8000;
    if (signBit) temperature = (temperature ^ 0xffff) + 1;  // 2's complement
  
    currentTemp = (6 * temperature) + temperature / 4;  // multiply by 100 * 0.0625
    if (signBit) currentTemp *= -1;
    updateLcd();
  }
  
  
  //Serial.println("initialising onewire");
  // Start temperature conversion with parasitic power
  oneWire.reset();                                      // time: 1 millisecond
  oneWire.select(address);                              // time: 5 milliseconds
  oneWire.write(ONE_WIRE_COMMAND_START_CONVERSION, 1);  // time: 1 millisecond

  // Must wait at least 750 milliseconds for temperature conversion to complete
  oneWireInitialized = true;
}

/* -------------------------------------------------------------------------- */
/* LCD KS0066 4-bit data interface, 3 Arduino pins and MC14094 8-bit register
 * http://www.datasheetsite.com/datasheet/KS0066
 *
 * MC14094 input:  Arduino digital pin 2=Clock, pin 4=Data, pin 7=Strobe
 * MC14094 output: Q8=DB4, Q7=DB5, Q6=DB6, Q5=DB7, Q4=E, Q3=RW, Q2=RS, Q1=None
 * http://www.ee.mut.ac.th/datasheet/MC14094.pdf
 *
 *   +--------------------------------------------+
 *   |    Arduino (ATMega 168 or 328)             |
 *   |    D02           D03           D04         |
 *   +----+-------------+-------------+-----------+
 *        |4            |5            |6
 *        |1            |2            |3
 *   +----+-------------+-------------+-----------+
 *   |    Strobe        Data          Clock       |
 *   |    MC14094 8-bit shift/latch register      |
 *   |    Q8   Q7   Q6   Q5   Q4   Q3   Q2   Q1   |
 *   +----+----+----+----+----+----+----+----+----+
 *        |11  |12  |13  |14  |7   |6   |5   |4
 *        |11  |12  |13  |14  |6   |5   |4 
 *   +----+----+----+----+----+----+----+---------+
 *   |    DB4  DB5  DB6  DB7  E    RW   RS        |
 *   |               LCD KS0066                   |
 *   +--------------------------------------------+
 */



// LCD pin bit-patterns, output from MC14094 -> LCD KS0066 input
#define LCD_ENABLE_HIGH 0x10  // MC14094 Q4 -> LCD E
#define LCD_ENABLE_LOW  0xEF  //   Enable (high) / Disable (low)
#define LCD_RW_HIGH     0x20  // MC14094 Q3 -> LCD RW
#define LCD_RW_LOW      0xDF  //   Read (high) / Write (low)
#define LCD_RS_HIGH     0x40  // MC14094 Q2 -> LCD RS
#define LCD_RS_LOW      0xBF  //   Data (high) / Instruction (low) Select

// LCD Commands
#define LCD_COMMAND_CLEAR             0x01  // Clear display
#define LCD_COMMAND_HOME              0x02  // Set DD RAM address counter to (0, 0)
#define LCD_COMMAND_ENTRY_SET         0x06  // Entry mode set
#define LCD_COMMAND_DISPLAY_SET       0x0C  // Display on/off control
#define LCD_COMMAND_FUNCTION_SET      0x28  // Function set
#define LCD_COMMAND_SET_DDRAM_ADDRESS 0x80  // Set DD RAM address counter (row, column)

#define LCD_SECOND_ROW 64  // Second row literal
#define LCD_THIRD_ROW 20  // Third row literal
#define LCD_FOURTH_ROW 84  // Fourth row literal

byte lcdSetup[] = {         // LCD command, delay time in milliseconds
  LCD_COMMAND_HOME,         50,  // wait for LCD controller to be initialized
  LCD_COMMAND_HOME,         50,  // ditto
  LCD_COMMAND_FUNCTION_SET,  1,  // 4-bit interface, 2 display lines, 5x8 font
  LCD_COMMAND_DISPLAY_SET,   1,  // turn display on, cursor off, blinking off
  LCD_COMMAND_CLEAR,         2,  // clear display
  LCD_COMMAND_ENTRY_SET,     1   // increment mode, display shift off
};

void lcdInitialize(void) {
  pinMode(PIN_LCD_CLOCK,  OUTPUT);
  pinMode(PIN_LCD_DATA,   OUTPUT);
  pinMode(PIN_LCD_STROBE, OUTPUT);

  byte length = sizeof(lcdSetup) / sizeof(*lcdSetup);
  byte index = 0;

  while (index < length) {
    lcdWrite(lcdSetup[index ++], false);
    delay(lcdSetup[index ++]);
  }
}

void lcdWrite(
  byte value,
  byte dataFlag) {

  digitalWrite(PIN_LCD_STROBE, LOW);

  byte output = value >> 4;                                    // Most Significant Nibble
  if (dataFlag) output = (output | LCD_RS_HIGH) & LCD_RW_LOW;  // Command or Data ?

  for (byte loop1 = 0; loop1 < 2; loop1 ++) {  // First MSN, then LSN
    for (byte loop2 = 0; loop2 < 3; loop2 ++) {  // LCD ENABLE LOW -> HIGH -> LOW
      output = (loop2 == 1) ? (output | LCD_ENABLE_HIGH) : (output & LCD_ENABLE_LOW);

      shiftOut(PIN_LCD_DATA, PIN_LCD_CLOCK, LSBFIRST, output);
      digitalWrite(PIN_LCD_STROBE, HIGH);
      delayMicroseconds(10);
      digitalWrite(PIN_LCD_STROBE,LOW);
    }
delay(1);
    output = value & 0x0F;                                       // Least Significant Nibble
    if (dataFlag) output = (output | LCD_RS_HIGH) & LCD_RW_LOW;  // Command or Data ?
  }
}

void lcdClear(void) {
  lcdWrite(LCD_COMMAND_CLEAR, false);
  delay(2);
}

void lcdPosition(
  byte row,        // Must be either 0 (first row) or 1 (second row)
  byte column) {   // Must be between 0 and 15

  if (row == 1) row = LCD_SECOND_ROW;
  else if (row == 2) row = LCD_THIRD_ROW;
  else if (row == 3) row = LCD_FOURTH_ROW;
  lcdWrite(LCD_COMMAND_SET_DDRAM_ADDRESS | row | column, false);
  delayMicroseconds(40);
}

void lcdWriteString(char message[]) {
  while (*message) lcdWrite((*message ++), true);
}

// checks out how many digits there are in a number

int estimateDigits(int nr) {
  int dec = 10;
  int temp = 1;
  int div = nr/dec;
  while (div > 0) {
    dec *= 10;
    div = nr/dec;
    temp++;
  }
  return temp;
}

// Raise number to power

int pow(int base, int expo) {
  int temp = 1;
  for (int c = 1; c <= expo; c++) {
    temp *= base;
  }
  return temp;
}

// this function help us to write numbers
// with more than one digit

void lcdWriteNumber(int nr, int digits) {
  for (int i = digits-1; i >= 0; i--) {
    int dec = pow(10,i);
    int div = nr/dec;
    lcdWrite(div+48, true);
    if (div > 0) {
      nr -= div*dec;
    }
  }
}

void lcdWriteNumber(int nr) {
  int value = nr;

  if (value < 0) {
    lcdWrite('-', true);
    value = - nr;
  }

  int digits = estimateDigits(value);
  lcdWriteNumber(value, digits);
}


void lcdWriteTemperature(int temp) {
  byte signBit = temp < 0;
  if (signBit) temp *= -1;
    
  int temperature_whole    = temp / 100;
  int temperature_fraction = temp % 100;
  
  if (signBit){
    lcdWriteString("-");
  }
  else {
    lcdWriteString(" ");
  }
  
  if (temperature_whole < 10) lcdWriteString(" ");
  lcdWriteNumber(temperature_whole);
  lcdWriteString(".");
  lcdWriteNumber(temperature_fraction, 2);
}

