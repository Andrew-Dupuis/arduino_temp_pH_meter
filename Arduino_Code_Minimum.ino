// Includes
#include <Keypad.h>
#include <OneWire.h>
#include <LiquidCrystal.h>


// Defines

// Constants
const int KEYPAD_ROWS = 4;
const int KEYPAD_COLUMNS = 3;
const byte keypad_row_pins[KEYPAD_ROWS] = {42, 41, 40, 39};
const byte keypad_column_pins[KEYPAD_COLUMNS] = {38, 37, 36};
const char kaypad_map[KEYPAD_ROWS][KEYPAD_COLUMNS] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};
const char KEYPAD_CANCEL_CHARACTER = '*';
const char KEYPAD_START_CHARACTER = '#';

const int TEMPERATURE_ONEWIRE_PIN = 28;
const byte CHANNEL_ONE_TEMPERATURE_ADDRESS[8] = {0x28, 0xFF, 0x25, 0x20, 0xB0, 0x17, 0x05, 0x13};
const int CHANNEL_ONE_PH_PIN = A1;
const byte CHANNEL_TWO_TEMPERATURE_ADDRESS[8] = {0x28, 0xFF, 0xA4, 0x1D, 0xB0, 0x17, 0x05, 0xC0};
const int CHANNEL_TWO_PH_PIN = A0;
const int LCD_RS = 12, LCD_EN = 11, LCD_D4 = 5, LCD_D5 = 4, LCD_D6 = 3, LCD_D7 = 2;

// Variables
bool isProcessSetupCompleted = false;
float processingDuration;
Keypad keypad = Keypad(makeKeymap(kaypad_map), keypad_row_pins, keypad_column_pins, KEYPAD_ROWS, KEYPAD_COLUMNS);
OneWire oneWire(TEMPERATURE_ONEWIRE_PIN);
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);




// Default Arduino Methods
void setup() {
  lcd.begin(16, 2);
  display_reset_wait();
  Serial.begin(9600);  
  Serial.println("Ready");    //Test the serial monitor
}

void loop() {
  char key = check_keypad(keypad);
  if(key){
    switch (key){
      case '1':
        status_monitor_start();
        display_reset_wait();
        break;
      case '2':
        break;
      case '3':
        break;
      default:
        Serial.println(key);
    }
  }
}





// Main Operation Starts
void status_monitor_start() {
  lcd.clear();
  while (true) {
    if (check_keypad(keypad) == KEYPAD_CANCEL_CHARACTER) {
      break;
    }

    float ChOnepH = check_pH(CHANNEL_ONE_PH_PIN);
    float ChOnetemperature = check_temperature(CHANNEL_ONE_TEMPERATURE_ADDRESS);

    float ChTwopH = check_pH(CHANNEL_TWO_PH_PIN);
    float ChTwotemperature = check_temperature(CHANNEL_TWO_TEMPERATURE_ADDRESS);

    update_status_display(ChOnepH, ChOnetemperature, ChTwopH, ChTwotemperature);
  }
}


// Timer Methods
void do_countdown(float duration) {
  lcd.clear();
  unsigned long endTime = millis() + duration * 1000;
  while (millis() < endTime) {
    if (check_keypad(keypad) == KEYPAD_CANCEL_CHARACTER) {
      break;
    }
    update_time_display((endTime - millis()) / 1000);
    delay(10);
  }
}


// Display Methods
void update_time_display(unsigned long seconds) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Time Remaining:");
  lcd.setCursor(0,1);
  lcd.print(seconds);
}

void update_status_display(float chOnepH, float chTwopH, float chOnetemp, float chTwotemp) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Ch1: ");
  lcd.print(chOnepH);
  lcd.print(" ");
  lcd.print(chOnetemp);
  lcd.setCursor(0, 1);
  lcd.print("Ch2: ");
  lcd.print(chTwopH);
  lcd.print(" ");
  lcd.print(chTwotemp);
}

void display_reset_wait() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Resetting... ");
  lcd.setCursor(0, 1);
  delay(100);
  lcd.print("Please Wait...");
  delay(1500);    
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Ready");
}



// Input Query Methods
float check_temperature(byte addr[8]) {
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  float celsius;

  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return;
  }

  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      type_s = 1;
      break;
    case 0x28:
      type_s = 0;
      break;
    case 0x22:
      type_s = 0;
      break;
    default:
      return;
  } 

  oneWire.reset();
  oneWire.select(addr);
  oneWire.write(0x44);        // start conversion, use ds.write(0x44,1) with parasite power on at the end

  delay(100);     // maybe 750ms is enough, maybe not

  present = oneWire.reset();
  oneWire.select(addr);    
  oneWire.write(0xBE);         // Read Scratchpad
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = oneWire.read();
  }
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  Serial.println(celsius);
  return celsius;
}

float check_pH(int analog_pin) {
  pinMode(analog_pin, INPUT);
  unsigned long int avgValue=0;
  float b;
  int buf[10],temp;
  for(int i=0;i<10;i++)       // Get 10 sample values
  { 
    buf[i]=analogRead(analog_pin);
    delay(10);
  }
  for(int i=2;i<8;i++){                      //take the average value of 6 center sample
    avgValue+=buf[i];
  }
  float phValue=(float)avgValue*5.0/1024/6; //convert the analog value into millivolts
  phValue=3.5*phValue +0.06;                      //convert the millivolts into pH value
  Serial.println(phValue);
  return phValue;
}

char check_keypad(Keypad keypad_in) {
  char key = keypad_in.getKey();
  if (key)
  {
    Serial.println(key);
    return key;
  }
}

