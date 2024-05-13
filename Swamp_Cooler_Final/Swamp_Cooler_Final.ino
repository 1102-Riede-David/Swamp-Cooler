

//David Riede
//Final Project Spring 2024
//CPE 301.1001
//April 28, 2024

//DHT
#include <dht.h>
#define dht_apin 7


dht DHT;




//Stepper Moter
#include <Stepper.h>
int motorSpeed = 10;
Stepper myStepper(2048, 22, 23, 24, 25);


//LCD
#include <LiquidCrystal.h>
const int RS = 12, EN = 6, D4 = 38, D5 = 37, D6 = 36, D7 = 35;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);


//UART
volatile unsigned char *myUCSR0A  = 0xC0;
volatile unsigned char *myUCSR0B  = 0xC1;
volatile unsigned char *myUCSR0C  = 0xC2;
volatile unsigned int  *myUBRR0   = 0xC4;
volatile unsigned char *myUDR0    = 0xC6;

 #define RDA 0x80
 #define TBE 0x20  


//Pin 8, 9, 10, 11 are the LED's
//PH
volatile unsigned char* port_h = (unsigned char*) 0x102;
volatile unsigned char* ddr_h  = (unsigned char*) 0x101;
//PB
volatile unsigned char* port_b = (unsigned char*) 0x25;
volatile unsigned char* ddr_b  = (unsigned char*) 0x24;


//ADC
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

//PE
volatile unsigned char* ddr_e  = (unsigned char*) 0x2D;
volatile unsigned char* pin_e  = (unsigned char*) 0x2C;
volatile unsigned char* port_e = (unsigned char*) 0x2E;

//PG
volatile unsigned char* ddr_g  = (unsigned char*) 0x33;
volatile unsigned char* pin_g  = (unsigned char*) 0x32;

//PF
volatile unsigned char* ddr_f  = (unsigned char*) 0x2F;

//PA (A0, A1, A2, A3)
volatile unsigned char* ddr_a  = (unsigned char*) 0x21;
volatile unsigned char* port_a = (unsigned char*) 0x22;
volatile unsigned char* pin_a  = (unsigned char*) 0x20;

//PC
volatile unsigned char* pin_c  = (unsigned char*) 0x26;
volatile unsigned char* ddr_c  = (unsigned char*) 0x27;
//Clock
#include "RTClib.h"
RTC_DS1307 rtc;

//Lights
//Red ( water level low)
bool warningLight = 0;

//Blue ( Fan On)
bool fanOnLight = 0;

//Yellow( Fan off)
bool fanOffLight = 0;

//Green (Water levels right)
bool allClearLight = 0;

int resval = 0;
int aSig = 5;

bool resetButton = 0;


bool x = false;
bool buttonNew = 0;
bool buttonOld = 0;
bool fanState = 0;

//States
bool Disabled = 0;
bool Idle = 0;
bool Running = 0;
bool Error = 0;


void setup() {
  //Start
  U0Init(9600);

  myStepper.setSpeed(motorSpeed);
  
  
  #ifndef ESP8266
  while (!Serial); // wait for serial port to connect. Needed for native USB
  #endif

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }

  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }


  lcd.begin(16, 2);

 
  //Initializing ADC
  adc_init();

  //setting INPUTS
  *ddr_e &= 0xEF;

  *ddr_g &= 0xF7;
  *ddr_g &= 0xBF;
  *ddr_a &= 0x7F;
  //*ddr_f &= 0xDF;



  //setting OUPUTS
  *ddr_h |= 0x01 << 5;
  *ddr_h |= 0x01 << 6;

  *ddr_b |= 0x01 << 4;
  *ddr_b |= 0x01 << 5;
  *ddr_b |= 0x01 << 2;

  *ddr_e |= 0x01 << 5;
  *ddr_a &= 0x0F;
}

void loop() {
  
  int temp = DHT.temperature;
  toggleOn();
   unsigned int result = adc_read(aSig);
  water_monitor(result);
  changeState(result);
  fan();
  lightControl();
  motorControl();
  reset();
  checkTime();
  lcd.clear();
  
  
  

}

void U0Init(int U0baud)
{
  unsigned long FCPU = 16000000;
  unsigned int tbaud;
  tbaud = (FCPU / 16 / U0baud - 1);
  // Same as (FCPU / (16 * U0baud)) - 1;
  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0  = tbaud;
}

void button() {
  if (*pin_e & 0x10) {
    buttonNew = 1;
  } else {
    buttonNew = 0;
  }

}

void toggleOn() {
  button();
  if (buttonOld == 0 && buttonNew == 1) {
    if (fanState == 0) {
      fanState = 1;
    } else if (fanState == 1) {
      fanState = 0;
    }
  }
  buttonOld = buttonNew;
  lcd.clear();
}


void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}

unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if (adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while ((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}


void write_ph(unsigned char pin_num, bool state)
{
  if (state == 0)
  {
    *port_h &= ~(0x01 << pin_num);
  }
  else
  {
    *port_h |= 0x01 << pin_num;
  }
}

void write_pb(unsigned char pin_num, bool state)
{
  if (state == 0)
  {
    *port_b &= ~(0x01 << pin_num);
  }
  else
  {
    *port_b |= 0x01 << pin_num;
  }
}

void lightControl() {
  write_pb(5, allClearLight);
  write_pb(4, warningLight);
  write_ph(6, fanOnLight);
  write_ph(5, fanOffLight);
}

void water_monitor(int level) {
  int temp = DHT.temperature;
  if (fanState == 0 && level > 100 && (temp < 18|| temp >= 18)) {
    Disabled = 0;
    Idle = 1;
    Error = 0;
    Running = 0;
  } else if (fanState == 1 && level < 100 && temp >= 18) {
    Disabled = 0;
    Error = 1;
    Idle = 0;
    Running = 0;
  } else if (fanState == 1 && level > 100 && temp > 18) {
    Idle = 0;
    Running = 1;
    Error = 0;
    Disabled = 0;
  } else if (fanState == 0 && level < 100) {
    allClearLight = 0;
    Error = 0;
    Disabled = 1;
    Idle = 0;
    Running = 0;
    
  }
}
void timeMonitor(){
  char noon;

  DateTime now = rtc.now();
  char a = '0';
  char a1 = '0';
    charConverter(now.day(), a, a1);
    U0putchar('/');

    char m1 = '0';
    char m = '0';

    charConverter(now.month(), m, m1);
    U0putchar('/');

    char y[ ] = "2024";
    stringPrint(y, 4);
    U0putchar('\n');

    char h1 = '0';
    char h = '0';
    charConverter(now.hour(), h, h1);
    U0putchar(':');

    char i1;
    char i;
    charConverter(now.minute(), i, i1);
    U0putchar(':');

    char s1;
    char s;
    charConverter(now.second(), s, s1);
    U0putchar('\n');
    
    delay(200);
}

void motorControl() {
  if (*pin_a & 0x80) {
    myStepper.step(350);
  }
  

}

void display_temp_humid(){
  DHT.read11(dht_apin);
  int humid = DHT.humidity;
  int temp = DHT.temperature;
  lcd.clear();
  //print Temperature to the LCD
  lcd.setCursor(0,0);
  lcd.print(temp);
  lcd.print((char)223);
  lcd.print("C");

  //HUMIDITY
  lcd.setCursor(0, 1);
  lcd.print(humid);
  lcd.print(" %rh");
  delay(500);
  }

  void changeState(int lev){
    if(Disabled){
      fanOffLight = 1;
      lcd.setCursor(0,0);
      lcd.print("Swamp");
      lcd.setCursor(0,1);
      lcd.print("Cooler");
      //attachInterrupt(digitalPinToInterrupt(2). button, RISING);
      delay(500);
    }else if(Error){
      checkReset(lev);
      errorMessage();
      delay(500);
      warningLight = 1;
      fanOffLight = 0;
      allClearLight = 0;
      fanOnLight = 0;
    }else if(Idle){
      display_temp_humid();
      timeMonitor();
      delay(500);
      allClearLight = 1;
      fanOnLight = 0;
      warningLight = 0;
      
    }else if(Running){
      display_temp_humid();
      delay(500);
      //analogWrite(3, 500);
      fanOnLight = 1;
      warningLight = 0;
      fanOffLight = 0;
      allClearLight = 0;
    }
  }

  void reset() {
  if (*pin_g & 0x20) {
    resetButton = 1;
  } else {
    resetButton = 0;
  }

}


void checkReset(int water){
  if(resetButton){
    if(water > 100){
      Idle = 1;
      Error = 0;
      lcd.clear();
    }
  }
}

void errorMessage(){
  lcd.setCursor(0,0);
  lcd.print("ERROR:");
  lcd.setCursor(0,1);
  lcd.print("WATER LEVEL LOW");
  delay(500); 
}

void fan(){
  if(Running){
    analogWrite(51, 500);
  }else{
    analogWrite(51, 0);
  }
}

unsigned char U0kbhit()
{
  //Check the RDA status bit, and return True if the bit is set, return False if the bit is clear.
  return (*myUCSR0A & RDA) ? 1 : 0;

  
}
//
// Read input character from USART0 input buffer
//
unsigned char U0getchar()
{
  //Return the character which has been received by the UART.
  return *myUDR0;
}
//
// Wait for USART0 (myUCSR0A) TBE to be set then write character to
// transmit buffer
//
void U0putchar(unsigned char U0pdata)
{
   while(!(*myUCSR0A & TBE));
    *myUDR0 = U0pdata;
   
}

void stringPrint(char string[ ], int a){
  for(int i = 0; i < a; i++){
    U0putchar(string[i]); 
  }
}

void charConverter(int orignal, char d, char d1){
  d = '0';
  d1 = '0';
    if(orignal < 10){
      d = orignal + 48;
    }else if(orignal >= 10 && orignal < 20){
      d1 = '1';
      d = orignal + 38;
    }else if(orignal >= 20 && orignal < 30){
      d1 = '2';
      d = orignal + 28;
    }else if(orignal >= 30 && orignal < 40){
      d1 = '3';
      d = orignal + 18;
    }else if(orignal >= 40 && orignal < 50){
      d1 = '4';
      d = orignal + 8;
    }else if(orignal >= 50 && orignal < 60){
      d1 = '5';
      d = orignal - 2;
    }


    
    U0putchar(d1);
    U0putchar(d);
}

void checkTime(){
  if(Idle){
    stringPrint("Fan off", 7);
    U0putchar('\n');
    timeMonitor();
    U0putchar('\n');
  }else if(Running){
    stringPrint("Fan on", 6);
    U0putchar('\n');
    timeMonitor();
    U0putchar('\n');
  }
}
