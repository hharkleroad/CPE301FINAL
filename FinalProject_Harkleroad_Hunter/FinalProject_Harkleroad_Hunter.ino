#include <LiquidCrystal.h>
#include <dht.h>
#include <Stepper.h>
#include <DS3231.h>
#define RDA 0x80
#define TBE 0x20
#define STEPS 500
//GPIO Pointers
volatile unsigned char *port_a = (unsigned char *) 0x22;
volatile unsigned char *ddr_a = (unsigned char *) 0x21;
volatile unsigned char *port_b = (unsigned char *) 0x23;
volatile unsigned char *port_c = (unsigned char *) 0x28;
volatile unsigned char *ddr_c = (unsigned char *) 0x27;
volatile unsigned char *port_g = (unsigned char *) 0x34;
volatile unsigned char *ddr_g = (unsigned char *) 0x33;
volatile unsigned char *port_h = (unsigned char *) 0x102;
volatile unsigned char *ddr_h = (unsigned char *) 0x101;
volatile unsigned char *pin_h = (unsigned char *) 0x100;
//Serial Pointers
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;
//ADC Pointers
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;
//Timer Pointers
volatile unsigned char *myTCCR1A  = 0x80;
volatile unsigned char *myTCCR1B  = 0x81;
volatile unsigned char *myTCCR1C  = 0x82;
volatile unsigned char *myTIMSK1  = 0x6F;
volatile unsigned char *myTIFR1   = 0x36;
volatile unsigned int  *myTCNT1   = 0x84;
// LCD pins <--> Arduino pins
const int RS = 50, EN = 52, D4 = 53, D5 = 51, D6 = 49, D7 = 47;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);
//global tick counter
unsigned int currentTicks = 0;
unsigned int timer_running;
//Set Humidity and Temp Pin
dht DHT;
//Set Stepper Pins
Stepper stepper(STEPS, 22, 24, 26, 28);
int Pval = 0;
//Set Start Button ISR Pins
const byte ledPin = 35;
const byte interruptPin = 18;
volatile byte powerstate = LOW;
//set Machine State
int Machinestate = 0;
int prevstate;
int change;
// Set RTC
DS3231  rtc(SDA, SCL);
bool sent = false;
int min;
// set millis variable
unsigned long startmill;
unsigned long mill;
const unsigned long timer = 60000;
// set reset variable
int resstate = 0;


//Setup
void setup()
{
  //serial rate 9600
  U0init(9600);
  //adc initialize
  adc_init();
  //GPIO output pins
  *ddr_c |= 0x05;
  *ddr_g |= 0x05;
  *ddr_a |= 0x02;
  *ddr_h &= ~(0x10);
  *port_h &= ~(0x10);

  //Set Stepper Speed
  stepper.setSpeed(20);
  //Set RTC
  rtc.begin();
  rtc.setDOW(WEDNESDAY);
  rtc.setTime(12, 15, 0);
  rtc.setDate(5, 8, 2024);
  //Set power button ISR
  attachInterrupt(digitalPinToInterrupt(interruptPin), power, RISING);
  //Set LCD
  lcd.begin(16,2);
  SetDisplayHT();
  //Disable Motor
  MotorOFF();

}
//Loop
void loop()
  {
  if (powerstate == LOW)
  {
    *port_c &= ~(0x05);
    *port_g &= ~(0x05);
    *port_c |= 0x04 ;
    MotorOFF();
  }
  else if(powerstate == HIGH)
  {
    if (Machinestate == 0)//idle
    {
      *port_c &= ~(0x05);
      *port_g &= ~(0x05);
      *port_c |= 0x01 ;
      SendTime();
      MoveStepper();
      DisplayHT();
      MotorOFF();
      MonitorTemp();
      MonitorWater();

    }
    else if (Machinestate == 1)// running
    {
      SendTime();
      *port_c &= ~(0x05);
      *port_g &= ~(0x05);
      *port_g |= 0x04;
      MoveStepper();
      DisplayHT();
      MotorON();
      MonitorTemp();
      MonitorWater();
      
    }
    else if (Machinestate == 2)//Error
    {
      SendTime();
      DisplayError();
      MotorOFF();
      *port_c &= ~(0x05);
      *port_g &= ~(0x05);
      *port_g |= 0x01;
      MonitorWater();
      resetbutton();
    }
  }
  }


//Functions

//Serial Functions
void U0init(int U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}
unsigned char U0kbhit()
{
  return *myUCSR0A & RDA;
}
unsigned char U0getchar()
{
  return *myUDR0;
}
void U0putchar(unsigned char U0pdata)
{
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}
//ADC Functions
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
  if(adc_channel_num > 7)
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
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}
//Timer Functions
//Delay
void my_delay(unsigned int (freq))
{
  // calc period
  double period = 1.0/double(freq);
  // 50% duty cycle
  double half_period = period/ 2.0;
  // clock period def
  double clk_period = 0.0000000625;
  // calc ticks
  unsigned int ticks = half_period / clk_period;
  // stop the timer
  *myTCCR1B &= 0xF8;
  // set the counts
  *myTCNT1 = (unsigned int) (65536 - ticks);
  // start the timer
  * myTCCR1A = 0x0;
  * myTCCR1B |= 0x04;
  // wait for overflow
  while((*myTIFR1 & 0x01)==0); // 0b 0000 0000
  // stop the timer
  *myTCCR1B &= 0xF8;   // 0b 0000 0000
  // reset TOV           
  *myTIFR1 |= 0x01;
}
// Power Button
void power() 
{
  sent = false;
if (powerstate == LOW )
{
  powerstate = HIGH;
}
else if (powerstate == HIGH)
{
  powerstate = LOW;
}
    if (sent == false)
    {
    U0putchar(rtc.getDOWStr());
    U0putchar(' ');
    U0putchar(rtc.getDateStr());
    U0putchar(' ');
    U0putchar(rtc.getTimeStr());
    U0putchar('\n');
    sent = true;
    }
}
// Display Temp and Humidity
void DisplayHT()
{
mill = millis();
if ((mill - startmill) >= timer)
{
  DHT.read11(32);
  float Temp = DHT.temperature;
  float Humid = DHT.humidity;
  lcd.clear();
  lcd.setCursor(0,0); 
  lcd.print("Temp: ");
  lcd.print(Temp);
  lcd.print((char)223);
  lcd.print("C");
  lcd.setCursor(0,1);
  lcd.print("Humidity: ");
  lcd.print(Humid);
  lcd.print("%");
  startmill = mill;
}
else{}
}
void SetDisplayHT()
{
  lcd.clear();
  lcd.setCursor(0,0); 
  lcd.print("Temp: ");
  lcd.print(0.0);
  lcd.print((char)223);
  lcd.print("C");
  lcd.setCursor(0,1);
  lcd.print("Humidity: ");
  lcd.print("0.0");
  lcd.print("%");
}
// Move Stepper()
void MoveStepper()
{
  int val = adc_read(5);
  stepper.step(val - Pval);
  Pval = val;
  my_delay(40);
}
// Send Time
void SendTime() 
{ 
  state_check();
  if (change == 1)
  {
    if (sent == false)
    {
    U0putchar(rtc.getDOWStr());
    U0putchar(' ');
    U0putchar(rtc.getDateStr());
    U0putchar(' ');
    U0putchar(rtc.getTimeStr());
    U0putchar('\n');
    my_delay(40);
    sent = true;
    }
  }
}
// Monitor Temperature
void MonitorTemp()
{
  DHT.read11(32);
  float Temp = DHT.temperature;
  if (Temp > 23.0)
  {
    prevstate = Machinestate;
    Machinestate = 1;
  }
  else if (Temp <= 22.0)
  {
    prevstate = Machinestate;
    Machinestate = 0;
  }
  my_delay(40);
}
// Monitoer Water Level
void MonitorWater()
{
 int Water = adc_read(8);
if(Water < 10)
{
  prevstate = Machinestate;
  Machinestate = 2;
}
my_delay(30);
}
// Display Error to LCD
void DisplayError()
{
  lcd.clear();
  lcd.setCursor(0,0);// set the cursor on the first row and column
  lcd.print("ERROR LOW WATER");
}
// Turn DC on
void MotorON()
{
*port_a &= ~(0x02);
*port_a |= (0x02);
}
// Turn DC off
void MotorOFF()
{
*port_a &= ~(0x02);
}
//Check State Change
void state_check()
{

if (Machinestate == prevstate)
{
  change = 0;
  sent = true;
}
else if (Machinestate != prevstate)
{
  change = 1;
  sent = false;
}

}
//ResetButton
void resetbutton()
{
int reset;
if (*pin_h & 0x10)
{
reset = 1;
}
else
{
reset = 0;
}
if(reset != resstate)
{
  if ((reset == 1) && (adc_read(8) >= 27))
  {
    prevstate = Machinestate;
    Machinestate = 0;
    SetDisplayHT();
  }
  else
  {
    prevstate = Machinestate;
    Machinestate = 2;
  }
}
resstate = reset;
}
