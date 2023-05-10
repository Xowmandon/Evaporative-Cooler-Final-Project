//CPE301 - Final Project -
//Professor: Shawn Ray
//May 9th 2023
//Joshua Ferguson

// --Description --

//Monitor the water levels in a reservoir and print an alert when the level is too low

// Monitor and display the current air temp and humidity on an LCD screen

// Start and stop a fan motor as needed when the temperature falls out of a specified range (high or
//low)

// Allow a user to use a control to adjust the angle of an output vent from the system

// Allow a user to enable or disable the system using an on/off button

// Record the time and date every time the motor is turned on or off. This information should be
//transmitted to a host computer (over USB)


#include <dht.h>
#include <RTClib.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <Stepper.h>

#define RDA 0x80
#define TBE 0x20
#define DHT_PIN 11


//Holds Current and Previous State (Either DISABLED, ERROR, IDLE, or RUNNING)
char* previousStatePosition = "DISABLED";

//state for On or Off State
volatile bool OnState = false;
unsigned int StateButton = 2;

//instance for Temp/Humidity Module 
dht DHT;

//instance for RTC Module 
RTC_DS1307 rtc;

const int rs = 25, en = 23, d4 = 28, d5 = 26, d6 = 24, d7 = 22;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

const unsigned int STEPS_TOTAL = 32;
const unsigned int STEPPER_PIN_IN1 = 37;
const unsigned int STEPPER_PIN_IN2 = 39;
const unsigned int STEPPER_PIN_IN3 = 41;
const unsigned int STEPPER_PIN_IN4 = 43;

unsigned int potValueVent = 0;
unsigned int prevPotValueVent = 0;

Stepper ventStepperController(STEPS_TOTAL
                          ,STEPPER_PIN_IN1
                          ,STEPPER_PIN_IN3
                          ,STEPPER_PIN_IN2
                          ,STEPPER_PIN_IN4);


//monitoring minutes (update/read temp every minute that passes)
unsigned int currentMin = 0;
unsigned int previousMin = 61;  //impossible value to read on first loop and prevent edge case 

//holds water Level
volatile unsigned int waterLevel = 0;
unsigned int waterLevelThreshold = 200;
unsigned int AnalogPinWaterReadIn = 7;

//Records for Temp and Humidity
volatile unsigned int Temp = 0;
volatile unsigned int Humidity = 0;
unsigned int TempThreshold = 10; //50

//For reading in from ADC and Serial Ports
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;
 
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;


//Digital Pin 50 and Pin 51 (BLUE LED - PB3) (GREEN LED - PB2)
volatile unsigned char* port_b = (unsigned char*) 0x25; 
volatile unsigned char* ddr_b = (unsigned char*) 0x24; 
volatile unsigned char* pin_b  = (unsigned char*) 0x23; 


//Digital Pin 48 and 49 (YELLOW LED - PL1) (RED LED = PL0)
volatile unsigned char* port_l = (unsigned char*) 0x10B; 
volatile unsigned char* ddr_l = (unsigned char*) 0x10A; 
volatile unsigned char* pin_l  = (unsigned char*) 0x109; 

//DC MOTOR/FAN
//PH4 PH5 PH6
//Digital Pin 7 = Enable
//Digital Pin 8 = Direction A
//Digital Pin 9 = Direction B

volatile unsigned char* port_h = (unsigned char*) 0x102; 
volatile unsigned char* ddr_h = (unsigned char*) 0x101; 
volatile unsigned char* pin_h  = (unsigned char*) 0x100; 


//EVENT MESSAGES
const char* tempHumidityMessage = "Monitored Temp/Humidity: ";
const char* waterLevelMessage = "Water Level is ";
const char* DISABLED_MESSAGE = "DISABLED STATE - FAN OFF";
const char* IDLE_MESSAGE = "IDLE STATE - FAN OFF";
const char* ERROR_MESSAGE = "ERROR: Water Level is Too Low!";
const char* RUNNING_MESSAGE = "FAN RUNNING STATE";


void setup() {
  // put your setup code here, to run once:
  // setup the UART
  U0init(9600);

  //Serial Monitor for Event and Time Reporting
  Serial.begin(9600);

  // setup the ADC
  adc_init();

  //Set PB3 and PB2 to OUTPUT (BLUE,GREEN LED)
  //Pb7 is Water Sensor VCC
  *ddr_b |= 0b00001100;

  //Set PL1 and PL0 to OUTPUT (YELLOW, RED LED)
  *ddr_l |= 0b00000011;

  //DC Motor
  //Set Enable, Direction A and B to Output (PH4,PH5,PH6)
  *ddr_h |= 0b01110000;

  //constant monitoring of State Button, interrupts when pressed (turn ON System) assumed off at start
  attachInterrupt(digitalPinToInterrupt(StateButton), StateButtonPressedISR, RISING);

  rtc.begin();
  //sets rtc to compile date and time
  rtc.adjust(DateTime(__DATE__, __TIME__));;

  //begins LCD to print Temp and Humidity
  lcd.begin(16, 2);

  ventStepperController.setSpeed(15);

}

void loop() {


  //delay(500);
  lcd.clear();

  if(previousStatePosition != "RUNNING") {
    
    //turn fan off if On
    if(*pin_h & (1<<4)) {*port_h &= ~((1 << 4));}

    if(*pin_h & (1<<5)) {
      recordEvent("FAN/MOTOR TURNED OFF");
      *port_h &= ~(1 << 5);
    }
  }

  //DISABLED STATE
  if(OnState == false) {

    recordEvent(DISABLED_MESSAGE);
    lcd.noDisplay();

    //turn off RED LED IF HIGH
    if(*pin_l & (1<<0)) {*port_l &= ~((1 << 0));}

    //if GREEN PB2 HIGH, turn LOW
    else if(*pin_b & (1<<2)) {*port_b &= ~(1 << 2);}

    //if BLUE PB3 HUGH< TURN LOW
    else if(*pin_b & (1<<3)) {*port_b &= ~((1 << 3));}

    //yellow LED On
    *port_l |= (1 << 1);
    //makeLEDHigh(*port_l,1);

    //Turn off Monitoring of Water Level and Temperature
    previousStatePosition = "DISABLED";

  }

  //assumed OnState
  else {

    //checks for Analog Data from Potionmeter for Vent Control
    potValueVent = adc_read(15);

    if(potValueVent > (prevPotValueVent + 10 )) {
      //move vent 6 steps clockwise
      controlVent(6);      
    }
    else if(potValueVent < (prevPotValueVent + 10)) {
      //move vent 6 steps counter-clockwise
      controlVent(-6);      
    }
    //sets prev to current for next iteration/comparision
    //prevPotValueVent = potValueVent;


    //TURN YELLOW LED OFF
    if(*pin_l & (1<<1)) {
       *port_l &= ~((1 << 1));
    }

    

    //display LCD again
    lcd.display();

    //Monitor Water Level (Using Water Sensor Module)
    //saves to WaterLevel
    *port_b |= (1 << 7);
    readWaterLevel();
    *port_b &= ~(1<<7);
    Serial.println(waterLevel);

    if(waterLevel <= waterLevelThreshold) {

      //if GREEN PB2 HIGH, turn LOW
      if(*pin_b & (1<<2)) {*port_b &= ~(1 << 2);}

      //if BLUE PB3 HUGH< TURN LOW
      else if(*pin_b & (1<<3)) {*port_b &= ~((1 << 3));}

      //RED LED ON (PL0)
      *port_l |= (1 << 0);

      //ERROR
      lcd.print("ERROR");
      lcd.setCursor(0, 1);      
      lcd.print("Water Level Low!");
      recordEvent(ERROR_MESSAGE);

      // if(resetButton) go back to IDLE

      previousStatePosition = "ERROR";

    }

    //IDLE (WaterLevel is Healthy) or RUNNING State
    else {

      //turn off RED LED IF HIGH
      if(*pin_l & (1<<0)) {*port_l &= ~((1 << 0));}

      //records current min new Temp/Humidity Data is read in
      DateTime timeNow = rtc.now();
      currentMin = timeNow.minute();

      if((currentMin != previousMin) | ((previousStatePosition == "DISABLED") | (previousStatePosition == "ERROR")))  {
        //updates timer to currentMin
        previousMin = currentMin;

        //Monitors Temp and Humidity, stored in DHT Instance
        readTempHumidity();
        
        recordEvent(tempHumidityMessage);
      }

      //Display Temp and Humidity to LCD
      Temp = DHT.temperature;;
      lcd.print("Temp: ");
      lcd.print(Temp);

      lcd.setCursor(0, 1);
      Humidity = int(DHT.humidity);
      lcd.print("Humidity: ");;
      lcd.print(Humidity);

      //RUNNING
      if(Temp > TempThreshold) {

        recordEvent(RUNNING_MESSAGE);

        //Run Fan (ENABLE AND DIRECTION A)
        *port_h |= (1 << 4);
        *port_h |= (1 << 5);

        //if GREEN PB2 HIGH, turn LOW
        if(*pin_b & (1<<2)) {
          *port_b &= ~(1 << 2);
        }


        //BLUE LED ON (PB3)
        *port_b |= (1 << 3);

        previousStatePosition = "RUNNING";     
      }

      else {
        //IDLE (WaterLevel is Healthy)
        recordEvent(IDLE_MESSAGE);

        //IF BLUE LED ON, TURN LOW
        if(*pin_b & (1<<3)) {
          *port_b &= ~((1 << 3));
        }

        //GREEN LED ON (PB2)
        *port_b |= (1 << 2);

        previousStatePosition = "IDLE";

      }
    }
  }
}

void readTempHumidity() {

  //reads in Temp and Humidity, stores in DHT instance
  int chck = DHT.read11(DHT_PIN);

}

//reads in water level and updates var change
void readWaterLevel() {
  //reads in current waterLevel from sensor, updates waterLevel
  waterLevel = adc_read(AnalogPinWaterReadIn);
}



void recordEvent(char* message) {

  //Using 
  //Display Time and Date of Change to Serial Port
  displayTime();

  Serial.print(" -- ");

  //Display Changes to Stepper Motor

  Serial.println(message);

}

void displayTime() {
  DateTime timeNow = rtc.now();
  Serial.print(timeNow.year(), DEC);
  U0putchar("/");
  Serial.print(timeNow.month(),DEC);
  U0putchar("/");
  Serial.print(timeNow.day(),DEC);
  U0putchar("/");
  Serial.print(timeNow.hour(),DEC);
  U0putchar("/");
  Serial.print(timeNow.minute(),DEC);
  U0putchar("/");
  Serial.print(timeNow.second());
}

//Interrupt to record that Button was Pressed, inverts the State of Button
void StateButtonPressedISR() {
  Serial.println("CHANGING ON/OFF");
  OnState = (!OnState);
}

void controlVent(int steps) {

  //RECORDS WHEN VENT IS MOVED
  recordEvent("VENT CONTROL ACTIVATED");
  ventStepperController.step(steps);  
  
}

void makeLEDHigh(volatile unsigned char* port, int pos) {
  *port |= (1 << pos);
}
void makeLEDLow(volatile unsigned char* port, int pos) {
  *port &= ~((1 << pos));
}


//Inits ADC to be able to read in characters
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

//READS in from adc and returns value
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


void U0init(int U0baud)
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
