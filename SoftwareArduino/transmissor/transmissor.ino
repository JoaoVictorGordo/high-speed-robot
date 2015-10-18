/* 
 * by João Gordo, based on:
 *            YourDuinoStarter Joystick Example: nRF24L01 Transmit  values
 - WHAT IT DOES: 
 - CONNECTIONS: nRF24L01 Modules See:
 http://arduino-info.wikispaces.com/Nrf24L01-2.4GHz-HowTo
   1 - GND
   2 - VCC 3.3V !!! NOT 5V
   3 - CE to Arduino pin 9
   4 - CSN to Arduino pin 10
   5 - SCK to Arduino pin 13
   6 - MOSI to Arduino pin 11
   7 - MISO to Arduino pin 12
   8 - UNUSED

/*-----( Import needed libraries )-----*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
/*-----( Declare Constants and Pin Numbers )-----*/
#define CE_PIN   9  
#define CSN_PIN 10
//#define JOYSTICK_X A0
//#define JOYSTICK_Y A1

// NOTE: the "LL" at the end of the constant is "LongLong" type
const uint64_t pipe = 0xE8E8F0F0E1LL; // Define the transmit pipe

char s;
char msg[] = "OK"; 
int count = 0;


/*-----( Declare objects )-----*/
RF24 radio(CE_PIN, CSN_PIN); // Create a Radio
/*-----( Declare Variables )-----*/
int joystick[2];  // 2 element array holding Joystick readings

void setup()   /****** SETUP: RUNS ONCE ******/
{
  Serial.begin(57600);
  radio.begin();
  radio.openWritingPipe(pipe);
}//--(end setup )---

void loop()   /****** LOOP: RUNS CONSTANTLY ******/
{
  joystick[0] = 11;  //analogRead(JOYSTICK_X);
  joystick[1] = 23;  //analogRead(JOYSTICK_Y);

    //  s = msg[count];
     // count = (count + 1)%strlen(msg);
  radio.write( msg, sizeof(msg) );

//pro - uno
/*
      Serial.println(radio.isPVariant()); // desligado: 0 ; ligado 0 . desligado 0 ligado 1
      Serial.println(radio.getDataRate()); // desligado: 0 ; ligado 0 . desligado 0 ligado 0
      Serial.println(radio.getPayloadSize()); // desligado: 32 ;  ligado 32 . desligado 32 ligado 32
      Serial.println(radio.getPALevel()); // desligado: 0 ; ligado 3 . desligado 0 ligado 3
      Serial.println();
*/

}//--(end main loop )---

/*-----( Declare User-written Functions )-----*/

//NONE
//*********( THE END )***********

