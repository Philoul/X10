// #include <SoftwareSerial.h>  // For the TTL to RS232
//
// Debug Mode (command from RFLink message)
//  RFDEBUG = Full messsages : RFLink debug (interface PC - ESP) and CM11 Debug message (interface ESP - CM11A)
//  RFUDEBUG = Debug only interface ESP - CM11A
//  QRFDebug = Debug only interface PC - ESP

extern "C" {
    #include "user_interface.h"  // Required for wifi_station_connect() to work
}

#include <X10CM11A.h>

#include <ESP8266WiFi.h>
#define FPM_SLEEP_MAX_TIME 0xFFFFFFF

#define BUILDNR                         0x07                                    // shown in version
#define REVNR                           0x33                                    // shown in version and startup string
#define BAUD                           57600                                    // Baudrate for serial communication.

#define INPUT_COMMAND_SIZE                60                                    // Maximum number of characters that a command via serial can be.
#define PRINT_BUFFER_SIZE                 60                                    // Maximum number of characters that a command should print in one go via the print buffer.

#define SWITCH                            D8
#define SWITCH_PULL_UP                   false                                 // true if pull up input, false if pull down
#define SHORT_PRESS						2000

#define LED                               D3
#define TIMERSWITCH                     1000                                    // une seconde mini entre 2 appuis switch pour détection du front montant

void WiFiOn();
void WiFiOff();



//****************************************************************************************************************************************
void(*Reboot)(void)=0;                                                          // reset function on adress 0.
byte PKSequenceNumber=0;                                                        // 1 byte packet counter
boolean RFDebug=false;                                                          // debug X10 signals From CM11A 
boolean RFUDebug=false;                                                         // debug RF signals with plugin 254 
boolean QRFDebug=false;                                                         // debug RF signals with plugin 254 but no multiplication

// char pbuffer[PRINT_BUFFER_SIZE];                                                // Buffer for printing data
char InputBuffer_Serial[INPUT_COMMAND_SIZE];                                    // Buffer for Seriel data

void PrintHex8(uint8_t *data, uint8_t length);                                  // prototype
void PrintHexByte(uint8_t data);                                                // prototype
byte reverseBits(byte data);                                                    // prototype

// ===============================================================================



void setup() {
  WiFiOff();
  pinMode(SWITCH, INPUT_PULLUP);                                                       // Par défaut D8 est en Pull Down Switch pour changer entre mode RFLink et Mode passerelle CM11A => PC
  
  pinMode(LED, OUTPUT);                                                         // Activation sortie LED (built in ou externe pour repérer le mode
  digitalWrite(LED, LOW);                                                       // turn the LED on (LOW is the voltage level)
  
  Serial.begin(BAUD);                                                           // Initialise the serial port (Mode RFLink)
  
  CM11A.begin();                                                                // Initialise le CM11A et récupère le mode par défaut (Si RFLink c'est déjà initialisé, si CM11A, bascule à 4800Bds
  CM11A.ReceiveStd(CM11A_StdR);                                                 // Initialise la fonction appelée en cas de réception d'un ordre X10 sur le CM11A
  CM11A.ReceiveFastM(CM11A_FastM);                                              // Initialise la fonctions appelée en cas d'exécution d'une macro mémorisée dans le CM11A


  if(!CM11A.PCMode) {
    Serial.print(F("20;00;CM11A - RFLink Gateway V1.1 - "));
    sprintf(InputBuffer_Serial,"R%02x;",REVNR);
    Serial.println(InputBuffer_Serial); 
    PKSequenceNumber++;
  }
 
  digitalWrite(LED, CM11A.PCMode ? LOW : HIGH );                              // on affiche sur la LED l'état
  
}

void loop() {
  if(digitalRead(SWITCH) == !SWITCH_PULL_UP)
	SwitchPressed();
  CM11A.X10Debug = RFDebug || RFUDebug ;
  CM11A.handleCM11A();                                                        // scan des events sur le CM11A
  if(!CM11A.PCMode) {                                                         // Mode RFLink, scan les event sur le RFLink
    handleRFLink();                                                           // Scan for RFLink command
  }
}

/*********************************************************************************************/

void SwitchPressed() {
	unsigned long TimerSwitch=millis();                                   // Timer to keep focus on the task during communication
	while(digitalRead(SWITCH) == !SWITCH_PULL_UP) {
		if(millis()-TimerSwitch >= SHORT_PRESS) {
			digitalWrite(LED, !digitalRead(LED));                              // Flash de la LED d'état pour indiquer l'appui long
			delay(100);
			digitalWrite(LED, !digitalRead(LED));                              // Flash de la LED d'état pour indiquer l'appui long
			delay(100);
		} else
			delay(50);
	}
	
	if(millis()-TimerSwitch < SHORT_PRESS) {											// Short press, change mode
		if(RFDebug){
		  if(CM11A.PCMode) {                                                                // Mode avant changement
			Serial.println("Je quitte le mode passerelle PC CM11A 4800Bds");
			delay(100);
		  } else {
			Serial.println("Je quitte le mode RFLink à 57600Bds");
			delay(10);
		  }
		}
		CM11A.SetMode(!CM11A.PCMode , false);                                       // je change le mode actif (si PCMode = true, cela initialise le débit à 4800Bd sur la liaison série
		if(CM11A.PCMode) {                                                          // Mode après changement
			if(RFDebug) {
				Serial.println("Je démarre le mode passerelle PC CM11A 4800Bds");
				delay(100);
			}
		} else {
			Serial.begin(BAUD);                                                     // Je change le débit sur la liaison série
			if(RFDebug) {
				Serial.println("Je démarre le mode d'échange RFLink à 57600Bds");
				delay(10);
			}
		}
		digitalWrite(LED, CM11A.PCMode ? LOW : HIGH );                              // on affiche sur la LED l'état		
	} else {																		// Long Press
		CM11A.SetMode(CM11A.PCMode , true);                                       	// je change le mode actif (si PCMode = true, cela initialise le débit à 4800Bd sur la liaison série
	}
}
