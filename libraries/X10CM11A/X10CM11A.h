/*
Name:		X10CM11A.h
Created:	18/04/2018 13:49:40
Author:		JP LEGOUPIL
for ESP8266
*/

#ifndef _X10CM11A_h
#define _X10CM11A_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <EEPROM.h>
#include <SoftwareSerial.h>  			// For the TTL to RS232 to CM11A
#include "EEPROM_map.h"
#include "TimeLib.h"


/*
* This macro must be defined even if you are using a software serial
* port. You can change this to any serial port supported by your
* Arduino (i.e, Serial1, Serial2, etc.)
*/
#define X10_HW_SERIAL_PORT Serial  		// For the TTL to RS232 to CM11A
#define X10_HW_PC_SERIAL_PORT Serial  	// For CM11A emulation to PC
#define X10_DEBUG_SERIAL_PORT Serial  	// For printing debug information

#define X10_CM11A_TX	D1
#define X10_CM11A_RX	D2



/*
* Optional macros, define as needed
*/
#define X10_SOFTWARE_SERIAL
//#define X10_PC_SOFTWARE_SERIAL


// Don't change anything above this line



// Bits for X10 Definition
#define ISDIMMER		0x80		// set = dimmer, reset = appliance (default)
#define ISEXTENDED		0x40		// set allow extended message (0x31 command)
#define	DIM-ON			0x20		// set if when Off, a Dim function is a full bright function
#define DIM-RESUME		0x10		// set if when Off, a Dim function is a resume
#define BRIGHT-ON		0x08		// set if when Off, a Bright function is a full bright function
#define	BRIGHT-RESUME	0x04		// set if when Off, a Bright function is a resume 
#define ON-RESUME		0x02		// set if when Off, a On function is a resume (and not a full bright)
#define FULLDIM-ON		0x01		// set if when full Dim, the Lamp status remains ON


#define X10BAUD       4800    		// Baudrate for X10serial communication.
#define TIMEOUTDELAY  1500   		// 
#define DELAY         750     		// 750 ms to 2000 ms delay after first three bytes are sent
#define SHORT_DELAY   20      		// 10 ms delay waiting for checksum before sending the next three bytes
#define STATUS_DELAY  60000 		// 1 hour delay between updated status
#define POLLING_DELAY  3600000 		// 1 heure delay between polling CM11A if not alive
#define COMMIT_DELAY	30000		// 30s delay after last EEPROM sent from PC to CM11A to commit ESP EEPROM



#define X10CM11A_BUFFER_SIZE 20  	// Maximum number of characters that can be sent by CM11A.
#define X10CM11A_STATUS_SIZE 14  	// Number of bytes request for CM11A Status.
#define X10CM11A_SET_TIME_SIZE 7  	// Number of bytes of CM11A Set Time command.
#define X10CM11A_UPLOAD_EEPROM_SIZE 18 // Number of bytes for an EEPROM upload command
#define X10CM11A_UPLOAD_EEPROM_DELAY 10000 // Specific DELAY for an EEPROM upload command (> 3s constaté)
#define MAXIT			3			// Nb de tentatives d'envois avant arrèt

#define X10_MODE true

#define _X10Debug X10_DEBUG_SERIAL_PORT

// Function Code
#define ALL_UNITS_OFF     0x00
#define ALL_LIGHTS_ON     0x01
#define ON                0x02
#define OFF               0x03
#define DIM               0x04
#define BRIGHT            0x05
#define ALL_LIGHTS_OFF    0x06
#define EXTENDED_CODE     0x07
#define HAIL_REQUEST      0x08
#define HAIL_ACKNOWLEDGE  0x09
#define PRE_SET_DIM_1     0x0A
#define PRE_SET_DIM_2     0x0B
#define EXTENDED_DATA     0x0C
#define STATUS_ON         0x0D
#define STATUS_OFF        0x0E
#define STATUS_REQUEST    0x0F

// Nombre d'alias différents pris en compte pour l'interprétation des commande X10
#define NBLIBX10          4



class X10CM11A
{
public:
	X10CM11A(int TXpin, int RXpin);
	X10CM11A();
	~X10CM11A();
	void begin();
#ifdef X10_PC_SOFTWARE_SERIAL
	void beginPC(int RXpin, int TXpin);
#else
	void beginPC();
#endif
	byte HouseCode(char* StrHouseCode);											// Converti un char * (A-P ou a-p) en byte "code house" X10 : A -> 0x60 , B -> 0xE0 , ... , P -> 0xC0 , invalide = 0xFF
	byte UnitCode(char* StrUnitCode);											// Converti un char * (1-16) en bytr "code unit" X10 : 1 -> 0x06 , 2 -> 0x0E , ... , 16 -> 0x0C , invalide = 0xFF
	byte FunctionCode(char* StrFunctionCode);									// Converti un char * (ALL_UNIT_OFF, ALLON, ON, OFF, DIM ...) en byte "code function" X10 : ALLOFF -> 0x00 , ON -> 0x02 , DIM -> 0x04, ... , invalide = 0xFF
	boolean DevCode(char* StrDevCode, byte* DevCode); 							// Converti un char * (A1, C6, ... , P16) en byte* DevCode ( ex A2 -> 0x6E ). Return true si Ok, false si Ko
	boolean SendDevice(byte Housecode_Devicecode);								// Envoi un Device code au CM11A (donc sur le courant porteur
	boolean SendBitmap( byte hcode, unsigned int bitmap );
	boolean SendFunction(byte housefunctioncode, byte luminance); 				// housefunctioncode = housecode + functioncode (ex A ON -> 0x62) , Luminance = byte (entre 0 et 22)
	boolean SendFunction(byte housecode, byte functioncode, byte luminance);	// housecode (ex 0x60 pour 'A'), functioncode (ex 0x02 pour 'ON', 0x04 pour 'DIM'), Luminance = byte (entre 0 et 22 , 22 = 100%)
	boolean SendExtFunction(byte devcode, byte data, byte extfunction);
	boolean SendEEPROMData(byte* data);											// Send 18 bytes in buffer data to CM11A EEPROM (2 bytes address + 16 bytes data) + Acknowledge
	void DecodeFastMacro ( unsigned int Fast_Macro_Addr );						// Decode immediate fast_macro (call DecodeBlocFastMacro) or set timer for delayed fast macro
	void DecodeBlocFastMacro ( unsigned int Fast_Macro_Addr , boolean chained );// Decode a fastmacro address block (if chained the timer is set for the following block) and then call _Std function for each command of fast macro
	
	boolean SetClock(byte second, byte minutes, byte hour, int yearday, byte daymask, byte HouseCode, byte TMB );
//	boolean SetClock(byte second, byte minutes, byte hour, int yearday, byte daymask, byte HouseCode );
	boolean StatusRequest();													// Send a status request to CM11A
	boolean EnableRing(boolean ER);												// enable or disable ring to CM11A
	void handleCM11A();
	void handlePC();
	void UpdateCurrentDevice( byte code, boolean fct );							// update current selected address and house code
	void ReceiveStd(void (* fn)()) ;											// set used defined standard function on received X10 command
	void ReceiveFastM(void (* fn)()) ;											// set user defined fastmacro function on received X10 Fastmacro address
	void SetMode(boolean _PCMode, boolean _DefaultMode);												// set ESP mode (X10_MODE or RFLINK_MODE)
	boolean SetYear(unsigned int  _year);
	
	unsigned char Buffer_Serial[X10CM11A_BUFFER_SIZE];                              // Buffer for Serial data
	unsigned char PCBuffer[X10CM11A_BUFFER_SIZE];                                	// Buffer for PC data
	int Buffer_Size;
	int PCBuffer_Size;
	
	byte CurrentHC;				// Current Addressed House Code (High Nibble, Low Nibble = 0 Last message = Address or 1 = Last message = function)
	byte CurrentFC;				// Current Function Code (Low Nibble, High Nibble = 1 if active, 0 if treated)
	byte CurrentData;			// Current Data for Extended functions (0-FF) or for Dim / Bright Functions (0-3F)
	byte CurrentCommand;		// Current Command for Extended functions
	unsigned int CurrentUnits;	// Current active Units (Bitmap)

	int Status_Size;
	
	unsigned int Fast_Macro;
	unsigned int FastMacroAddress;
	unsigned long FastMacroTime = 0;
	unsigned long FastMacroDelay = 0;
	boolean FastMacroWaiting = false;
	

	
	byte Fast_Macro_Init ;
	boolean X10Debug=false;
	boolean PCMode=false;
	boolean CM11A_Ok = false;	// true if data received from CM11A or if response to request from ESP
	boolean PC_Ok = false;		// true if in PCMode (not RFLink Mode), if command received from PC or response to request from PC
	const char* FunctionC[16][NBLIBX10] =  {{"ALL_UNITS_OFF"   ,"ALLUOFF"    ,"ALL UNITS OFF"    ,"ALLOFF"      },
											{"ALL_LIGHTS_ON"   ,"ALLLON"     ,"ALL LIGHTS ON"    ,"ALLON"       },
											{"ON"              ,"ON"         ,"ON"               ,"ON"          },
											{"OFF"             ,"OFF"        ,"OFF"              ,"OFF"         },
											{"DIM"             ,"DIM"        ,"DIM"              ,"DIM"         },
											{"BRIGHT"          ,"BRIGHT"     ,"BRIGHT"           ,"BRIGHT"      },
											{"ALL_LIGHTS_OFF"  ,"ALLLOFF"    ,"ALL LIGHTS OFF"   ,"ALLOFF"      },
											{"EXTENDED_CODE"   ,"EXTENDCODE" ,"EXTENDED CODE"    ,""            },
											{"HAIL_REQUEST"    ,"HAILREQ"    ,"HAIL REQUEST"     ,"HAIL REQ"    },
											{"HAIL_ACKNOWLEDGE","HAILACK"    ,"HAIL ACKNOWLEDGE" ,"HAIL ACK"    },
											{"PRE_SET_DIM_1"   ,"PDIML"      ,"PRESET DIM LOW"   ,""            },
											{"PRE_SET_DIM_2"   ,"PDIMH"      ,"Preset DIM HIGH"  ,""            },
											{"EXTENDED_DATA"   ,"EXTENDDATA" ,"EXTENDED DATA"    ,""            },
											{"STATUS_ON"       ,"STATON"     ,"STATUS ON"        ,"STATUS=ON"   },
											{"STATUS_OFF"      ,"STATOFF"    ,"STATUS OFF"       ,"STATUS=OFF"  },
											{"STATUS_REQUEST"  ,"STATREQ"    ,"STATUS REQUEST"   ,"STATUS REQ"  } }; 	
	const byte HouseC[16] = { 'M','E','C','K','O','G','A','I','N','F','D','L','P','H','B','J' };
	const byte UnitC[16] = { 13 , 5 , 3 , 11, 15, 7 , 1 , 9 , 14, 6 , 4 , 12, 16, 8 , 2 , 10 };



private:
	boolean Acknowledge(byte CheckSum);
	boolean AcknowledgePC(byte CheckSum);
	boolean AwakePC();
	boolean ReadyToReceive();
	void X10serialPCFlush();
	boolean SendBuffer ( unsigned char *buffer, int length, int chkoff );
	void X10serialFlush();    
	byte checksum( unsigned char *buffer, int length ); 
	void upstr(char *s);
	void PrintHexByte(uint8_t data);
	int _TXpin;
	int _RXpin;	
	unsigned char _CM11AStatus[X10CM11A_STATUS_SIZE];                                // Buffer CM11A StatusRequest
	unsigned long _TimeRequest = 0; 
	unsigned long _TimePolling = 0; 
	unsigned long _TimePollingPC= 0; 
	unsigned long _TimeEEPROM = 0; 	
	boolean	_RequestCommit = false;
	void UpdateTime();
	
	unsigned int BatteryTimer = 0;
	
	byte CurrentTimeS;
	byte CurrentTimeM;
	byte CurrentTimeH;
	unsigned int CurrentYearDay;
	unsigned int CurrentYear;
	byte DayMask; 						// (SMTWTFS)
	
	
	byte MonitoredHouseCode;
	byte CurrentVersion;
	unsigned long _TimeMillis = 0;
	
	unsigned int CurrentMonitoredUnits;
	unsigned int OnOffMonitoredUnits;
	unsigned int DimMonitoredUnits;

	
#ifdef X10_PC_SOFTWARE_SERIAL
	SoftwareSerial _X10PCSerial;
#else
	#define _X10PCSerial X10_HW_PC_SERIAL_PORT
#endif

#ifdef X10_SOFTWARE_SERIAL
	SoftwareSerial _X10Serial;
#else
	#define _X10Serial X10_HW_SERIAL_PORT
#endif

	void (* _StdR)();
	void (* _FastM)();
	byte _MemDev[255];
};


static X10CM11A CM11A(X10_CM11A_RX, X10_CM11A_TX);  // RX, TX to TTL RS232

#endif
