/*
Name:		X10CM11A.cpp
Created:	18/04/2018 12:04:02
Author:	PC
Editor:	http://www.visualmicro.com
*/

#include <SoftwareSerial.h>  // For the TTL to RS232
#include "X10CM11A.h"



const byte _DHCmap[16] =  { 0x06 , 0x0E , 0x02 , 0x0A , 0x01 , 0x09 , 0x05 , 0x0D ,
							0x07 , 0x0F , 0x03 , 0x0B , 0x00 , 0x08 , 0x04 , 0x0C };
							




#ifdef X10_SOFTWARE_SERIAL
X10CM11A::X10CM11A(int RXpin, int TXpin):_X10Serial(RXpin , TXpin)
#else
//#define _X10Serial X10_HW_SERIAL_PORT
X10CM11A::X10CM11A()
#endif
{
#ifdef X10_SOFTWARE_SERIAL
	_RXpin=RXpin;
	_TXpin=TXpin;
#endif

	_StdR = NULL;
	_FastM = NULL;
}

X10CM11A::~X10CM11A() 
{}

void X10CM11A::begin()
{
/*
	pinMode(_TXpin, OUTPUT);
	pinMode(_RXpin, OUTPUT); 
	digitalWrite(_TXpin, HIGH);			// to enable CM11A reception after a reboot	TX VCC
	digitalWrite(_RXpin, LOW);			// to enable CM11A reception after a reboot RX GND 2k Pull down resistor added
	delay(100); */
	// pinMode(_RXpin, INPUT);  
	
	_X10Serial.begin(4800);    			// X10 Baud 4800 8,NONE,1
	EEPROM.begin(EEPROM_SIZE);			// 
	PCMode= EEPROM.read(EEPROM_PCMODE);
	EEPROM.get(EEPROM_YEAR,CurrentYear);
	SetMode(PCMode,false);

	delay(5);
}


#ifdef X10_PC_SOFTWARE_SERIAL
void X10CM11A::beginPC(int RXpin, int TXpin)
{
	_X10PCSerial(RXpin , TXpin);
	_X10PCSerial.begin(4800);    // X10 Baud 4800 8,NONE,1
}
#else
void X10CM11A::beginPC()
{
	_X10PCSerial.begin(4800);    // X10 Baud 4800 8,NONE,1	
}
#endif


byte X10CM11A::HouseCode(char* StrHouseCode)
{ //  "0x41", "A", "a"
	int HC = 0;
	if (strncmp(StrHouseCode,"0x",2)==0) {
		StrHouseCode[0] = strtoul(StrHouseCode,NULL,0);       // Home: A..P
		StrHouseCode[1] = 0 ;
	}
	upstr(StrHouseCode);
	if (StrHouseCode[0] >= 'A' && StrHouseCode[0] <= 'P') { // upper case between A & P
		HC=StrHouseCode[0] - 'A';
		if(X10Debug) {
			_X10Debug.print("HouseCode : ");
			PrintHexByte(_DHCmap[HC]);
			_X10Debug.println("");
		}		 
	}
	if (HC < 16 ) {
		return (_DHCmap[HC] << 4) ;
	} 
	else {
		return (byte)0xFF;                    // invalid value
	}
}

byte X10CM11A::UnitCode(char* StrUnitCode)
{ // "1", "16" StrUnitCode
	int Uc = 0;

	Uc = strtoul(StrUnitCode,NULL,0);

		if(X10Debug) {
			_X10Debug.print("unitcode value : ");
			PrintHexByte(Uc);
			_X10Debug.println("");
		}		

	if (Uc > 0 && Uc <= 16 ) {                       // UnitCode 1 - 16
		if(X10Debug){
			_X10Debug.print("UnitCode : ");
			PrintHexByte(_DHCmap[Uc-1]);
			_X10Debug.println("");
		}
		return _DHCmap[Uc-1];
	} else {
		return (byte)0xFF;                    // invalid value
	}
}

byte X10CM11A::FunctionCode(char* StrFunctionCode)
{ // "ON", "All Units Off", "Dim" StrFunctionCode must be terminated by '\0', 4 aliases per function in FunctionC Array
	upstr(StrFunctionCode); // Upper case convertion
	int Fc = (byte)0xFF;
	byte _idx = 0;
	for(_idx=0;_idx<16;_idx++)
		for(int _idx2=0;_idx2<NBLIBX10;_idx2++)
			if(strcmp(StrFunctionCode,FunctionC[_idx][_idx2])==0)
				Fc = _idx;
	
	if(X10Debug){
		_X10Debug.print("Function Code : ");
		PrintHexByte(Fc);
		_X10Debug.println("");
		delay(10);
	}	
	
	return Fc;
}


boolean X10CM11A::DevCode(char* StrDevCode, byte* DevCode)
{ // "A", "B1"
	byte HC = HouseCode(StrDevCode);

	byte UC = 0;
	if(StrDevCode[1] != 0)
		UC = UnitCode(StrDevCode+1);
	if(HC !=(byte)0xFF && UC !=(byte)0xFF)
	{
		*DevCode = HC | UC;  
		return true;
	} 
	else
		return false;
}

boolean X10CM11A::SendDevice(byte devcode) {
	byte Checksum = ((0x04 + devcode)&0xFF);         // CHeksum for CM11 acknowledge
	boolean AckOk = false;
	char msgbuff[80];
	int Iteration = 0;
	unsigned long previousmillis = millis();
	while( millis() - previousmillis < TIMEOUTDELAY && !AckOk ) {
		Iteration++;
		_X10Serial.write((byte)0x04);                      // start code
		_X10Serial.write(devcode);                      // device code and (house | unit) code
		if(X10Debug){
			_X10Debug.println("Send Device to CM11A :");
			PrintHexByte((byte)0x04 );
			PrintHexByte(devcode);
			sprintf(msgbuff, "            Sending address bytes: %c%d", HouseC[devcode>>4], UnitC[devcode&0x0F]);
			_X10Debug.println(msgbuff);
//			_X10Debug.println("");
			delay(10);
			
		}		
		AckOk = Acknowledge(Checksum);
	}

	if (!AckOk) {
		return false;                                 // No Acknowledge or bad checksum received several times
	} else {
		UpdateCurrentDevice( devcode, false );
		return true;                                  // Correct Checksum received from CM11A
	}
}

boolean X10CM11A::SendFunction(byte housefunctioncode, byte luminance) {
	byte Checksum = (((byte)0x06 + (luminance << 3) + housefunctioncode)&0xFF);
	boolean AckOk = false;
	char msgbuff[80];
	int Iteration = 0;
	unsigned long previousmillis = millis();
	while( millis() - previousmillis < TIMEOUTDELAY && !AckOk ) {
		Iteration++;
		_X10Serial.write((byte)0x06 + (luminance << 3));  // start code plus bright or dim value. luminance value is shiftbit left by 3
		_X10Serial.write((byte)housefunctioncode);       // house and function code. on/off dim/bright
		if(X10Debug){
			_X10Debug.println("Send Function to CM11A :");
			PrintHexByte((byte)0x06 + (luminance << 3));
			PrintHexByte((byte)housefunctioncode);
			sprintf(msgbuff, "            Sending function bytes: %c %d ", HouseC[housefunctioncode>>4], FunctionC[housefunctioncode&0x0F][0]);
			_X10Debug.print(msgbuff);
			if (luminance == 0)
				_X10Debug.println("");
			else
				_X10Debug.println( (int) (luminance * 100 / 22));
			delay(10);
		}
		
		AckOk = Acknowledge(Checksum);
	}

	if (!AckOk) {
		return false;                                 // No Acknowledge or bad checksum received several times
	} else {
		UpdateCurrentDevice( housefunctioncode, true );
		return true;                                  // Correct Checksum received from CM11A
	}
}

boolean X10CM11A::SendFunction(byte housecode, byte functioncode, byte luminance) {
	byte Checksum = (((byte)0x06 + (luminance << 3) + housecode + functioncode)&0xFF);
	boolean AckOk = false;
	char msgbuff[80];
	int Iteration = 0;
	unsigned long previousmillis = millis();
	while( millis() - previousmillis < TIMEOUTDELAY && !AckOk && Iteration < MAXIT ) {
		Iteration++;
		_X10Serial.write((byte)0x06 + (luminance << 3));  // start code plus bright or dim value. luminance value is shiftbit left by 3
		_X10Serial.write((byte) (housecode + functioncode));       // house and function code. on/off dim/bright
		if(X10Debug){
			_X10Debug.println("Send Function to CM11A :");
			PrintHexByte((byte)0x06 + (luminance << 3));
			PrintHexByte(housecode + functioncode);
			sprintf(msgbuff, "            Sending function bytes: %c %d ", HouseC[housecode>>4], FunctionC[functioncode&0x0F][0]);
			_X10Debug.print(msgbuff);
			if (luminance == 0)
				_X10Debug.println("");
			else
				_X10Debug.println( (int) (luminance * 100 / 22));
			delay(10);
		}
		
		AckOk = Acknowledge(Checksum);
		
		if (!AckOk)
			delay(SHORT_DELAY);
	}
	if (!AckOk) {
		return false;                                 // No Acknowledge or bad checksum received several times
	} else {
		UpdateCurrentDevice( housecode + functioncode , true );
		return true;                                  // Correct Checksum received from CM11A
	}
}



boolean X10CM11A::SendExtFunction(byte devcode, byte data, byte extfunction) {
	byte function = ((byte)0xF0 & devcode) | (byte)0x07 ;
	byte unit = ((byte)0x0F & devcode);
	byte Checksum = ((byte)0x07 + function + unit + data + extfunction)&0xFF;
	boolean AckOk = false;
	int Iteration = 0;
	unsigned long previousmillis = millis();
	while( millis() - previousmillis < TIMEOUTDELAY && !AckOk && Iteration < MAXIT ) {
		Iteration++;
		_X10Serial.write((byte)0x07 );             // start code for extended function
		_X10Serial.write(function);                 // housecode high nybble, 7 low nibble
		_X10Serial.write(unit);                     // unitcode low nybble
		_X10Serial.write(data);                     // data (any value between 0x00 and 0xFF)
		_X10Serial.write(extfunction);              // extfunction
		if(X10Debug){
			_X10Debug.println("Send Extended Function to CM11A :");
			_X10Debug.print("07");
			PrintHexByte(function);
			PrintHexByte(unit);
			PrintHexByte(data);
			PrintHexByte(extfunction);
			_X10Debug.println("");
			delay(10);
		}
		AckOk = Acknowledge(Checksum);
		if (!AckOk)
			delay(SHORT_DELAY);
		
	}

	if (!AckOk) {
		return false;                                 // No Acknowledge or bad checksum received several times
	} else {
		CurrentHC = ((byte)0xF0 & devcode) | 1 ;
		CurrentUnits = 1 << unit;
		return true;                                  // Correct Checksum received from CM11A
	}
}

boolean X10CM11A::StatusRequest(){
	boolean ReceivedData = false;
	unsigned long previousmillis = millis();
	unsigned long getTime = 0;
	Status_Size=0;
	char msgbuff[80];
//	X10serialFlush();                           // Clear Input buffer
	_X10Serial.write((byte)0x8B );             // start code for CM11A Status Request
	if(X10Debug){
		_X10Debug.println();
		sprintf(msgbuff, "%02d/%02d/%d %02d:%02d:%02d ", day(), month(), year(), hour(), minute(),second());
		_X10Debug.print(msgbuff);
		_X10Debug.println("Send Status request to CM11A :");
		_X10Debug.println("8B");
	}
	while( millis() - previousmillis < TIMEOUTDELAY && !ReceivedData ) {
		if (_X10Serial.available()) {
			if(getTime == 0)
				getTime == millis();			// reference time get on first byte received
			_CM11AStatus[Status_Size++] = _X10Serial.read();
			if (Status_Size == X10CM11A_STATUS_SIZE )
				ReceivedData = true;				
		} else delay(SHORT_DELAY);                          // short delay to wait for data from Interface
	}
	
	_X10Serial.write((byte)0x00 );				// ActiveHome send 0x00 after status request answer or not
	
	if(ReceivedData) {
		_TimeRequest = getTime;
		_TimeMillis = getTime;
		
		BatteryTimer = (_CM11AStatus[0]<<8) + _CM11AStatus[1];
		CurrentTimeS = _CM11AStatus[2];
		CurrentTimeM = _CM11AStatus[3];
		CurrentTimeH = _CM11AStatus[4];
		CurrentYearDay = ((_CM11AStatus[6] & 0x80)<<1)+ _CM11AStatus[5];
		DayMask = _CM11AStatus[6] & 0x7F; 						// (SMTWTFS)
		if(X10Debug){
			sprintf(msgbuff, "Avant Set time\nCurrent Year : %d \nYear : %d", CurrentYear, year());
			_X10Debug.println(msgbuff);	
		}			
		setTime(CurrentTimeH*2+CurrentTimeM/60,CurrentTimeM%60,CurrentTimeS, CurrentYearDay, CurrentYear);
		setTime(CurrentTimeH*2+CurrentTimeM/60,CurrentTimeM%60,CurrentTimeS, 30, 6, CurrentYear);

		if(CurrentYear!=year()) {
		if(X10Debug){
			sprintf(msgbuff, "Après set time\nCurrent Year : %d \nYear : %d", CurrentYear, year());
			_X10Debug.println(msgbuff);	
		}			
			CurrentYear=year();
		}
		
		MonitoredHouseCode = _CM11AStatus[7] & 0xF0;
		CurrentVersion = _CM11AStatus[7] & 0x0F;
		CurrentMonitoredUnits = (_CM11AStatus[8]<<8) + _CM11AStatus[9];
		OnOffMonitoredUnits = (_CM11AStatus[10]<<8) + _CM11AStatus[11];
		DimMonitoredUnits = (_CM11AStatus[12]<<8) + _CM11AStatus[13];
		
		if(CurrentYearDay != EEPROM.read(EEPROM_CM11A_STAT + 5)) {	// Status sauvegardé en EEPROM si changement de jour
			_TimeEEPROM = millis();
			_RequestCommit = true;
		}
		for( int i = 0 ; i < X10CM11A_STATUS_SIZE ; i++)
			EEPROM.write(EEPROM_CM11A_STAT + i , _CM11AStatus[i]) ;
		
		if(X10Debug){
			_X10Debug.print("      ");
			for(int i = 0; i < Status_Size ; i++)
				PrintHexByte((byte)_CM11AStatus[i]);
			_X10Debug.println("");
			sprintf(msgbuff, "%02d/%02d/%d %02d:%02d:%02d", day(), month(), year(), hour(), minute(),second());
			_X10Debug.println(msgbuff);						
			_X10Debug.println("");
		}
		return true;
	} else {
		return false;
	}
}



boolean X10CM11A::SendEEPROMData(byte* data) {
	byte Checksum = 0;
	boolean AckOk = false;
	int Iteration = 0;

	unsigned long previousmillis = millis();
	while( millis() - previousmillis < TIMEOUTDELAY && !AckOk && Iteration < MAXIT ) {
		Iteration++;
		X10serialFlush();                       // Clear Input buffer
		_X10Serial.write(0xFB);                 // Send EEPROM command
		_X10Serial.write(data , X10CM11A_UPLOAD_EEPROM_SIZE );           // Address Low B

		Checksum = checksum( data , X10CM11A_UPLOAD_EEPROM_SIZE );
		AckOk = Acknowledge(Checksum);
		if (!AckOk)
			delay(SHORT_DELAY);
	}

	return AckOk; 
}


boolean X10CM11A::SetClock(byte second, byte minutes, byte hour, int yearday, byte daymask, byte HouseCode, byte TMB ) {
	_X10Serial.write((byte)0x9B);                               // start code for timer download
	_X10Serial.write(second);                                   // seconds
	_X10Serial.write(minutes + 60*(hour&1));                    // minutes (0 à 199)
	_X10Serial.write(hour >> 1 );                               // hours / 2 (0 à 11)
	_X10Serial.write((byte)yearday );                           // 8 low bits of yearday (really 9 bits)
	_X10Serial.write( daymask | ( yearday & 0x100 ) >> 1 ) ;    // High bit of year day added to day mask (SMTWTFS)
	_X10Serial.write(HouseCode | TMB );                         // High nibble House Code, bits of Low nibble  : 0:Timerpurge, 1:Monitor clear, 3:battery clear
	_X10Serial.write((byte)0x00);                               // To enable CM11A to receive data even if Acknowledge problem
}


boolean X10CM11A::EnableRing(boolean ER) {
	byte _ER = ER ? 0xEB : 0xDB;
	boolean AckOk = false;
	int Iteration = 0;
	unsigned long previousmillis = millis();
	char msgbuff[80];
	if(X10Debug){
		_X10Debug.println();
		sprintf(msgbuff, "%02d/%02d/%d %02d:%02d:%02d ", day(), month(), year(), hour(), minute(),second());
		_X10Debug.print(msgbuff);
		_X10Debug.println(ER ? "Send EnableRing to CM11A :" : "Send DisableRing to CM11A :");
		_X10Debug.println(ER ? "EB" : "DB");
	}	
	while( millis() - previousmillis < TIMEOUTDELAY && !AckOk && Iteration < MAXIT ) {
		Iteration++;
		_X10Serial.write(_ER);             						// start code for extended function
		if(X10Debug){	
			_X10Debug.print("Essai N° ");
			_X10Debug.println(Iteration);
		}
		AckOk = Acknowledge(_ER);
		if (!AckOk)
			delay(SHORT_DELAY);
	}
	return AckOk; 
}

void X10CM11A::handleCM11A() {
	byte FirstByte = 0;
	Buffer_Size = 0;
	PCBuffer_Size = 0;
	boolean ReceivedData = false;
	boolean ReceivedPCData = false;
	boolean AckPC = false;
	char msgbuff[80];
	
	if(CurrentYearDay==0 && _TimePolling == 0)	{						// Initialisationde CurrentYearDay au démarrage
		CurrentYearDay = EEPROM.read(EEPROM_CM11A_STAT+5);
		MonitoredHouseCode = EEPROM.read(EEPROM_CM11A_STAT+7) & 0xF0;;
	}
	
	
	for(int intcpt=0;intcpt < X10CM11A_BUFFER_SIZE;intcpt++) {
		Buffer_Serial[intcpt]=0;
		PCBuffer[intcpt]=0;
	}
	Fast_Macro = 0;
	unsigned long previousmillis = millis();
	
	if (_X10Serial.available() > 0 ) {
		Buffer_Serial[Buffer_Size++]= _X10Serial.read();
		FirstByte = Buffer_Serial[0];
		if(X10Debug) {
			sprintf(msgbuff, "%02d/%02d/%d %02d:%02d:%02d ", day(), month(), year(), hour(), minute(),second());
			_X10Debug.print(msgbuff);
			_X10Debug.println("Receiving CM11A data:");
			delay(10);
		}
		CM11A_Ok = true;										// CM11A Alive he sends data to ESP...
		switch (FirstByte) {
		case ((byte)0x5A):          							// Polling. this happens after remote button has been pressed.
			delay(SHORT_DELAY);                  				// wait 20 ms before sending Acknowledge
			_X10Serial.write((byte)0xC3);   					// VERY IMPORTANT : Acknowledge , This must be done or it will not accept any other x10 commands

			if(X10Debug) {
				sprintf(msgbuff, "%02d/%02d/%d %02d:%02d:%02d ", day(), month(), year(), hour(), minute(),second());
				_X10Debug.print(msgbuff);
				_X10Debug.println("Awake ESP before Acknowledge CM11A:");
				_X10Debug.println("      5A");
				_X10Debug.println("C3              Acknowledge from ESP");
				delay(10);
			}
			
			if(PCMode)											// Awake PC before sending CM11A data to be received by ESP
				AwakePC();

			while(millis() - previousmillis < TIMEOUTDELAY && !ReceivedData ) {		// waiting for data from CM11A
				if (_X10Serial.available()) {
					handleCM11A();									// Relance la commande pour traiter les données supplémentaires envoyées par le CM11A
					ReceivedData = true;				
				} else delay(SHORT_DELAY);                          // short delay to wait for data from Interface
			}       

			break;
		case ((byte)0x5B):          								// This happens after remote wireless has received a fast macro (sending EEPROM address).
			// received 2 bytes EEPROM addresss from CM11A
			while(millis() - previousmillis < TIMEOUTDELAY && !ReceivedData ) {
				if (_X10Serial.available()>=2) {
					Buffer_Serial[Buffer_Size++]= _X10Serial.read();
					Buffer_Serial[Buffer_Size++]= _X10Serial.read();
					Fast_Macro = ( (Buffer_Serial[1] & 0x03) << 8 ) + Buffer_Serial[2] ;
					ReceivedData = true;
				} else delay(SHORT_DELAY);							// short delay to wait for response from Interface
			}
			Fast_Macro_Init = (Buffer_Serial[1] >> 4 ) & 0x07;
			if(X10Debug) {
				sprintf(msgbuff, "%02d/%02d/%d %02d:%02d:%02d ", day(), month(), year(), hour(), minute(),second());
				_X10Debug.print(msgbuff);
				_X10Debug.println("Receive macro EEPROM Address from CM11A:");
				_X10Debug.print("      5B        " );
				_X10Debug.println( Fast_Macro_Init == 0 ? "Timer" : "Macro initiator" );
				_X10Debug.print("      ");
				PrintHexByte(Buffer_Serial[1]);
				PrintHexByte(Buffer_Serial[2]);
				_X10Debug.print("      Adresse : ");
				PrintHexByte((Fast_Macro >> 8)& 0xFF);
				PrintHexByte(Fast_Macro & 0xFF);
				_X10Debug.println("");
				_X10Debug.println("First bytes in EEPROM Address:");
				for(int i = 0; i < 50 ; i++)
					if((Fast_Macro + i) < EEPROM_CM11A_SIZE)
						PrintHexByte(EEPROM.read(Fast_Macro + i));
				_X10Debug.println();
				delay(10);
			}
			if(PCMode) {									// Sending Received Byte from CM11A to PC (Fast macro preceeded by 5A - C3 handshake) just informations, no answer needed
				_X10PCSerial.write(Buffer_Serial,Buffer_Size);		//
				if(X10Debug) {
					_X10Debug.println("Sending macro EEPROM Address received from CM11A to PC:");
					for(int i = 0; i < Buffer_Size ; i++)
						PrintHexByte((byte)Buffer_Serial[i]);
					_X10Debug.println("");
					delay(10);
				} 
			} else {												// Update Current HC and Current Bitmap
				unsigned int BackupCU = CurrentUnits;				// Backup Current selection before external function _StdR
				byte BackupHC = CurrentHC;		
				if(_FastM != NULL && ReceivedData && !PCMode )		//	go to external _FastM function only if not in PCMode
					_FastM();
				CurrentUnits = BackupCU;							// Restore Current selection after external function _StdR
				CurrentHC = BackupHC;					
			}
			break;       
		case ((byte)0xA5):          								// Received Time Request from CM11A
			if(PCMode) {											// If PCMode, sending Time Request to PC (and update if necessary PC_Ok)
				_X10PCSerial.write((byte)0xA5);							// If in PCMode sending Time request to PC
				if(X10Debug) {
					_X10Debug.println("Sending Time Request to PC:");
					_X10Debug.println("A5");
				}
				while(millis() - previousmillis < TIMEOUTDELAY && !ReceivedPCData ) {		// waiting for data from PC
					if (_X10PCSerial.available()) {
						handlePC();									// Relance la commande pour traiter les données supplémentaires envoyées par le CM11A
						ReceivedData = true;				
					} else delay(SHORT_DELAY);                          // short delay to wait for data from Interface
				}
			} 
			if (!PC_Ok || !PCMode) {									// If RFLink Mode or PC not alive Just send Acknowledge else response gave to CM11A from handlePC
				delay(SHORT_DELAY);                  					// wait 20 ms before sending Acknowledge
				_X10Serial.write((byte)0x9B);   						// Acknowledge / Clear, This must be cleared or it will not except any other x10 commands
				delay(SHORT_DELAY);
				if(X10Debug) {
					_X10Debug.println("Time request from CM11A:");
					_X10Debug.println("      A5");
					_X10Debug.println("9B              Only Acknowledge from ESP");
					delay(10);
				}
			}
			break;
		default:
			byte Byte_Sent = FirstByte;
			
			if(X10Debug) {
				_X10Debug.print("Length of data sent from CM11A: ");
				PrintHexByte((byte)Byte_Sent);
				_X10Debug.println("");
				delay(10);
			}
			if(Byte_Sent < 10) {									// 1 byte mask and maximum 8 bytes sent
				while( millis() - previousmillis < TIMEOUTDELAY && Buffer_Size < Byte_Sent + 1 ) {
					for(int j = _X10Serial.available() ; j > 0 ; j--)
							Buffer_Serial[Buffer_Size++] = _X10Serial.read();
				
					if (Buffer_Size < Byte_Sent + 1 )
						delay(SHORT_DELAY);                            	// short delay to wait for full data from CM11A
				}
				
				if(X10Debug) {
					_X10Debug.print("      ");
					for(int i = 0; i < Buffer_Size ; i++)
						PrintHexByte((byte)Buffer_Serial[i]);
					_X10Debug.println("");
					delay(10);
				}
				if(PCMode) {											// Sending Received Byte from CM11A to PC only if in PCMode (just information, no answer needed)
					_X10PCSerial.write(Buffer_Serial,Buffer_Size);
					if(X10Debug) {
						_X10Debug.println("Sending Data received from CM11A to PC:");
						for(int i = 0; i < Buffer_Size ; i++)
							PrintHexByte((byte)Buffer_Serial[i]);
						_X10Debug.println("");
						delay(10);
					}
				}
				
				if( Buffer_Size == Byte_Sent + 1 ) {							// Update Current Device bitmap 
					for ( int i = 2; i < Buffer_Size ; i++) {
						if ( (Buffer_Serial[1] >> (i-2)) & 0x01) {				// Function Byte
							UpdateCurrentDevice( Buffer_Serial[i] , true );
	
							if(X10Debug) {
								_X10Debug.print("Numéro Byte traité : ");
								_X10Debug.print(i);
								_X10Debug.print("   ");
								PrintHexByte(Buffer_Serial[i]);
								_X10Debug.println();
							}
							
							CurrentFC = Buffer_Serial[i] & 0x0F | 0x10 ;
							CurrentData = 0;									// Current Data for Extended functions or for Dim / Bright Functions
							CurrentCommand = 0;
							if( (CurrentFC & 0x0F) == DIM || (CurrentFC & 0x0F) == BRIGHT ) {		// DIM or BRIGHT Function with Bright level byte (from 0 to 210)
								CurrentData = Buffer_Serial[i+1] * 63 / 210 ;	// Dim / Bright command sent between 0-210 value coded between 0-63
								i++;											// 1 extra byte for Dim / Bright command
							}
							else if ( (CurrentFC & 0x0F) == EXTENDED_CODE ) {	// Extended function with Data and Command byte
								CurrentData = Buffer_Serial[i+1];
								CurrentCommand = Buffer_Serial[i+2];
								i+=2;											// 2 extra bytes for extended command
							}
	
							if(X10Debug) {
								
								
								if( (CurrentFC & 0x0F) == DIM || (CurrentFC & 0x0F) == BRIGHT ) {		// DIM or BRIGHT Function with Bright level byte (from 0 to 210)
									_X10Debug.println("Commande Dim ou Bright");											// 1 extra byte for Dim / Bright command
								}
								else if ( (CurrentFC & 0x0F) == EXTENDED_CODE ) {	// Extended function with Data and Command byte
									_X10Debug.println("Commande Extended");											// 1 extra byte for Dim / Bright command
								}
								_X10Debug.print("CurrentData : ");
								PrintHexByte(CurrentData);
								_X10Debug.println();
								if(i+1<Buffer_Size){
									_X10Debug.print("Prochain octet à décoder : ");
									PrintHexByte(Buffer_Serial[i+1]);
									_X10Debug.println();
								}
//								_X10Debug.println("UpdateCurrentDevice");
								
							}

	

							if(_StdR != NULL && !PCMode )
								_StdR();
						} else	{										// Device byte
							UpdateCurrentDevice( Buffer_Serial[i] , false );
						}
					}
				}
			} else {
				
			}
		} // Switch
	} // if _X10Serial.available
	
	
	if(!PCMode && FastMacroWaiting && (millis() - FastMacroTime > FastMacroDelay ) ) {
		FastMacroWaiting = false ;
		DecodeBlocFastMacro ( FastMacroAddress , true );										// Bloc décodé avec chainage
	}
	
	
	if(CM11A_Ok && ((millis() - _TimePolling >= STATUS_DELAY) || _TimePolling == 0 ) && !_RequestCommit) {
		_TimePolling = millis();
		StatusRequest();
	}	

	if(PCMode) {
		handlePC();
	}
	
	if( _RequestCommit && (millis() - _TimeEEPROM >= COMMIT_DELAY)) {							// commit ESP EEPROM changes
		EEPROM.commit();
		_RequestCommit = false;
	}
		
	
}


void X10CM11A::handlePC() {											// Handle PC only if in PCMode (sending Received Byte from PC to CM11A) and CM11A OK
	byte FirstByte = 0;
	Buffer_Size = 0;
	PCBuffer_Size = 0;
	unsigned long previousmillis = millis();
	boolean ReceivedData = false;
	boolean ReceivedPCData = false;
	boolean AckPC = false;
	boolean AckOk = false;
	byte Checksum = 0;
	char msgbuff[80];
	int i, j;
	
	for(int intcpt=0;intcpt < X10CM11A_BUFFER_SIZE;intcpt++) {
		Buffer_Serial[intcpt]=0;
		PCBuffer[intcpt]=0;
	}		
	if (_X10PCSerial.available() > 0 ) {
		PCBuffer[PCBuffer_Size++]= _X10PCSerial.read();
		FirstByte = PCBuffer[0];
		// _X10Serial.write(FirstByte);
		if(X10Debug) {
			_X10Debug.println("Receiving PC data:");
			delay(10);
		}
		PC_Ok = true;											// receiveing data from PC
		switch (FirstByte) {			
			case ((byte)0x00):
				delay(SHORT_DELAY);
				_X10PCSerial.write(0x55);						// ESP Sending ready to receive to PC
				if(X10Debug) {
					_X10Debug.println("      00        Checksum correct");
					_X10Debug.println("55              ESP Ready to receive data from PC");
				}
				delay(SHORT_DELAY);
				break;
			case ((byte)0x9B):          								// Time Set received from PC
				while( millis() - previousmillis < TIMEOUTDELAY && PCBuffer_Size < X10CM11A_SET_TIME_SIZE ) {
					for( j = _X10PCSerial.available() ; j > 0 ; j--)
						PCBuffer[PCBuffer_Size++]= _X10PCSerial.read();
					if (PCBuffer_Size < X10CM11A_SET_TIME_SIZE )
						delay(SHORT_DELAY);                            	// short delay to wait for response from Interface
				}
				
				if(X10Debug) {
					_X10Debug.println("Set Time Received from PC:");
					_X10Debug.print("      ");
					for( i = 0; i < PCBuffer_Size ; i++)
						PrintHexByte((byte)PCBuffer[i]);
					_X10Debug.println("");
					delay(10);
				}

				
				if(PCBuffer_Size == (X10CM11A_SET_TIME_SIZE-1))				// Bug of ActiveHome Pro who only send 6 bytes and forget Monitoring Housecode + reset bits
					PCBuffer[PCBuffer_Size++] = MonitoredHouseCode;
				
				Checksum=checksum( PCBuffer + 1 , PCBuffer_Size );
				
				
				if(PCBuffer_Size == X10CM11A_SET_TIME_SIZE ) {		// Acknowledge only if Acknowledge received from CM11A

					CurrentTimeS = PCBuffer[1];
					CurrentTimeM = PCBuffer[2];
					CurrentTimeH = PCBuffer[3];
					CurrentYearDay = ((PCBuffer[5] & 0x80)<<8)+ PCBuffer[4];
					DayMask = PCBuffer[5] & 0x7F; 						// (SMTWTFS)
					MonitoredHouseCode = PCBuffer[6] & 0xF0;
							
					setTime(CurrentTimeH*2+CurrentTimeM/60,CurrentTimeM%60,CurrentTimeS, CurrentYearDay, CurrentYear);
					if(CurrentYear!=year()) {
						CurrentYear=year();
					}
					
					_TimeMillis = previousmillis;
					AckOk = SendBuffer ( PCBuffer, PCBuffer_Size, 1 );

					for( i = 0 ; i < X10CM11A_SET_TIME_SIZE - 1; i++)
						EEPROM.write(EEPROM_CM11A_STAT + 2 + i, PCBuffer[i+1] );					
					
					if(X10Debug) {
						_X10Debug.println("Set Time sent to CM11A:");
						for( i = 0; i < PCBuffer_Size ; i++)
							PrintHexByte((byte)PCBuffer[i]);


						sprintf(msgbuff, "N° Jour: %d HMS : %02d:%02d:%02d", CurrentYearDay, CurrentTimeH*2+CurrentTimeM/60, CurrentTimeM%60,CurrentTimeS);
						_X10Debug.println(msgbuff);						
						_X10Debug.println("");
						delay(10);
					}
					AckPC = AcknowledgePC(Checksum);
				}
				break;
			case ((byte)0xFB):          								// EEPROM Upload From PC to CM11A
				while( millis() - previousmillis < X10CM11A_UPLOAD_EEPROM_DELAY && PCBuffer_Size < X10CM11A_UPLOAD_EEPROM_SIZE + 1 ) {
					for(int j = _X10PCSerial.available() ; j > 0 ; j--)
						PCBuffer[PCBuffer_Size++]= _X10PCSerial.read();
					if (PCBuffer_Size < X10CM11A_UPLOAD_EEPROM_SIZE + 1 )
						delay(SHORT_DELAY);                            	// short delay to wait for response from Interface
				}
				
				if(X10Debug) {
					_X10Debug.println("EEPROM Upload Received from PC:");
					_X10Debug.print("      ");
					for(int i = 0; i < PCBuffer_Size ; i++)
						PrintHexByte((byte)PCBuffer[i]);
					_X10Debug.println("");
					delay(10);
				}					
				if(PCBuffer_Size == X10CM11A_UPLOAD_EEPROM_SIZE + 1) {	// if received data complete, send EEPROM to CM11A
					delay(SHORT_DELAY);					
					AckOk = SendEEPROMData(PCBuffer + 1);
					unsigned int EEPROMAddress = (PCBuffer[1] << 8) + PCBuffer[2];
					for(int i=0; i< X10CM11A_UPLOAD_EEPROM_SIZE - 2;i++) {
						if(EEPROMAddress + i < EEPROM_CM11A_SIZE)
							EEPROM.write(EEPROMAddress + i , PCBuffer[i+3] );	
					}
					_RequestCommit = true;								// send timer et request commit for EEPROM
					_TimeEEPROM = millis();
				}
				Checksum = checksum( PCBuffer+1 , X10CM11A_UPLOAD_EEPROM_SIZE );
				if(AckOk)												// Acknowledge only if  Acknowledge received from CM11A
					AcknowledgePC(Checksum);
				break;			
			case ((byte)0xEB):          								// enable Ring
			case ((byte)0xDB):          								// disable Ring
				if (EnableRing(FirstByte == 0xEB)) {
					_X10PCSerial.write(FirstByte);						// Checksum Acknowledge to PC
					AcknowledgePC(FirstByte);
				}
				break;
			case ((byte)0x8B):          								// status request
				if(X10Debug) {
					_X10Debug.println("      8B        Status request received from PC");
				}
				if (StatusRequest()){
					_X10PCSerial.write(_CM11AStatus , X10CM11A_STATUS_SIZE);
					if(X10Debug) {
						for(int i = 0; i < Status_Size ; i++)
							PrintHexByte((byte)_CM11AStatus[i]);
						_X10Debug.println("");
					}						
				}
				break;
			default:													// Standard 04, 06 07 command
				delay(SHORT_DELAY);										// Short delay before reading input buffer to be shure all bytes are sent
				int byterequest = 0;
				if((FirstByte & 0x07) == 0x04 || (FirstByte & 0x07) == 0x06)
					byterequest = 2;
				else if ((FirstByte & 0x07) == 0x07)
					byterequest = 5;
				else
					byterequest = 0;
				while( millis() - previousmillis < TIMEOUTDELAY && PCBuffer_Size < byterequest ) {
					for(int j = _X10PCSerial.available() ; j > 0 ; j--)
						PCBuffer[PCBuffer_Size++]= _X10PCSerial.read();
					if (PCBuffer_Size < byterequest )
						delay(SHORT_DELAY);                            	// short delay to wait for response from Interface
				}
				
				
				if (PCBuffer_Size == byterequest ) {
					if ((PCBuffer[0] & 0x07) == 0x04) {						// device sent
						if(X10Debug) {
							_X10Debug.println("Device Received from PC:");
							_X10Debug.print("      ");
							PrintHexByte((byte)PCBuffer[0]);
							PrintHexByte((byte)PCBuffer[1]);
							_X10Debug.println("");
							delay(10);
						}							
						SendDevice(PCBuffer[1]);
					} else if ((PCBuffer[0] & 0x07) == 0x06 ) {
						if(X10Debug) {
							_X10Debug.println("Function Received from PC:");
							_X10Debug.print("      ");
							PrintHexByte((byte)PCBuffer[0]);
							PrintHexByte((byte)PCBuffer[1]);
							_X10Debug.println("");
							delay(10);
						}
						SendFunction(PCBuffer[1], PCBuffer[0] >> 3);							
					} else if ((PCBuffer[0] & 0x07) == 0x07) {
						if(X10Debug) {
							_X10Debug.println("Extended Function Received from PC:");
							_X10Debug.print("      ");
							for(int i = 0; i < PCBuffer_Size ; i++)
								PrintHexByte((byte)PCBuffer[i]);
							_X10Debug.println("");
							delay(10);
						}
						SendExtFunction( (PCBuffer[1] & 0xF0) || (PCBuffer[2] & 0x0F) ,  PCBuffer[3],  PCBuffer[4]);							
					}
					Checksum = checksum( PCBuffer, PCBuffer_Size );
					AcknowledgePC(Checksum);
				}
		}
	}
	

	
	
	if(!PC_Ok && (millis() - _TimePollingPC >= POLLING_DELAY) && !_RequestCommit) {
		_TimePollingPC = millis();
		_X10PCSerial.write(0xA5);
		if(X10Debug) {
			_X10Debug.println("PC Request for Time set:");
			_X10Debug.println("      A5");
			delay(10);
		}
	}

}

void X10CM11A::ReceiveStd(void (* fn)()) {
	_StdR = fn;
}

void X10CM11A::ReceiveFastM(void (* fn)()) {
	_FastM = fn;
}


/*********************************************************************************************************************************
* Permet d'accepter la réponse du CM11A (true si réponse avec checksum ok, false si pas de réponse ou cheksum ko)
*********************************************************************************************************************************/
boolean X10CM11A::Acknowledge(byte CheckSum) {
	boolean Ack1 = false;
	boolean ReceivedData = false;
	byte ReceivedByte;

	unsigned long previousmillis = millis();
	while( millis() - previousmillis < TIMEOUTDELAY && !ReceivedData ) {             //
		if (_X10Serial.available()) {
			ReceivedData = true;
			ReceivedByte = _X10Serial.read();
			if(X10Debug) {
				_X10Debug.print("      ");
				PrintHexByte(ReceivedByte);
				_X10Debug.println(ReceivedByte == CheckSum ? "        Checksum Ok" : "        Bad Checksum" );
			}
			if (ReceivedByte == CheckSum ) {                	// check if correct checksum sent from interface		
				Ack1 = true;
			} 
		} else delay(SHORT_DELAY);                            	// short delay to wait for checksum from Interface
	}

	if(Ack1)
		Ack1 = ReadyToReceive();
	
	CM11A_Ok = Ack1;
	return CM11A_Ok;
}

boolean X10CM11A::ReadyToReceive() {
	boolean Ack1 = false;
	boolean ReceivedData = false;
	byte ReceivedByte;

	_X10Serial.write((byte)0x00);        			// Acknowledge checksum
	if(X10Debug) {
		_X10Debug.println("00              Acknowledge from ESP");
	}	
	delay(SHORT_DELAY);                            				// short delay to wait for Interface response
	unsigned long previousmillis = millis();
	while( millis() - previousmillis < TIMEOUTDELAY && !ReceivedData ) {             	//
		if (_X10Serial.available()) {
			ReceivedData = true;
			ReceivedByte = _X10Serial.read();
			if(X10Debug) {
				_X10Debug.print("      ");
				PrintHexByte(ReceivedByte);
				_X10Debug.println(ReceivedByte == (byte)0x55 ? "              Ready to receive data from ESP" : "              Incorrect answer");
			}
			if (ReceivedByte == (byte)0x55 )                	// check if CM11A is ready to receive data
				Ack1 = true;
		} else delay(SHORT_DELAY);                            	// short delay to wait for Interface response
	}
	delay(SHORT_DELAY);                                       	// short delay before sending response (minimum delay between 2 commands)
	return Ack1;
}


/*********************************************************************************************************************************
* Permet d'accepter la réponse du PC (true si réponse avec checksum ok, false si pas de réponse ou cheksum ko)
*********************************************************************************************************************************/
boolean X10CM11A::AcknowledgePC(byte CheckSum) {
	boolean Ack1 = false;
	boolean ReceivedData = false;
	X10serialPCFlush();
	
	unsigned long previousmillis = millis();
	_X10PCSerial.write(CheckSum);								// Sending checksum to PC
	if(X10Debug) {
		PrintHexByte(CheckSum);
		_X10Debug.println("              Sending Checksum to PC");
	}		
	while( millis() - previousmillis < TIMEOUTDELAY && !ReceivedData ) {             // Waiting for Checksum Acknowledge 0x00
		if (_X10PCSerial.available()) {
			ReceivedData = true;
			if (_X10PCSerial.read() == 0x00 ) {
				delay(SHORT_DELAY);                            	// short delay
				_X10PCSerial.write(0x55);						// ESP Sending ready to receive to PC
				if(X10Debug) {
					_X10Debug.println("      00        Checksum correct");
					_X10Debug.println("55              ESP Ready to receive data from PC");
				}			
				Ack1 = true;
			} else {
				Ack1 = false;
				if(X10Debug) {
					_X10Debug.println("                No Checksum Acknowledge from PC");
				}
			} 
		} else delay(SHORT_DELAY);                            	// short delay to wait for checksum from Interface
	}
	PC_Ok = Ack1;
	return PC_Ok;
}

/*********************************************************************************************************************************
* Réveille le PC (true si réponse ok, false si pas de réponse)
*********************************************************************************************************************************/
boolean X10CM11A::AwakePC() {
	boolean ReceivedPCData = false;
	boolean AckPC = false;
	unsigned long previousmillis = millis();	
	_X10PCSerial.write((byte)0x5A);	
	while( millis() - previousmillis < TIMEOUTDELAY && !ReceivedPCData ) {		// waiting for data from PC
		if (_X10PCSerial.available()) {
			AckPC = _X10PCSerial.read() == 0xC3 ;								// Acknowledge Ok, PC Alive
			ReceivedPCData = true;
		} else 
			delay(SHORT_DELAY);
	}
	if(X10Debug) {
		_X10Debug.println("Awake PC:");
		_X10Debug.println("5A");
		_X10Debug.println( AckPC ? "      C3        Acknowledge from PC" : "                No Acknowledge received from PC");
		delay(10);
	}	
	PC_Ok = AckPC;
	return PC_Ok;
}

/*********************************************************************************************************************************
* Vide le buffer d'entrée provenant du CM11A
*********************************************************************************************************************************/
void X10CM11A::X10serialFlush(){
	if(X10Debug) {
		_X10Debug.println("flush input from CM11A :");
		_X10Debug.print("      ");
		while(_X10Serial.available()) {
			PrintHexByte(_X10Serial.read());
			delay(1);
		}
	}
	else {
		while(_X10Serial.available()) {
			char t = _X10Serial.read();
			delay(1);
		}
	}
}

void X10CM11A::X10serialPCFlush(){
	while(_X10PCSerial.available()) {
		char t = _X10PCSerial.read();
		delay(1);
	}
}
/*---------------------------------------------------------------+
 |  Return the 8-bit checksum of the bytes in argument buffer.   |
 +---------------------------------------------------------------*/     
byte X10CM11A::checksum( unsigned char *buffer, int length ) 
{
   int j;
   byte sum = 0; 

   for ( j = 0; j < length; j++ )
      sum += buffer[j];

   return (sum & (byte)0xff);
}


/*---------------------------------------------------------------+
 |  Return the 8-bit checksum of the bytes in argument buffer.   |
 +---------------------------------------------------------------*/     
void X10CM11A::UpdateCurrentDevice( byte code, boolean fct ) 
{
//	byte CurrentHC;				// Current Addressed House Code (High Nibble, Low Nibble = 0 Last message = Address or 1 = Last message = function)
//	unsigned int CurrentUnits;	// Bitmap
	char msgbuff[80];
	if(X10Debug) {
		sprintf(msgbuff, "Previous Device: %c %04X   Bit Function : %d", HouseC[CurrentHC>>4], CurrentUnits, CurrentHC & 0x0F);
		_X10Debug.println(msgbuff);
		PrintHexByte(code);
		if(fct) {
			sprintf(msgbuff, "   Function reçue: %c %s  (%02x)", HouseC[code>>4], FunctionC[code & 0x0F][3], code);
		} else {
			sprintf(msgbuff, "   Device reçu: %c %d  (%02x)", HouseC[code>>4], UnitC[code & 0x0F], code);
		}
		_X10Debug.println(msgbuff);
	}	
	if ( (CurrentHC & (byte)0xF0) == ( code & (byte)0xF0) ) {	// Même House code
		if ( fct) {									// If Function sent => Function Nibble update
			CurrentHC |= 1;
			switch ( code & (byte)0x0F) {
			case ALL_UNITS_OFF:
			case ALL_LIGHTS_ON:
			case ALL_LIGHTS_OFF:
				CurrentUnits = 0;
			}
		} else if ( (CurrentHC & (byte)0x0F) == 0 )		// If Address && Last code == Address => Bitmap update
			CurrentUnits |= 1 << (code & 0x0F);
		else {										// If Address && Last code == Function => Bitmap reinit + Function Nibble update
			CurrentUnits = 1 << (code & 0x0F);
			CurrentHC = code & (byte)0xF0 ;			
		}
	} else {					// Changement de code House, Bitmap remis à 0 puis actualisé si c'est une adresse envoyée
		CurrentUnits = 0;
		CurrentHC = code & (byte)0xF0 ;
		if(fct)
			CurrentHC |= 1 ;
		else
			CurrentUnits = 1 << (code & 0x0F);
	}
	if(X10Debug) {
		sprintf(msgbuff, "Current Device: %c %04X   Bit Function : %d", HouseC[(CurrentHC & 0xF0)>>4], CurrentUnits, CurrentHC & 0x0F);
		_X10Debug.println(msgbuff);
	}
}


/*********************************************************************************************/
void X10CM11A::upstr(char *s) {
	char  *p;

	for (p = s; *p != '\0'; p++) 
	*p = (char) toupper(*p);
}


/*********************************************************************************************
* Pour debug : Affiche sur le port série la valeur d'un octet
* ex: si data = 0x3A -> _X10Debug.print("3A")
*********************************************************************************************/
void X10CM11A::PrintHexByte(uint8_t data) { // prints 8-bit value in hex (single byte) 
	char tmp[3];
	byte first ;
	first = (data >> 4) | 48;                   // or with 0x30
	if (first > 57) 
		tmp[0] = first + (byte)7;   // 39;  // if > 0x39 add 0x07 
	else 
		tmp[0] = first ;

	first = (data & 0x0F) | 48;
	if (first > 57) 
		tmp[1] = first + (byte)7;  // 39; 
	else 
		tmp[1] = first;
	tmp[2] = 0;
	_X10Debug.print(tmp);
}
/*********************************************************************************************/


/*---------------------------------------------------------------+
 |  Send the contents of the buffer to the interface.            |
 |  Return true if successful, false otherwise.                  |
 |  Argument 'chkoff' - which byte starts the checksum.          |
 +---------------------------------------------------------------*/  
boolean X10CM11A::SendBuffer ( unsigned char *buffer, int length, int chkoff )
{
	byte CheckSum;
	boolean AckOk = false;
	
	CheckSum = checksum(buffer + chkoff, length - chkoff);

	_X10Serial.write( buffer, length);
	AckOk = Acknowledge(CheckSum);

	if (!AckOk) {
		_X10Serial.write((byte)0x00);                  // To enable CM11A to receive data even if Acknowledge problem
		return false;                                 // No Acknowledge or bad checksum received several times
	} else
		return true;                                  // Correct Checksum received from CM11A
}

 
/*-----------------------------------------------------------------+
 |  Send an encoded housecode|bitmap to the interface.             |
 |  Try up to 3 times.  Return true if successful, otherwise false |
 |  Argument housecode is a X10 encoded char A-P high nibble       |
 |  Argument bitmap is encoded per X10                             |
 +-----------------------------------------------------------------*/
boolean X10CM11A::SendBitmap ( byte hcode, unsigned int bitmap )
{  
	unsigned int  mask;
	unsigned char buffer[4];
	char msgbuff[80];
	byte  j, k;
	boolean result = false;
	  
	if ( bitmap == 0 )
	  return true;

	mask = 1;
	for ( j = 0; j < 16; j++ ) {
		if ( bitmap & mask ) {
			buffer[0] = 0x04;
			buffer[1] = hcode | j ;

			/* Kluge "fix" for checksum 5A problem. */
			/* CM11A seems to disregard a bit in the Dim field. */

			if ( checksum(buffer, 2) == 0x5A )
				buffer[0] = 0x0C;

			if(X10Debug) {
				sprintf(msgbuff, "Sending address bytes: %02x %02x\n", buffer[0], buffer[1]);
				_X10Debug.println(msgbuff);
			}

			for ( k = 0; k < 3 ; k++ ) {
				if ( (result = SendBuffer(buffer, 2, 0)) )
					break;
				delay(SHORT_DELAY);
			}

			/* Delay so many powerline cycles between commands */
			delay(SHORT_DELAY);

			if ( !result ) {
				return false;
			}
		}   
		mask = mask << 1;
	}

	return true;
}




void X10CM11A::DecodeFastMacro ( unsigned int Fast_Macro_Addr )
{
	byte fmd = EEPROM.read( Fast_Macro_Addr );
	
	if(fmd !=0 ) { 												// delayed Fast Macro, initialise timer, 
		FastMacroAddress = Fast_Macro_Addr;
		FastMacroTime = millis();
		FastMacroDelay = fmd * 60000 ;							// timer in minutes from 0 to 240
		FastMacroWaiting = true;
	} else {													// immediate Fast Macro Decode it
		DecodeBlocFastMacro( Fast_Macro_Addr , true );			// bloc décodé avec chainage
	}
}


void X10CM11A::DecodeBlocFastMacro ( unsigned int Fast_Macro_Addr , boolean chained )
{
	unsigned char func = 0xff, nelem;
	unsigned int sp;
	unsigned int  bitmap;

	
	/* Ignore delay */
	sp = Fast_Macro_Addr + 1 ;
	nelem = EEPROM.read(sp++);

	/* Sent each element */
	for ( int k = 0; k < (int)nelem; k++ ) {
		if ( sp >= 5 && sp < EEPROM_CM11A_SIZE ) {
			CurrentHC = EEPROM.read(sp) & 0xF0 ;
			func =  EEPROM.read(sp) & 0x0F;
			CurrentUnits = (EEPROM.read(sp+1) << 8) | EEPROM.read(sp+2);
			CurrentCommand = 0;
			CurrentData = 0;
			/* Send the function */
			if ( func == DIM || func == BRIGHT ) {
				/* Dim or Bright function */
				if ( EEPROM.read(sp+3) & 0x80u ) {				// Brighten before dimming	
					CurrentFC = EXTENDED_CODE ;					// Convert to absolute extended code 0-22 steps to 0-63
					CurrentCommand = 0X31;
					CurrentData = func == DIM ? (22-(EEPROM.read(sp+3) & 0x1F )) * 64 / 22 : 63; // if DIM, Full Bright (22) - Dim value
					CurrentData = CurrentData = 64 ? 63 : CurrentData ;
					if(_StdR != NULL && !PCMode )
						_StdR();
				} else {										// Convert 0-22 steps to 0-16 steps and send the number of DIM Commands
					CurrentFC = func;
					byte Steps = (EEPROM.read(sp+3) & 0x1F ) * 16 / 22 ;
					if(_StdR != NULL && !PCMode )
						for(int i = 0; i<Steps ; i++)
							_StdR();
				}
				sp += 4;
			}
			else if ( func == EXTENDED_CODE ) {
				/* Extended function */
				CurrentCommand = EEPROM.read(sp+4);
				// CurrentUnits = EEPROM.read(sp+3);
				CurrentData = EEPROM.read(sp+5);
				if(_StdR != NULL && !PCMode )
					_StdR();			
				sp += 6;
			}
			else {
			 /* Basic function */
				CurrentFC = func;
				CurrentData = 0;								// Current Data for Extended functions or for Dim / Bright Functions
				CurrentCommand = 0;
				if(_StdR != NULL && !PCMode )
					_StdR();
				sp += 3;
			}
		}
	}

	if(EEPROM.read(sp) != 0 && chained )								// Chained Fast macro
		DecodeFastMacro ( EEPROM.read(sp) );
	
}

void X10CM11A::SetMode(boolean _PCMode, boolean _DefaultMode)
{
	PCMode=_PCMode;
	if(_DefaultMode) {
		if(X10Debug) {
			_X10Debug.print("Previous defaut mode : ");
			PrintHexByte(EEPROM.read(EEPROM_PCMODE));
			_X10Debug.println();
			_X10Debug.print("True / False : ");
			PrintHexByte((byte)true);
			PrintHexByte((byte)false);
			_X10Debug.println();
		}			
		EEPROM.write(EEPROM_PCMODE , (byte)PCMode) ;
		EEPROM.commit();
		_RequestCommit = false;
		if(X10Debug) {
			_X10Debug.print("New defaut mode : ");
			PrintHexByte(EEPROM.read(EEPROM_PCMODE));
			_X10Debug.println();
		}			
	}
	if(PCMode)
		beginPC();
}


boolean X10CM11A::SetYear(unsigned int  _year)
{
	boolean setOk = false;
	if(X10Debug) {
		_X10Debug.print("Previous Year : ");
		_X10Debug.println(CurrentYear);
	}	
	if(_year > 2018 && _year < 2070) {
		CurrentYear = _year;
		EEPROM.put(EEPROM_YEAR , _year) ;
		EEPROM.commit();
		_RequestCommit = false;
		if(X10Debug) {
			_X10Debug.print("New Year : ");
			_X10Debug.println(CurrentYear);
		}			
		setOk = true;
	}
	return setOk;
}

void X10CM11A::UpdateTime()
{
/*
	byte CurrentTimeS;
	byte CurrentTimeM;
	byte CurrentTimeH;
	unsigned int CurrentYearDay;
	unsigned int CurrentYear;
	byte DayMask; 						// (SMTWTFS)	
*/
	boolean leapyear = ((CurrentYear & 0x0003) == 0);
	
	if(++CurrentTimeS ==60) {			// on ajoute 1 seconde et on vérifie si on change de minute
		CurrentTimeS=0;
		if(++CurrentTimeM==120) {		// on ajoute 1 minute et on vérifie si on change de bloc de 2 heures
			CurrentTimeM=0;
			if(++CurrentTimeH==12) {	// on ajoute 1 paire d'heure et on vérifie si on change de jour
				CurrentTimeH==0;
				DayMask = DayMask>>1;	// on change de jour
				if(DayMask==0)
					DayMask=0x40;
				if(++CurrentYearDay==(366 + leapyear)) {	// on ajoute un jour et on vérifie si on change d'année en tenant compte des années bissextiles
					CurrentYearDay=1;
					CurrentYear++;
					_TimeEEPROM = millis();
					_RequestCommit = true;

				}
			}
		}
	}

}