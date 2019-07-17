//#######################################################################################################
//##                    This Plugin is only for use with the RFLink software package                   ##
//##                                        Plugin-06: X10 RF                                          ##
//#######################################################################################################

byte incomingByte;     // from the X10 controller or the keyboard
byte LastHeader;
byte SelectedHouseCode;
int SelectedBitMap;

boolean CM11A_TX(char *string) {
  boolean success=false;
//  CM11A.X10Debug = RFDebug;
  
  //10;X10;000041;1;OFF;
  //0123456789012345678
  // 1ère étape on remplace tous les ";" par des "\0" pour pouvoir traiter facilement chaque bloc de commande RFLink
  for(byte cpt = 0; cpt < INPUT_COMMAND_SIZE ; cpt++)
    if (InputBuffer_Serial[cpt] == ';')  
      InputBuffer_Serial[cpt] = 0 ;

  // Une fois là, la chaîne contient la commande complète. Testez d'abord si la commande spécifiée correspond à X10
  if (strncasecmp(InputBuffer_Serial+3,"X10",3) == 0) {     // X10 Command eg. 
    unsigned long bitstream=0L;
    byte x=14;                                      // compteur pointant vers le caractère à traiter
    byte command=0;
    byte Home=0;                                    // Home A..P
    byte Address=0;                                 // Blyss subchannel 1..5
    byte c;
    uint32_t newadd=0;
    byte HouseCode;
    byte UnitCode;
    byte FunctionCode;
    byte Luminance = 0xFF;

    InputBuffer_Serial[ 9]=0x30; // remplace "10 X10 000041 1 OFF "  par "10 X10 000x41 1 OFF " et converti "0x41" en Housecode
    InputBuffer_Serial[10]=0x78;
    HouseCode=CM11A.HouseCode(InputBuffer_Serial+9);// Home: A..P
    if(RFDebug || QRFDebug) {
      Serial.println(InputBuffer_Serial+9);
      // PrintHexByte(HouseCode);
      Serial.println("");
    }
    
    if (HouseCode == 0xFF)                          // invalid value
      return false;                                 // invalid value

    
    InputBuffer_Serial[12]=0x30; // remplace "10 X10 000x41 1 OFF "  par "10 X10 000x40x1 OFF " et converti "0x1" en Unitcode
    InputBuffer_Serial[13]=0x78;    UnitCode=CM11A.UnitCode(InputBuffer_Serial + 12);// Unit: 1..16
    if(RFDebug || QRFDebug) {
      Serial.println(InputBuffer_Serial + 12);
      // PrintHexByte(UnitCode);
      Serial.println("");
    }
    
    if (UnitCode == 0xFF)                           // invalid value
      return false;                                 // invalid value

    x=16;
    if (InputBuffer_Serial[x] == 0 )                // si le unit code est > f il est sur 2 caractères
      x++;                                          // x pointe sur la fonction encodée RFLINK
      
    if(RFDebug || QRFDebug) {
      Serial.println(InputBuffer_Serial + x);
      PrintHexByte(UnitCode);
      Serial.println("");
    }

    FunctionCode = CM11A.FunctionCode(InputBuffer_Serial + x);        // InputBuffer_Serial+x pointe sur la commande

    
    if(FunctionCode == 0xFF) {                    // Code d'erreur si la commande correspond à une valeur entière (0-15) correspondant à une valeur Dim / Bright
      Luminance = (str2int(InputBuffer_Serial+x) * 63 / 15) & 0xFF ;      // get DIM value (1-15 and convert to 4-63 scale)
      if(RFDebug || QRFDebug) {
        Serial.println(InputBuffer_Serial + x);
        PrintHexByte(Luminance);
        Serial.println("");
      }            
    } else {
      if(RFDebug || QRFDebug) {
        Serial.println(InputBuffer_Serial + x);
        // PrintHexByte(FunctionCode);
        Serial.println("");
      }            
    }
          
    CM11A.SendDevice(HouseCode | UnitCode);
    
    if(Luminance == 0xFF)                         // Commande standard X10 (ON, OFF, ALLON, ALLOFF)
      CM11A.SendFunction(HouseCode,FunctionCode,0);
    else {                                          // Commande DIMMER avec valeur directe entre 0 (OFF) et 3F (Full Bright)
      CM11A.SendExtFunction(HouseCode | UnitCode  , Luminance, 0x31);
    }
    success=true;
  }
  return success;
}

// fonction appelée en cas de réception standard X10 de la part du CM11A 
void CM11A_StdR() {
//	byte CurrentHC;				// Current Addressed House Code (High Nibble, Low Nibble = 0 Last message = Address or 1 = Last message = function)
//	byte CurrentFC;				// Current Function Code (Low Nibble, High Nibble = 1 if active, 0 if treated)
//	byte CurrentData;			// Current Data for Extended functions (0-FF) or for Dim / Bright Functions (0-3F)
//	byte CurrentCommand;		// Current Command for Extended functions
//	unsigned int CurrentUnits;	// Current active Units (Bitmap)
	if(RFDebug || QRFDebug) {
		Serial.println("CM11A data received");
    sprintf(InputBuffer_Serial,"    HouseCode : %c   Device Bitmap : %04x  Function : %s  Command : %02x  Data : %02x",  CM11A.HouseC[(CM11A.CurrentHC & 0xF0)>>4] , CM11A.CurrentUnits ,  CM11A.FunctionC[CM11A.CurrentFC & 0x0F][3] , CM11A.CurrentCommand , CM11A.CurrentData);
    Serial.println(InputBuffer_Serial);
	}

	switch (CM11A.CurrentFC & 0x0F) {
		case ALL_UNITS_OFF :	// All Units Off
		case ALL_LIGHTS_ON :	// All Lights On
		case ALL_LIGHTS_OFF :  // All Lights Off
			sprintf(InputBuffer_Serial,"20;%02X;X10;ID=%02x;CMD=%s;",PKSequenceNumber++, CM11A.HouseC[(CM11A.CurrentHC & 0xF0)>>4] , CM11A.FunctionC[CM11A.CurrentFC & 0x0F][3] );
			Serial.println(InputBuffer_Serial);				
			break;
		case ON :	// On
		case OFF :	// Off
			for ( int j = 0; j < 16 ; j++ ) {
				if (CM11A.CurrentUnits & (1 << j)) {
					sprintf(InputBuffer_Serial,"20;%02X;X10;ID=%02x;SWITCH=%X;CMD=%s;",PKSequenceNumber++, CM11A.HouseC[(CM11A.CurrentHC & 0xF0)>>4] ,CM11A.UnitC[j] , CM11A.FunctionC[CM11A.CurrentFC & 0x0F][3] );
					Serial.println(InputBuffer_Serial);
				}
			}
			break;
		case DIM :	// Dim
    case BRIGHT : // Bright   
      for ( int j = 0; j < 16 ; j++ ) {
        if  (CM11A.CurrentUnits & (1 << j)) {
          int NbSteps = CM11A.CurrentData >> 2;
          do {
            sprintf(InputBuffer_Serial,"20;%02X;X10;ID=%02x;SWITCH=%X;CMD=%s;",PKSequenceNumber++, CM11A.HouseC[(CM11A.CurrentHC & 0xF0)>>4] ,CM11A.UnitC[j] , CM11A.FunctionC[CM11A.CurrentFC & 0x0F][3] );
            Serial.println(InputBuffer_Serial);
          } while( --NbSteps > 0);
        }
      }
			break;
		case EXTENDED_CODE :	// La seule extended function traitée est la 0x31 (PRESET RECEIVER O/P on this HC DC)
			if (CM11A.CurrentCommand == 0X31) {
				for ( int j = 0; j < 16 ; j++ ) {
					if (CM11A.CurrentUnits & (1 << j)) {
						sprintf(InputBuffer_Serial,"20;%02X;X10;ID=%02x;SWITCH=%X;SET_LEVEL=%d;",PKSequenceNumber++, CM11A.HouseC[(CM11A.CurrentHC & 0xF0)>>4] ,CM11A.UnitC[j] , (CM11A.CurrentData)>>2 );
						Serial.println(InputBuffer_Serial);
					}
				}				
			}
		default :
			break;
	}

}
/*
// fonction appelée en cas de réception extended X10 de la part du CM11A
void CM11A_ExtR() {

}
*/
// fonction appelée en cas de réception Fast Macro X10 de la part du CM11A
void CM11A_FastM() {
  // CM11A.Fast_Macro contient l'adresse de la macro dans la mémoire de l'EEPROM
  if(RFDebug ||QRFDebug) {
      Serial.println("EEPROM Macro address received");
   }
  CM11A.DecodeFastMacro(CM11A.Fast_Macro);
   
}
