#define FOCUS_TIME                        50                                    // Duration in mSec. that, after receiving serial data from USB only the serial port is checked. 


#define VALUE_PAIR                      44
#define VALUE_ALLOFF                    55
#define VALUE_OFF                       74
#define VALUE_ON                        75
#define VALUE_DIM                       76
#define VALUE_BRIGHT                    77
#define VALUE_UP                        78
#define VALUE_DOWN                      79
#define VALUE_STOP                      80
#define VALUE_CONFIRM                   81
#define VALUE_LIMIT                     82
#define VALUE_ALLON                     141

/*********************************************************************************************\
 * Convert string to command code
\*********************************************************************************************/
int str2cmd(char *command) {
    if(strcasecmp(command,"ON") == 0) return VALUE_ON;
    if(strcasecmp(command,"OFF") == 0) return VALUE_OFF;
    if(strcasecmp(command,"ALLON") == 0) return VALUE_ALLON;
    if(strcasecmp(command,"ALLOFF") == 0) return VALUE_ALLOFF;
    if(strcasecmp(command,"PAIR") == 0) return VALUE_PAIR;
    if(strcasecmp(command,"DIM") == 0) return VALUE_DIM;
    if(strcasecmp(command,"BRIGHT") == 0) return VALUE_BRIGHT;
    if(strcasecmp(command,"UP") == 0) return VALUE_UP;
    if(strcasecmp(command,"DOWN") == 0) return VALUE_DOWN;
    if(strcasecmp(command,"STOP") == 0) return VALUE_STOP;
    if(strcasecmp(command,"CONFIRM") == 0) return VALUE_CONFIRM;
    if(strcasecmp(command,"LIMIT") == 0) return VALUE_LIMIT;
    return false;
}


void handleRFLink() {
    // SERIAL: *************** vérifier si des données sont prêtes sur le port série **********************
  byte SerialInByte=0;                                                          // incoming character value
  int SerialInByteCounter=0;                                                    // number of bytes counter 

  unsigned long FocusTimer=0L;                                                  // Timer to keep focus on the task during communication
  InputBuffer_Serial[0]=0;                                                      // erase serial buffer string     
    byte ValidCommand=0;
  
    if(Serial.available()) {
      FocusTimer=millis();

      while( (millis() - FocusTimer) < FOCUS_TIME) {                                              // standby 
        if(Serial.available()) {
          SerialInByte=Serial.read();                
          
          if(isprint(SerialInByte))
            if(SerialInByteCounter<(INPUT_COMMAND_SIZE-1))
              InputBuffer_Serial[SerialInByteCounter++]=SerialInByte;
              
          if(SerialInByte=='\n' || SerialInByte==' ' || SerialInByte=='\t') {   // new line character, space caracter, tab caracter
            InputBuffer_Serial[SerialInByteCounter]=0;                          // serieel data is complete
            //Serial.print("20;incoming;"); 
            //Serial.println(InputBuffer_Serial); 
            if (strlen(InputBuffer_Serial) > 7){                                // need to see minimal 8 characters on the serial port
               // 10;....;..;ON; 
               if (strncmp (InputBuffer_Serial,"10;",3) == 0) {                 // Command from Master to RFLink
                  // -------------------------------------------------------
                  // Handle Device Management Commands
                  // -------------------------------------------------------
                  if (strcasecmp(InputBuffer_Serial+3,"PING;")==0) {
                     sprintf(InputBuffer_Serial,"20;%02X;PONG;",PKSequenceNumber++);
                     Serial.println(InputBuffer_Serial); 
                  } else
                  if (strcasecmp(InputBuffer_Serial+3,"REBOOT;")==0) {
                     strcpy(InputBuffer_Serial,"reboot");
                     // Reboot();
                     digitalWrite(LED_BUILTIN, LOW);
                     WiFiOff(); ESP.restart();
                  } else
                  if (strncasecmp(InputBuffer_Serial+3,"RFDEBUG=O",9) == 0) {
                     if (InputBuffer_Serial[12] == 'N' || InputBuffer_Serial[12] == 'n' ) {
                        RFDebug=true;                                           // full debug on
                        RFUDebug=false;                                         // undecoded debug off 
                        QRFDebug=false;                                         // undecoded debug off
                        sprintf(InputBuffer_Serial,"20;%02X;RFDEBUG=ON;",PKSequenceNumber++);
                     } else {
                        RFDebug=false;                                          // full debug off
                        sprintf(InputBuffer_Serial,"20;%02X;RFDEBUG=OFF;",PKSequenceNumber++);
                     }
                     Serial.println(InputBuffer_Serial); 
                  } else                 
                  if (strncasecmp(InputBuffer_Serial+3,"RFUDEBUG=O",10) == 0) {
                     if (InputBuffer_Serial[13] == 'N' || InputBuffer_Serial[13] == 'n') {
                        RFUDebug=true;                                          // undecoded debug on 
                        QRFDebug=false;                                         // undecoded debug off
                        RFDebug=false;                                          // full debug off
                        sprintf(InputBuffer_Serial,"20;%02X;RFUDEBUG=ON;",PKSequenceNumber++);
                     } else {
                        RFUDebug=false;                                         // undecoded debug off
                        sprintf(InputBuffer_Serial,"20;%02X;RFUDEBUG=OFF;",PKSequenceNumber++);
                     }
                     Serial.println(InputBuffer_Serial); 
                  } else                 
                  if (strncasecmp(InputBuffer_Serial+3,"QRFDEBUG=O",10) == 0) {
                     if (InputBuffer_Serial[13] == 'N' || InputBuffer_Serial[13] == 'n') {
                        QRFDebug=true;                                          // undecoded debug on 
                        RFUDebug=false;                                         // undecoded debug off 
                        RFDebug=false;                                          // full debug off
                        sprintf(InputBuffer_Serial,"20;%02X;QRFDEBUG=ON;",PKSequenceNumber++);
                     } else {
                        QRFDebug=false;                                         // undecoded debug off
                        sprintf(InputBuffer_Serial,"20;%02X;QRFDEBUG=OFF;",PKSequenceNumber++);
                     }
                     Serial.println(InputBuffer_Serial); 
                  } else                 
                  if (strncasecmp(InputBuffer_Serial+3,"VERSION",7) == 0) {
                      sprintf(InputBuffer_Serial,"20;%02X;VER=1.1;REV=%02x;BUILD=%02x;",PKSequenceNumber++,REVNR, BUILDNR);
                      Serial.println(InputBuffer_Serial); 
                  } else {
                     // -------------------------------------------------------
                     // Handle Generic Commands / Translate protocol data into Nodo text commands 
                     // -------------------------------------------------------
                     // check plugins
                     if (InputBuffer_Serial[SerialInByteCounter-1]==';') InputBuffer_Serial[SerialInByteCounter-1]=0;  // remove last ";" char
                     if(CM11A_TX(InputBuffer_Serial) || ESP_TX(InputBuffer_Serial)) {
                        ValidCommand=1;
                     } else {
                        // Answer that an invalid command was received?
                        ValidCommand=2;
                     }
                  }
               }
            } // if > 7
            if (ValidCommand != 0) {
               if (ValidCommand==1) {
                  sprintf(InputBuffer_Serial,"20;%02X;OK;",PKSequenceNumber++);
                  Serial.println( InputBuffer_Serial ); 
               } else {
                  sprintf(InputBuffer_Serial, "20;%02X;CMD UNKNOWN;", PKSequenceNumber++); // Node and packet number 
                  Serial.println( InputBuffer_Serial );
               }   
            }
            SerialInByteCounter=0;  
            InputBuffer_Serial[0]=0;                                            // serial data has been processed. 
            ValidCommand=0;
            FocusTimer=millis();                                             
          }// if(SerialInByte
       }// if(Serial.available())
    }// while 
   }// if(Serial.available()) 
}
