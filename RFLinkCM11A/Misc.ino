/*********************************************************************************************\
 * Convert HEX or DEC tring to unsigned long HEX, DEC
\*********************************************************************************************/

int strncasecmp(const char *s1, const char *s2, size_t n)
{
    if (n == 0) return 0;

    while (n-- != 0 && tolower(*s1) == tolower(*s2)) {
        if (n == 0 || *s1 == '\0' || *s2 == '\0')
            break;
        s1++;
        s2++;
    }

    return tolower(*(const unsigned char *)s1)
        - tolower(*(const unsigned char *)s2);
}


unsigned long str2int(char *string) {
  return(strtoul(string,NULL,0));  
}

/********************************************************************************************\
 * Convert unsigned long to float long through memory
\*********************************************************************************************/
float ul2float(unsigned long ul) {
    float f;
    memcpy(&f, &ul,4);
    return f;
}
/*********************************************************************************************/
void PrintHex8(uint8_t *data, uint8_t length) { // prints 8-bit data in hex (lowercase)
  char tmp[length*2+1];
  byte first ;
  int j=0;
  for (uint8_t i=0; i<length; i++) {
    first = (data[i] >> 4) | 48;
    if (first > 57) tmp[j] = first + (byte)39;
    else tmp[j] = first ;
    j++;

    first = (data[i] & 0x0F) | 48;
    if (first > 57) tmp[j] = first + (byte)39;  
    else tmp[j] = first;
    j++;
  }
  tmp[length*2] = 0;
  Serial.print(tmp);
}
//*********************************************************************************************
// todo: make uppercase?  3a = 3 or 48 (0x30) = 0x33   >57 (0x39)   a>3a >39 >   +27 
void PrintHexByte(uint8_t data) { // prints 8-bit value in hex (single byte) 
  char tmp[3];
  byte first ;
  first = (data >> 4) | 48;                   // or with 0x30
  if (first > 57) tmp[0] = first + (byte)7;   // 39;  // if > 0x39 add 0x27 
  else tmp[0] = first ;

  first = (data & 0x0F) | 48;
  if (first > 57) tmp[1] = first + (byte)7;  // 39; 
  else tmp[1] = first;
  tmp[2] = 0;
  Serial.print(tmp);
}
//*********************************************************************************************/
// Reverse all bits in a byte
byte reverseBits(byte data) {
    byte b = data;
    for (byte i = 0; i < 8; ++i) {
        data = (data << 1) | (b & 1);
        b >>= 1;
    }
    return data;
}
/*********************************************************************************************/

void WiFiOn() {

    wifi_fpm_do_wakeup();
    wifi_fpm_close();

    //Serial.println("Reconnecting");
    wifi_set_opmode(STATION_MODE);
    wifi_station_connect();
}


void WiFiOff() {

    //Serial.println("diconnecting client and wifi");
    //client.disconnect();
    wifi_station_disconnect();
    wifi_set_opmode(NULL_MODE);
    wifi_set_sleep_type(MODEM_SLEEP_T);
    wifi_fpm_open();
    wifi_fpm_do_sleep(FPM_SLEEP_MAX_TIME);

}
