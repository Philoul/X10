/*
Name:		X10CM11A.h
Created:	18/04/2018 13:49:40
Author:		JP LEGOUPIL
for ESP8266
*/

#ifndef _X10_h
#define _X10_h

#include <X10CM11A.h>

class X10
{
public:
	X10(byte _devcode);
	X10();
	~X10();

	
	void Function(byte _FunctionCode);				
	byte Status();

	byte devcode;

private:

};



#endif
