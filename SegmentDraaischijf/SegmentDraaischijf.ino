/*
 Name:		SegmentDraaischijf.ino
 Created:	11/27/2023 10:50:47 AM
 Author:	Rob Antonisse


 Arduino Uno sketch voor de aansturing van een segment draaischijf met twee sporen, zoals op de 'Baan'. 


*/

//libraries
#include <EEPROM.h> //EEprom 
#include <NmraDcc.h>  //DCC decoder

//constructors
NmraDcc  Dcc;


//variabelen
unsigned long slowtimer = 0;
byte readlast = 63; //B0011 1111



void setup() {
	Serial.begin(9600);
	Dcc.pin(0, 2, 1); //interrupt number 0; pin 2; pullup to pin2
	Dcc.init(MAN_ID_DIY, 10, 0b11000000, 0); //bit7 true maakt accessoire decoder, bit6 false geeft decoder 

	//Pinnen
	DDRC = 0; //port C A0~A5 as inputs
	PORTC = 63; //pullup weerstanden op pin A0~A5, tbv. switches en sensoren


}


void loop() 
{
	Dcc.process();
	if (millis() - slowtimer > 20) { //timer op 20ms
		slowtimer = millis();
		SW_exe(); //uitlezen switches en  sensoren.

	}

}

void SW_exe() 
{
	//called from loop om de 20ms.
	byte _read = PINC; //lees register C
	_read &= ~(1 << 192); //clear bit7 en 6
	byte  _change =_read ^ readlast; 
	if (_change > 0) {
		for (byte i = 0; i < 6; i++) {
			if (_change & (1 << i)) { 
				if (_read & (1 << i)) { //knop losgelaten
					SWoff(i);
				}
				else { //knopingedrukt
					SWon(i);
				}
			}
		}
	}
	readlast = _read;
}
void SWon( byte _sw) {
	Serial.print("knop aan: "); Serial.println(_sw);
}
void SWoff( byte _sw) {
	Serial.print("knop uit: "); Serial.println(_sw);
}
