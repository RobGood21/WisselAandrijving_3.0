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

//constanten
const int MinSpeed = 900;

//variabelen
unsigned long slowtimer = 0;
byte readlast = 63; //B0011 1111
unsigned int stepspeed = 0; //snelheid stappen in us. 
unsigned long steptimer;
byte stepfase = 0; //in fullstep max4 in halfstep max 8
bool steprichting = false;
bool stepdrive = false;

void setup() {
	Serial.begin(9600);
	Dcc.pin(0, 2, 1); //interrupt number 0; pin 2; pullup to pin2
	Dcc.init(MAN_ID_DIY, 10, 0b11000000, 0); //bit7 true maakt accessoire decoder, bit6 false geeft decoder 

	//Pinnen
	DDRC = 0; //port C A0~A5 as inputs
	PORTC = 63; //pullup weerstanden op pin A0~A5, tbv. switches en sensoren
	DDRB = 15; //pinnen 8~11 als output
	PORTB &= ~(15 << 0); //zet pinnen 8~11 laag

}


void loop()
{
	Dcc.process();

	if (millis() - slowtimer > 20) { //timer op 20ms
		slowtimer = millis();
		SW_exe(); //uitlezen switches en  sensoren.
	}

	if (stepdrive) {
		if (micros() - steptimer > stepspeed+MinSpeed) { //900us is minimaal, hoogste haalbare snelheid van de 48byj-28
			steptimer = micros();
			Step_exe();
		}
	}

}

//drukknoppen en sensoren
void SW_exe()
{
	//called from loop om de 20ms.
	byte _read = PINC; //lees register C
	_read &= ~(1 << 192); //clear bit7 en 6
	byte  _change = _read ^ readlast;
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
void SWon(byte _sw) {
	Serial.print("knop aan: "); Serial.println(_sw);

	switch (_sw) {
	case 0:
		stepdrive = !stepdrive;	
		Stepoff();
		break;
	case 1:
		steprichting = !steprichting;
		break;
	case 2:
		break;

	}



}
void SWoff(byte _sw) {
	Serial.print("knop uit: "); Serial.println(_sw);
}
//stappenmotor

void Step_exe() {

	//Serial.print(stepfase);

	//stuurt de stappenmotor aan
	Stepoff(); //zet alle spoelen uit

	switch (stepfase) {
	case 0:
		PINB |= (1 << 0);
		break;
	case 1:
		PINB |= (3 << 0);
		break;
	case 2:
		PINB |= (1 << 1);
		break;
	case 3:
		PINB |= (3 << 1);
		break;
	case 4:
		PINB |= (1 << 2);
		break;
	case 5:
		PINB |= (3 << 2);
		break;
	case 6:
		PINB |= (1 << 3);
		break;
	case 7:
		PINB |= (9 << 0);
		break;
	}

	if (steprichting) {
		stepfase--;
		if (stepfase > 7)stepfase = 7;
	}
	else {
		stepfase++;
		if (stepfase > 7) stepfase = 0;
	}
}
void Stepoff() {
	PORTB &= ~(15 << 0);
}