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
const int LimitSpeed = 900;

//variabelen
unsigned long slowtimer = 0;
byte readlast = 15; //B0000 1111   0=hoogactief(sensor)  1=laagactief(drukknop)
int stepspeed = 2000; //snelheid stappen in us. 
int minspeed = 15000; //EEprom 20
int maxspeed = 3000; //EEprom 21
byte afremfactor = 50; //EEprom 22
unsigned long steptimer;
byte stepfase = 0; //in fullstep max4 in halfstep max 8
bool steprichting = false;
bool stephomerichting = true;
bool stepdrive = false;
bool  start = true; //eerste start
bool stand=0; //false =rechts true is links
byte stepdoel = 0;
byte stepmovefase = 0;
byte memreg;
unsigned int standstep = 0; //huidige stand van de stepper
unsigned int standdoel = 0; //stop doel voor de stepper
unsigned int standrechts = 50; //ingeprogrammeerde stand rechts
unsigned int standlinks = 1000; //ingeprogrammeerde stand links
unsigned int standnahome = 0;



void Eeprom_read() {
	memreg = EEPROM.read(10);
	stephomerichting = memreg & (1 << 0); // bit 0 = opgeslagen stand stephomerichting true is standaard	

}

void setup() {
	Serial.begin(9600);
	Dcc.pin(0, 2, 1); //interrupt number 0; pin 2; pullup to pin2
	Dcc.init(MAN_ID_DIY, 10, 0b11000000, 0); //bit7 true maakt accessoire decoder, bit6 false geeft decoder 

	//Pinnen
	DDRC = 0; //port C A0~A5 as inputs
	PORTC = 63; //pullup weerstanden op pin A0~A5, tbv. switches en sensoren
	DDRB = 15; //pinnen 8~11 als output
	PORTB &= ~(15 << 0); //zet pinnen 8~11 laag

	//initialisaties
	start = true;
	Eeprom_read();
}


void loop()
{
	Dcc.process();
	Step_exe();

	if (millis() - slowtimer > 20) { //timer op 20ms
		slowtimer = millis();
		SW_exe(); //uitlezen switches en  sensoren.
	}
}
void Bezet(bool _bezet) {

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
	//Serial.print("knop aan: "); Serial.println(_sw);	
	switch (_sw) {
	case 0: //switch 1
		stand = !stand;
		Step_stand(stand);
		break;

	case 1: //switch 2
		steprichting = !steprichting;

		break;
	case 2:
		Step_stand(true);
		break;
	case 3:
		Step_stand(false);
		break;
	case 4: //Hall sensor voor positie
		//	Serial.println("Hall sensor niet actief");
		Step_sensor(false);
		break;
	}
}
void SWoff(byte _sw) {
	//Serial.print("knop uit: "); Serial.println(_sw);
	switch (_sw) {
	case 4: //hall sensor positie
		//Serial.println("Sensor actief"); //sensor is hoogactief, knoppen zijn laagactief
		Step_sensor(true);
		break;
	}
}
//stappenmotor

void Step_exe() {

	if (stepdrive) {

		if (micros() - steptimer > stepspeed + maxspeed) { //900us is minimaal staat in limitspeed, hoogste haalbare snelheid van de 48byj-28

			//nog af te leggen traject bepalen
			unsigned long _aantalsteps = standdoel - standstep;
			unsigned long _aantalafremsteps = (minspeed - stepspeed) / afremfactor;

			if (_aantalsteps > _aantalafremsteps) { //versnellen
				if (stepspeed > afremfactor) stepspeed = stepspeed - afremfactor;  //geeft versnelling
			}
			else { //vertragen
				if (stepspeed < minspeed) stepspeed = stepspeed + afremfactor;  //geeft vertraging
			}
			steptimer = micros();
			Step_steps();
		}
	}
}


void Step_steps() {

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
	standstep++;
	if (stepmovefase == 12) { //opweg naar doel.

		if (standstep == standdoel) {
			stepmovefase = 20;
			Step_move();
		}
	}
}
void Stepoff() {
	PORTB &= ~(15 << 0);
}

void Step_stand(bool _stand) {
	//verplaats de stappenmotor
	Bezet(true); //zet de schijf als bezet.
		stand = _stand;  //misschien is de extra variablele niet nodig..?
		if (stand) {
			standdoel = standrechts; //terugdraaien?
		}
		else {
			standdoel = standlinks;
		}
		stepmovefase = 10;
		Step_move();
	
}

void Step_sensor(bool onoff) {

	if (onoff) { //sensor wordt  actief

		switch (stepmovefase) {
		case 10: //home zoeken
			steprichting = !steprichting; //keren
			stepmovefase = 12;
			break;
		}
	}
	else { //sensor vrij
		switch (stepmovefase) {
		case 10: //home zoeken
			//treed op als de arm al op de sensor stond, richting draaien
			steprichting = !steprichting;
			stepmovefase = 12;

			break;
		case 12: //home positie bereikt, richting is hier ok.
			standstep = 0; //beginstand instellen
			standdoel = standnahome;  //eind doel terug zetten.
			break;
		}
	}
}
void Step_move() {
	switch (stepmovefase) {
	case 0:
		//idl
		break;
	case 10:
		//Zoek home switch
		steprichting = stephomerichting;
		stepspeed = minspeed; //starten in minimale snelheid
		stepdrive = true;
		standnahome = standdoel; //eind doel even opslaan
		standdoel = standstep; //huidige stand de doelstand maken
		standstep = 0; //huidige stand naar 0
		break;
	case 20: //doel bereikt
		stepdrive = false;
		Stepoff();
		Bezet(false); //geef de schijf vrij, locs kunnen gaan rijden 
		break;

	}
}