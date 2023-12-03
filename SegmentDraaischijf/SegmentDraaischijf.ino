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
const byte AantalProgramFases = 2;

//variabelen
byte shiftbyte[2];
byte digit[4];
byte digitcount = 0;

byte programfase = 0;

unsigned long slowtimer = 0;
unsigned int waittime = 0;
int waitnext = 0;


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
bool stand = 0; //false =rechts true is links
byte stepdoel = 0;
byte stepmovefase = 0;
byte memreg;
unsigned int standstep = 0; //huidige stand van de stepper
unsigned int standdoel = 0; //stop doel voor de stepper
unsigned int standrechts = 50; //ingeprogrammeerde stand rechts
unsigned int standlinks = 1000; //ingeprogrammeerde stand links
unsigned int standnahome = 0;
unsigned int displaycount = 0; //counter voor snelheid display 

//temps
int testnummer = 0;
int displaytestcount = 0;
byte displaydigit = 0;
byte displaysegment = 0;


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

	DDRD |= (1 << 7); //pins7,6,5,4 als output pin 7=bezet flag , pin 6 = SRCLK,  pin 5= RCLK,  pin 4=SER	
	DDRD |= (1 << 6); //srclk
	DDRD |= (1 << 5); //rclk
	DDRD |= (1 << 4); //ser

	//initialisaties
	start = true;
	Eeprom_read();
	;

	shiftbyte[0] = B11111001;
	shiftbyte[1] = 0; 
	
	DisplayMotor();

	testnummer = 0;

	Shift();
}
void loop()
{
	Dcc.process();
	Step_exe();
	Display_exe();

	if (millis() - slowtimer > 20) { //timer op 20ms
		slowtimer = millis();
		SW_exe(); //uitlezen switches en  sensoren.
		Wait();
		//testdisplay();
	}
}
//algemeen en debugs
void Shift() {
	//plaatst de shiftbytes in de schuifregisters
	//d4=ser, d5=srclk d6=sclk
	for (byte _byte = 1; _byte < 2; _byte--) {
		for (byte _bit = 7; _bit < 8; _bit--) {

			if (shiftbyte[_byte] & (1 << _bit)) { //bit is true
				PORTD |= (1 << 4); //set SER 
			}
			else {
				PORTD &= ~(1 << 4); //clear SER
			}
			PIND |= (1 << 6); PIND |= (1 << 6); //maak een puls op SRCLK pin 5, alles doorschuiven
		}
	}
	PIND |= (1 << 5); PIND |= (1 << 5); //maak een puls op RCLK 'klok' de beide bytes in de schuif registers.
}
void Wait() { //called all 20ms from loop
	//!! pas op dat de wachtlus niet wordt aangeroepen als het al loopt voor een ander proces. 

	if (waittime > 0) {
		waittime--; //verminder de wachttijd
		if (waittime == 0) {
			switch (waitnext) {
			case 1: //wachttijd na draaiopdracht naar het draaien
				Step_move();
				break;
			case 2: //?
				break;
			}
		}
	}
}
void NoodStop() {

}
void Bezet(bool _bezet) {
	if (_bezet) {
		PORTD |= (1 << 7); //leds, bezet flag output
	}
	else {
		PORTD &= ~(1 << 7);
	}
}

//display
void Display_exe() {
	displaycount ++;
	if (displaycount >500) { //clockcycles
		displaycount = 0; 
		digitcount++;
		if (digitcount > 3)digitcount = 0;
		shiftbyte[1] &= ~(15 << 0); //clear bits 0~3
		shiftbyte[1] |= (1 << digitcount);
		//Serial.println(shiftbyte[1]);
		shiftbyte[0] = digit[digitcount];
		Shift();
		}	
}
void DisplayMotor() {
	digit[0] = B00111111;//Cijfer(1);//B11111001; //alle segmenten aan
	digit[1] = B00111111;//Cijfer(2); //B11111000;
	digit[2] = B00111111;//Cijfer(3); //B11110000;
	digit[3] = B00111111;//Cijfer(4); //B11100000;
}
void DisplayNummer(int _nummer) {
	//toont een 4 cyfer nummer op het display

	byte  _cijfer[4];
	for (byte i = 0; i < 4; i++) {
		_cijfer[i] = 0;
	}
	while (_nummer >= 1000) {
		_cijfer[0]++;
		_nummer =_nummer- 1000;
	}
	while (_nummer >= 100) {
		_cijfer[1]++;
		_nummer -= 100;
	}
	while (_nummer >= 10) {
		_cijfer[2]++;
		_nummer -= 10;
	}
	_cijfer[3] = _nummer;

//voorloop nullen weg halen
	if (_cijfer[0] == 0) {
		_cijfer[0] = 10;
		if (_cijfer[1] == 0) {
			_cijfer[1] = 10;
			if (_cijfer[2] == 0)_cijfer[2] = 10;
		}
	}

	for (byte i = 0; i < 4; i++) {
		digit[i] = Cijfer(_cijfer[i]);
	}
}
void DisplayText() {
	//toont texten aan de hand van de programmeer stand en laatste positie van de draaielementen
	switch (programfase) {
	case 0:
		digit[0] = Cijfer(10);
		digit[1] = Cijfer(10);
		digit[2] = Letter('p');
		if (stand) {
			digit[3] = Cijfer(1);
		}
		else {
			digit[3] = Cijfer(2);
		}
		break;

	case 1: //posities standen van de steppenmotor
		digit[0] = Letter('p');
		digit[1] = Cijfer(0);
		digit[2] = Cijfer(5);
		if (stand) {
			digit[3] = Cijfer(1);
		}
		else {
			digit[3] = Cijfer(2);
		}
		break;
	}
}
byte Cijfer(byte _cijfer) {
	byte _result;
//PGFE DCBA
	switch (_cijfer) {
	case 0:
		_result = B11000000;
		break;
	case 1:		
		_result = B11111001;
		break;
	case 2:
		_result = B10100100;
		break;
	case 3:
		_result = B10110000;
		break;
	case 4:
		_result = B10011001;
		break;
	case 5:
		_result = B10010010;
		break;
	case 6:
		_result = B10000010;
		break;
	case 7:
		_result = B1111000;
		break;
	case 8:
		_result = B10000000;
		break;
	case 9:
		_result = B10010000;
		break;
	case 10: //niet zichtbare nul
		_result = 255;
		break;
	}
	return _result;
}
byte Letter(char _letter) {
	byte _result = 0;
	switch (_letter) {
	case 'p':
		_result = B10001100;
		break;
	}
	return _result;
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
		//steprichting = !steprichting;
		//testnummer++;
		//DisplayNummer(testnummer);
		break;
	case 2:
		//Step_stand(true);
		break;
	case 3:
		programfase++;
		if (programfase == AantalProgramFases)programfase = 0;
		DisplayText();
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

	//hier moet een noodstop komen 
	if (standstep > standlinks + standrechts + 100) {
		NoodStop();
	}



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
	DisplayMotor();
	stand = _stand;  //misschien is de extra variablele niet nodig..?
	if (stand) {
		standdoel = standrechts; //terugdraaien?
	}
	else {
		standdoel = standlinks;
	}
	stepmovefase = 10;
	//hier is een timer nodig
	waitnext = 1; //naar Step_move()
	waittime = 100;
	//Step_move();	
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
		DisplayText();
		break;

	}
}