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
#define maxdcc 511  //maximaal aantal dcc decoderadressen
#define dots digit[1]&=~(1<<7); //de punten in het display, aanzetten

const int LimitSpeed = 900;
const byte AantalProgramFases = 7;

//variabelen
int decoderadres = 1; //dccadres = (decoder adres-1) * 4 plus het channel(1~4) binair 0~3 dus deze ook plus 1 
int dccadres = 0; //komt uit EEPROM
bool dccprg = true; //true=text DCC false=Waarde dcc adres

byte shiftbyte[2];
byte digit[4];
byte digitcount = 0;

byte programfase = 0;

unsigned long slowtimer = 0;
unsigned int waittime = 0;
int waitnext = 0;

byte scrollcount[4]; //teller voor het ingedrukt houden van een knop
byte scrollmask = 0; //bepaald welke knoppen op welk moment moge scrollen

byte readlast = 15; //B0000 1111   0=hoogactief(sensor)  1=laagactief(drukknop), laatst gelezen stand van de knoppen

int stepspeed = 2000; //snelheid stappen in us. 
int minspeed = 15000; //EEprom 20
int maxspeed = LimitSpeed; //EEprom 21
byte accelspeed = 0; //aantal stappen tijdens afremmen accellereren = 10xafremfactor
byte speedmaxfactor = 0; //eeprom 11
byte speedminfactor = 0; //eeprom 12
byte afremfactor = 5; //EEprom 13
unsigned long steptimer;
byte stepfase = 0; //in fullstep max4 in halfstep max 8
bool steprichting = false;
bool stephomerichting = true; //naar eeprom, is bit 0 van memreg, true is altijd default voor de memreg bits
bool stepdrive = false;
bool  start = true; //eerste start
bool stand = false; //false =p1 true is p2
byte stepdoel = 0;
byte stepmovefase = 0;
byte memreg;
unsigned int standstep = 0; //huidige stand van de stepper
unsigned int standdoel = 0; //stop doel voor de stepper
unsigned int standp1 = 50; //ingeprogrammeerde stand rechts, P1
unsigned int standp2 = 3500; //ingeprogrammeerde stand links, P2
unsigned int standnahome = 0;
unsigned int displaycount = 0; //counter voor snelheid display 
byte displaycijfers[4]; //tijdelijke opslag berekende cijfers 0=1000tal 1=100 tal 2=10tal 3=eenheid


//temps
//int testnummer = 0;
//int displaytestcount = 0;
//byte displaydigit = 0;
//byte displaysegment = 0;

//algemeen en debugs
void Eeprom_write() {
	//algemeen
	memreg = 0xFF;
	if (stephomerichting == false)memreg &= ~(1 << 0);  //bit0=richting naar Home switch

	if (EEPROM.read(10) != memreg)EEPROM.write(10, memreg);


	//tbv.stappenmotor
	EEPROM.put(100, standp1); //checked automatisch of data is veranderd, alleen aanpassen als data veranderd is. spaart write cycles
	EEPROM.put(200, standp2);
	if (EEPROM.read(11) != speedmaxfactor)EEPROM.write(11, speedmaxfactor);
	if (EEPROM.read(12) != speedminfactor)EEPROM.write(12, speedminfactor);
	if (EEPROM.read(13) != afremfactor)EEPROM.write(13, afremfactor);
	//dcc
	EEPROM.put(50, decoderadres);

	Eeprom_read(); //data terug lezen.
}
void Eeprom_read() {
	//
	byte _temp = 0;
	//algemeen
	memreg = EEPROM.read(10); //default =0xFF
	stephomerichting = memreg & (1 << 0); //startrichting stepper. 


	//stappenmotor
	stephomerichting = memreg & (1 << 0); // bit 0 = opgeslagen stand stephomerichting true is standaard	
	EEPROM.get(100, standp1);
	if (standp1 > 3500) standp1 = 50; //default stand
	EEPROM.get(200, standp2);
	if (standp2 > 3500)standp2 = 400;
	speedmaxfactor = EEPROM.read(11); //max speed
	if (speedmaxfactor > 20) speedmaxfactor = 10; //10 snelheids stappen	
	maxspeed = (10000 + LimitSpeed) - (500 * speedmaxfactor);
	speedminfactor = EEPROM.read(12);
	if (speedminfactor > 20)speedminfactor = 5;
	minspeed = (LimitSpeed + 25000) - (speedminfactor * 1000);
	afremfactor = EEPROM.read(13);
	if (afremfactor > 20)afremfactor = 10;
	accelspeed = 10 * afremfactor;
	//DCC
	EEPROM.get(50, decoderadres);
	if (decoderadres > 512)decoderadres = 1;
	dccadres = 1 + ((decoderadres - 1) * 4); //eertse dccadres
}
void setup() {
	Serial.begin(9600);
	Dcc.pin(0, 2, 1); //interrupt number 0; pin 2; pullup to pin2
	Dcc.init(MAN_ID_DIY, 10, 0b10000000, 0); //bit7 true maakt accessoire decoder, bit6 false geeft decoder per decoderadres
	//Dcc.init(MAN_ID_DIY, 10, 0b11000000, 0); //bit7 true maakt accessoire decoder, bit6 true geeft decoder per adres
	//Pinnen
	DDRC = 0; //port C A0~A5 as inputs
	PORTC = 63; //pullup weerstanden op pin A0~A5, tbv. switches en sensoren
	DDRB = 15; //pinnen 8~11 als output
	PORTB &= ~(15 << 0); //zet pinnen 8~11 laag

	DDRD |= (1 << 7); //pins7,6,5,4 als output pin 7=bezet flag , pin 6 = SRCLK,  pin 5= RCLK,  pin 4=SER	
	DDRD |= (1 << 6); //srclk
	DDRD |= (1 << 5); //rclk
	DDRD |= (1 << 4); //ser

	Serial.println(PINC);

	//factory reset
	if (PINC == 41) { //knop 2 en 3 ingedrukt
		for (int i = 0; i < EEPROM.length(); i++) {
			if (EEPROM.read(i) < 0xFF)EEPROM.write(i, 0xFF);
			digit[0] = Letter('F');
			digit[1] = Letter('o');
			digit[2] = Letter('c');
			digit[3] = Letter('t');
			digit[1] |= (1 << 7); //dots uit
		}
	}
	else {
		DisplayMotor();
	}
	//initialisaties
	start = true;
	Eeprom_read();
	shiftbyte[0] = B11111001;
	shiftbyte[1] = 0;
	//DisplayMotor();
//	testnummer = 0;
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
			case 0: //uitschakelen spoelen na een puls				
				Stepoff();
				break;
			case 1: //wachttijd na draaiopdracht naar het draaien
				Step_move();
				break;
			case 2: //display verversen, dcc adres bv.
				DisplayShow();
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
//DCC
void notifyDccAccTurnoutBoard(uint16_t BoardAddr, uint8_t OutputPair, uint8_t Direction, uint8_t OutputPower) {
	//Serial.print("decoderadres "); Serial.println(BoardAddr);
	//Serial.print("channel "); Serial.println(OutputPair);
	byte _channel = 0;

	if (BoardAddr - (decoderadres) == 0) {
		_channel = OutputPair;
		Dcc_rx(_channel, Direction, OutputPower);
	}
	else {
		if (BoardAddr - (decoderadres) == 1) {
			_channel = OutputPair + 4;
			Dcc_rx(_channel, Direction, OutputPower);
		}
	}
}
void Dcc_rx(byte _channel, byte _port, bool _onoff) {
	//hier komen de dcc commands en kunnen ze worden toegewezen aan een functie, voorlopig even alleen de default

	switch (_channel) {
	case 0: //stepper
		//Serial.println("stepper");
		Step_stand(_port);
		break;
	case 1: //servo1
		//Serial.println("Servo1");
		break;
	case 2: //servo2
		break;
	case 3: //alarm
		break;
	case 4: //out1
		if (_port && _onoff) {			
				shiftbyte[1] |= (1 << 4);
			}
			else {
				shiftbyte[1] &= ~(1 << 4);
			}		
		break;
	case 5: //out2
		if (_port && _onoff) {
			shiftbyte[1] |= (1 << 5);
		}
		else {
			shiftbyte[1] &= ~(1 << 5);
		}
		break;
	case 6: //out3
		if (_port && _onoff) {
			shiftbyte[1] |= (1 << 6);
		}
		else {
			shiftbyte[1] &= ~(1 << 6);
		}
		break;
	case 7: //out4
		if (_port && _onoff) {
			shiftbyte[1] |= (1 << 7);
		}
		else {
			shiftbyte[1] &= ~(1 << 7);
		}
		break;

	}
}
//display
void Display_exe() {
	displaycount++;
	if (displaycount > 500) { //clockcycles
		displaycount = 0;
		digitcount++;
		if (digitcount > 3)digitcount = 0;
		shiftbyte[1] &= ~(15 << 0); //clear bits 0~3
		shiftbyte[1] |= (1 << digitcount);
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
	//maakt een 4 cyfer nummer en slaat dit op on cijfers[].

	byte  _cijfer[4];
	for (byte i = 0; i < 4; i++) {
		_cijfer[i] = 0;
	}
	while (_nummer >= 1000) {
		_cijfer[0]++;
		_nummer = _nummer - 1000;
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
		displaycijfers[i] = Cijfer(_cijfer[i]);
	}
}
void DisplayShow() {
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
		digit[0] = Letter('p');	digit[1] = Cijfer(0); digit[2] = Cijfer(5);
		if (stand) {
			digit[3] = Cijfer(1);
		}
		else {
			digit[3] = Cijfer(2);
		}
		break;
	case 2: //max speed stepper
		digit[0] = Cijfer(5);
		digit[1] = Letter('x'); //dots aan
		DisplayNummer(speedmaxfactor);//berekend de digits uit een nummer max 9999
		digit[2] = displaycijfers[2];
		digit[3] = displaycijfers[3];
		break;
	case 3: //min speed stepper
		digit[0] = Cijfer(5);
		digit[1] = Letter('|'); //dots aan
		DisplayNummer(speedminfactor); //berekend de digits uit een nummer max 9999
		digit[2] = displaycijfers[2];
		digit[3] = displaycijfers[3];
		break;
	case 4: //afremfactor *10=accelspeed.
		digit[0] = Cijfer(5);
		digit[1] = Letter('o');  //dots aan
		DisplayNummer(afremfactor); //berekend de digits uit een nummer max 9999
		digit[2] = displaycijfers[2];
		digit[3] = displaycijfers[3];
		break;
	case 5: //default richting stepper naar home
		digit[0] = Cijfer(5);
		digit[1] = Letter('-'); //met dots aan
		if (stephomerichting) {
			digit[2] = B11001110;
			digit[3] = B11111100;
		}
		else {
			digit[2] = B11011110;
			digit[3] = B11111000;
		}
		break;
	case 6:
		//twee standen de waarde of de text DCC
		if (dccprg) { //toon 'DCC'
			digit[0] = Cijfer(10);
			digit[1] = Letter('d');
			digit[2] = Letter('c');
			digit[3] = Letter('c');

			dccprg = false; //wachtlus instellen, zet dispaly om van 'dcc' ; naar de waarde
			waittime = 80;
			waitnext = 2;
		}
		else { //toon het DCC adres 			
			int _dccadres = 1 + ((decoderadres - 1) * 4);
			DisplayNummer(_dccadres);
			for (int i = 0; i < 4; i++) {
				digit[i] = displaycijfers[i];
			}
		}
		break;
	case 7: //functie van dccadres
		digit[0] = Letter('d'); digit[1] = Cijfer(1); dots;

		break;
	case 8: //functie van dccadres+1
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
	case 'd':
		_result = B10100001;
		break;
	case 'c':
		_result = B10100111;
		break;
	case '-':
		_result = B00111111;
		break;
	case 'F':
		_result = B10001110;
		break;
	case 't':
		_result = B10000111;
		break;
	case 'o': //kleine letter 0
		_result = B00100011;
		break;
	case 'p':
		_result = B10001100;
		break;

	case 'x':  //boven kapje aangeven maxspeed of zo met punten
		_result = B01011100;
		break;

	case '|': //onderkapje met punten
		_result = B01100011;
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
	//scrollen
	for (byte i = 0; i < 4; i++) {
		if (scrollmask & (1 << i)) { //alleen als scrollmask voor deze knop true is
			if ((_read & (1 << i)) == false) { //knop ingedrukt
				if (scrollcount[i] > 20) {
					SWon(i);
				}
				else {
					scrollcount[i]++;
				}
			}
		}
	}
	readlast = _read;
}
void SWon(byte _sw) {
	switch (_sw) {
	case 0: //switch 1
		if (programfase == 0) stand = !stand; //alleen positie wisselen in bedrijfsstand
		Step_stand(stand);
		break;
	case 1: //switch 2
		switch (programfase) {
		case 1: //positie stepper min, richting home switch
			Step_prg_pos(false);
			break;
		case 2: //maxspeed
			Prg_speed(0, false);
			break;
		case 3: //minspeed
			Prg_speed(1, false);
			break;
		case 4: //afremfactor
			Prg_speed(2, false);
			break;
		case 5: //default richting stepper
			Prg_home(false);
			break;
		case 6:
			Prg_dccadres(false);
			break;
		}
		break;

	case 2: //switch 3
		switch (programfase) {
		case 1: //positie stepper plus van home switch 
			Step_prg_pos(true);
			break;
		case 2: //maxspeed
			Prg_speed(0, true);
			break;
		case 3://minspeed
			Prg_speed(1, true);
			break;
		case 4://afremfactor, accelspeed
			Prg_speed(2, true);
			break;
		case 5: //default draairichting.
			Prg_home(true);
			break;
		case 6:
			Prg_dccadres(true);
		}
		break;
	case 3: //switch 4
		Prg_up();
		break;
	case 4: //Hall sensor voor positie
		Step_sensor(false);
		break;
	}
}
void SWoff(byte _sw) {
	scrollcount[_sw] = 0; //reset teller voor het scrollen
	switch (_sw) {
	case 4: //hall sensor positie
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
			unsigned long _aantalafremsteps = (minspeed - stepspeed) / accelspeed;

			if (_aantalsteps > _aantalafremsteps) { //versnellen
				if (stepspeed > accelspeed) stepspeed = stepspeed - accelspeed;  //geeft versnelling
			}
			else { //vertragen
				if (stepspeed < minspeed) stepspeed = stepspeed + accelspeed;  //geeft vertraging
			}
			steptimer = micros();
			Step_steps();
		}
	}
}
void Step_steps() {
	//stuurt de stappenmotor aan
	Stepoff(); //zet alle spoelen direct uit
	waittime = 10;
	waitnext = 0; //stepoff vertraagd

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
	if (standstep > standp2 + standp1 + 100) {
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
		standdoel = standp1; //terugdraaien?
	}
	else {
		standdoel = standp2;
	}
	stepmovefase = 10;
	//hier is een timer nodig
	waitnext = 1; //naar Step_move()
	waittime = 100;
	Prg_end();
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
		// Stepoff(); verplaatst naar een algemene wacht lus na iedere puls
		Bezet(false); //geef de schijf vrij, locs kunnen gaan rijden 
		DisplayShow();
		break;

	}
}
//program modes
void Prg_up() { //volgende programmamode
	programfase++;
	if (programfase == AantalProgramFases)programfase = 0;
	switch (programfase) {
	case 0: //in bedrijf
		Prg_end(); //afsluiten programmeerfase, opslaan data
		break;
	case 1: //posities standen instellen
		scrollmask = B0110;
		break;
	case 2: //maximale snelheid stappenmotor (herstart nodig, of eeprom read opnieuw???)
		scrollmask = 0;
		break;
	case 6: //DCC adres (altijd eerste channel van een decoder adres)
		scrollmask = B0110;
		dccprg = true;
		break;
	default:
		scrollmask = 0;
		break;
	}
	DisplayShow();
}
void Prg_end() { //einde programmamode
	//huidige programmamode opslaan	
	switch (programfase) {
	case 1:
		break;
	}
	Eeprom_write();
	programfase = 0;
	scrollmask = 0;
}
void Step_prg_pos(bool _richting) { //false is naar home, true is van home af
	steprichting = stephomerichting;
	if (_richting)steprichting = !steprichting;
	if (stand) {
		if (_richting) {
			standp1++;
		}
		else {
			if (standp1 > 0) standp1--;
		}
	}
	else {
		if (_richting) {
			standp2++;
		}
		else {
			if (standp2 > 0)standp2--;
		}
	}
	Step_steps();
}
void Prg_speed(byte _type, bool _plusmin) { //_type 0=maxspeedfactor, type 1=minspeedfactor, type 2=speedfactor, accelleratiefactor
	switch (_type) {
	case 0: //maxspeed
		if (_plusmin) {
			if (speedmaxfactor < 20)speedmaxfactor++;
		}
		else {
			if (speedmaxfactor > 0)speedmaxfactor--;
		}
		break;
	case 1: //minspeed
		if (_plusmin) {
			if (speedminfactor < 20)speedminfactor++;
		}
		else {
			if (speedminfactor > 0)speedminfactor--;
		}
		break;
	case 2: //afremfactor/accelspeed
		if (_plusmin) {
			if (afremfactor < 20)afremfactor++;
		}
		else {
			if (afremfactor > 0)afremfactor--;
		}
		break;
	}
	DisplayShow();
}
void Prg_home(bool _richting) {
	if (_richting) {
		stephomerichting = true;
	}
	else {
		stephomerichting = false;
	}
	DisplayShow();
}
void Prg_dccadres(bool plusmin) {
	dccprg = false;
	if (plusmin) { //decoderadres verhogen
		decoderadres++;
		if (decoderadres > maxdcc)decoderadres = 1; //maxdcc=een #define constante
	}
	else { //decoderadres verlagen
		decoderadres--;
		if (decoderadres < 1)decoderadres = maxdcc;
	}
	DisplayShow();
	waittime = 200;
	waitnext = 2; //displayshow()
	dccprg = true;
}