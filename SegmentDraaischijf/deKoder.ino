/*
 Name:		SegmentDraaischijf.ino
 Created:	11/27/2023 10:50:47 AM
 Author:	Rob Antonisse


 Arduino Uno sketch voor de aansturing van een segment draaischijf met twee sporen, zoals op de 'Baan'.


*/
//PINS 
//servo 1 op 12 portb 4 
//servo 2 op 13 PORTB 5


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
const byte AantalActies = 7; //uiteindelijk nog bepalen
const byte AantalOutputFuncties = 6; 
const byte StepMaxStops = 8; //max  posities voor de stappenmotor, = 1 decoderadres vol
const int ServoMinPositie = 80;
const int ServoMaxPositie = 310;
const int ServoMaxStops = 8;

//variabelen
unsigned int decoderadres = 1; //dccadres = (decoder adres-1) * 4 plus het channel(1~4) binair 0~3 dus deze ook plus 1 EEPROM 50
unsigned int dccadres = 1; //
bool dccprg = true; //true=text DCC false=Waarde dcc adres
byte stepaantalstops = 2; //EEprom 15+
unsigned int servorq[2]; // positie waar servo bij b=volgende puls naar toe gaat
unsigned int servocurrent[2]; //werkelijke positie van de servo
unsigned int servotarget[2]; //uiteindelijke doel van de servo
unsigned int servointeruptcount[2]; //teller in de ISR
byte servostopcount[2]; //teller voor uitzetten servo puls
byte servospeedcount[2];
byte servospeed[2];
byte servolangzaamcount[2];

byte servoaantalstops[2];
unsigned int servo1pos[8];
unsigned int servo2pos[8];
byte servostop[2];  //stop waar de servo nu in staat cq naar opweg is
byte servo; //de actieve servo 0 of 1

byte outputfunctie[4]; //ingestelde functie per output


unsigned long ServoTimer;
byte shiftbyte[2];
byte digit[4];
byte digitcount = 0;

byte actie; //wat is de ingestelde functie voor de knop1 actie knop
byte programfase = 0;

unsigned long slowtimer = 0;
unsigned int waittime = 0;
int waitnext = 0;

byte scrollcount[4]; //teller voor het ingedrukt houden van een knop
byte scrollmask = 0; //bepaald welke knoppen op welk moment moge scrollen

//byte readlast = 15; //B0000 1111   0=hoogactief(sensor)  1=laagactief(drukknop), laatst gelezen stand van de knoppen
byte readlast = B00001111;
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
bool confirm = false;

byte stepstop = 0; //begin positie van stepper na powerup

byte stepdoel = 0;
byte stepmovefase = 0;
byte memreg;

unsigned int standstep = 0; //huidige stand van de stepper
unsigned int standdoel = 0; //stop doel voor de stepper

unsigned int stepper[StepMaxStops]; //ingestelde posities van de stepper

unsigned int standnahome = 0;
unsigned int displaycount = 0; //counter voor snelheid display 
byte displaycijfers[4]; //tijdelijke opslag berekende cijfers 0=1000tal 1=100 tal 2=10tal 3=eenheid


ISR(TIMER1_COMPA_vect) {


	servointeruptcount[servo]++;
	if (servointeruptcount[servo] >= servocurrent[servo]) { //min 65 max 310 geeft 180graden servo
		TIMSK1 &= ~(1 << 1);
		servointeruptcount[servo] = 0;
		PORTB &= ~(1 << (4 + servo)); //4 of 5
	}
}

void Factory() {
	//factory reset
	if (confirm) {
		for (int i = 0; i < EEPROM.length(); i++) {
			if (EEPROM.read(i) < 0xFF)EEPROM.write(i, 0xFF);
		}
		Init();
	}
	else {
		confirm = true;
		DisplayShow();
	}
}
void Eeprom_write() {
	//algemeen
	memreg = 0xFF;
	if (stephomerichting == false)memreg &= ~(1 << 0);  //bit0=richting naar Home switch
	if (EEPROM.read(10) != memreg)EEPROM.write(10, memreg);

	//tbv.stappenmotor
	EEPROM.put(100, stepper[0]); //checked automatisch of data is veranderd, alleen aanpassen als data veranderd is. spaart write cycles
	EEPROM.put(110, stepper[1]);

	if (EEPROM.read(11) != speedmaxfactor)EEPROM.write(11, speedmaxfactor);
	if (EEPROM.read(12) != speedminfactor)EEPROM.write(12, speedminfactor);
	if (EEPROM.read(13) != afremfactor)EEPROM.write(13, afremfactor);
	if (EEPROM.read(14) != actie)EEPROM.write(14, actie); //dit is niet ideaal, misschien later iets beters voor verzinnen
	//dcc
	if (EEPROM.read(15) != stepaantalstops)EEPROM.write(15, stepaantalstops);
	if (EEPROM.read(16) != servoaantalstops[0])EEPROM.write(16, servoaantalstops[0]);
	if (EEPROM.read(17) != servoaantalstops[1])EEPROM.write(17, servoaantalstops[1]);
	if (EEPROM.read(18) != servospeed[0])EEPROM.write(18, servospeed[0]);
	if (EEPROM.read(19) != servospeed[1])EEPROM.write(19, servospeed[1]);

	EEPROM.put(50, decoderadres);

	//servo's
	for (byte i = 0; i < ServoMaxStops; i++) {
		EEPROM.put(300 + (i * 10), servo1pos[i]);
		EEPROM.put(400 + (i * 10), servo2pos[i]);
	}
	Eeprom_read(); //data terug lezen.
}
void Eeprom_read() {
	memreg = EEPROM.read(10); //default =0xFF
	stephomerichting = memreg & (1 << 0); //startrichting stepper. 
	actie = EEPROM.read(14); if (actie > AantalActies)actie = 1; //default stepper
	stepaantalstops = EEPROM.read(15); if (stepaantalstops > StepMaxStops)stepaantalstops = 2;
	servoaantalstops[0] = EEPROM.read(16);
	servoaantalstops[1] = EEPROM.read(17);
	servospeed[0] = EEPROM.read(18);
	servospeed[1] = EEPROM.read(19);
	for (byte i = 0; i < 2; i++) {
		if (servoaantalstops[i] > ServoMaxStops)servoaantalstops[i] = 2;
		if (servospeed[i] > 10)servospeed[i] = 8; //servo default
	}

	//instellen outputfuncties
	for (byte i = 0; i < 4; i++) {
		outputfunctie[i] = EEPROM.read(20 + i);
		if (outputfunctie[i] == 0xFF)outputfunctie[i] = 1; //default aan/uit
	}


	//stappenmotor
	stephomerichting = memreg & (1 << 0); // bit 0 = opgeslagen stand stephomerichting true is standaard	

	for (byte i = 0; i < StepMaxStops; i++) {
		EEPROM.get((i * 10) + 100, stepper[i]); //gebruikt 8 bytes
		if (stepper[i] > 9999 || stepper[i] == 0)stepper[i] = i * 100 + 50;
	}
	speedmaxfactor = EEPROM.read(11); //max speed
	if (speedmaxfactor > 20) speedmaxfactor = 10; //10 snelheids stappen	
	maxspeed = (10000 + LimitSpeed) - (500 * speedmaxfactor);
	speedminfactor = EEPROM.read(12);
	if (speedminfactor > 20)speedminfactor = 10;
	minspeed = (LimitSpeed + 25000) - (speedminfactor * 1000);
	afremfactor = EEPROM.read(13);
	if (afremfactor > 20)afremfactor = 10;
	accelspeed = 10 * afremfactor;

	//Servoos
	//standen servo 1
	for (byte i = 0; i < ServoMaxStops; i++) {
		EEPROM.get(300 + (i * 10), servo1pos[i]);
		if (servo1pos[i] > ServoMaxPositie)servo1pos[i] = ServoMinPositie + 10 + (i * 20);

		EEPROM.get(400 + (i * 10), servo2pos[i]);
		if (servo2pos[i] > ServoMaxPositie)servo2pos[i] = ServoMinPositie + 10 + (i * 20);
	}
	//DCC
	EEPROM.get(50, decoderadres);
	if (decoderadres > maxdcc)decoderadres = 1;
	dccadres = 1 + ((decoderadres - 1) * 4); //eerste dccadres

}
void debug() {
	for (byte i = 0; i < 4; i++) {
		digit[i] = Cijfer(1);
	}
	dots;
}
void setup() {
	Serial.begin(9600);
	Dcc.pin(0, 2, 1); //interrupt number 0; pin 2; pullup to pin2
	Dcc.init(MAN_ID_DIY, 10, 0b10000000, 0); //bit7 true maakt accessoire decoder, bit6 false geeft decoder per decoderadres
	//Dcc.init(MAN_ID_DIY, 10, 0b11000000, 0); //bit7 true maakt accessoire decoder, bit6 true geeft decoder per adres
	//Pinnen
	DDRC = 0; //port C A0~A3 as inputs 
	PORTC = B00001111; //pullup weerstanden op pin A0~A3, tbv. switches,   sensoren A4&A5 hebben een fysieke 10K pull down
	DDRB |= (63 << 0); //pinnen 8~13 als output
	PORTB &= ~(63 << 0); //zet pinnen 8~11 laag

	DDRD |= (1 << 7); //pins7,6,5,4 als output pin 7=bezet flag , pin 6 = SRCLK,  pin 5= RCLK,  pin 4=SER	
	DDRD |= (1 << 6); //srclk
	DDRD |= (1 << 5); //rclk
	DDRD |= (1 << 4); //ser

	//timer1  voor de servo's, puls van 10us maken (100xp=1ms 200xp=2ms)
	TCCR1A = 0; //Timer / Counter1 Control Register A
	TCCR1B = 0;//  Timer / Counter1 Control Register B
	TCCR1B |= (1 << 3); //WGM12 set, CTC mode (clear timer on compare met OCR1A)
	TCCR1B |= (1 << 0); //CS10 prescaler
	//TCCR1B |= (1 << 1); //CS11 prescaler
	//TCCR1B |= (1 << 2); //CS12 prescaler
	//TCCR1C  //Timer / Counter1 Control Register C
	//TCNT1H and TCNT1L – Timer / Counter1 //de feitelijke teller
	//OCR1A = 500;
	OCR1AH = 0;
	OCR1AL = 125;
	// 
	//OCR1AH and OCR1AL – Output Compare Register 1 A
	//TIMSK1 = 0;  //Timer / Counter1 Interrupt Mask Register
	//TIMSK1 |= (1 << 1); //Bit 1 – OCIE1A : Timer / Counter1, Output Compare A Match Interrupt Enable

	//initialisaties
	Init();
}
void Init() {
	Eeprom_read();

	servocurrent[0] = 150; servocurrent[1] = 150;



	start = true;
	servostop[0] = 0; servostop[1] = 0;
	programfase = 0;
	actie = 1;
	shiftbyte[0] = B11111001;
	shiftbyte[1] = 0;
	Shift();
	DisplayShow();
}

void loop()
{
	Dcc.process();
	Step_exe();
	Display_exe();

	if (millis() - ServoTimer > 4) {
		Servo_exe();
		ServoTimer = millis();
	}

	if (millis() - slowtimer > 20) { //timer op 20ms
		slowtimer = millis();
		SW_exe(); //uitlezen switches en  sensoren.
		Wait();
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
				//servo
			case 10:
				Servo1_move();
				break;
			case 11:
				Servo2_move();
				break;

			}
		}
	}
}

//byte beits[2];

void NoodStop() {
	//hier nog iets voor maken
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
		//positie stepper=stepstop
		StepperActie();
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
void DisplayNummer(int _nummer, byte _show) {
	//maakt een 4 cyfer nummer en slaat dit op on cijfers[].
	//hier kun je wat proces winst halen door de bool te veranderen in byte, en deze void de digits in laten schrijven

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
			if (_cijfer[2] == 0) {
				_cijfer[2] = 10;
			}
		}
	}

	for (byte i = 0; i < 4; i++) {
		displaycijfers[i] = Cijfer(_cijfer[i]);
		//displaycijfers[i] = _cijfer[i];
	}

	switch (_show) {
	case 4:
		for (int i = 0; i < 4; i++) {
			digit[i] = displaycijfers[i];
		}
		break;
	case 23:
		digit[2] = displaycijfers[2];
		digit[3] = displaycijfers[3];
		break;
	}
}
void DisplayShow() {
	switch (actie) { //functie van de actieknop
	case 0: //voor algemene programmeer stappen(dcc adres)
		DisplayCom();
		break;
	case 1: //bediening stepper
		DisplayStepper();
		break;
	case 2:
		DisplayServo(0);
		break;
	case 3:
		DisplayServo(1);
		break;
	case 4:
		DisplayOutput(0);
		break;
	case 5:
		DisplayOutput(1);
		break;
	case 6: 
		DisplayOutput(2);
		break;
	case 7:
		DisplayOutput(3);
		break;
	case 8:
		DisplayBezet();
		break;
	case 9:
		DisplayAlarm();
		break;
	case 10:
		DisplayHome();
		break;
	case 11:
		DisplaySensor();
		break;
	case 12:
		DisplaySequence();
		break;
	}
}
void DisplayGetal(byte _nummer, byte _digit) {
	byte _teken = 10;
	if (_nummer > 0)_teken = _nummer;
	digit[_digit] = Cijfer(_teken);
}
void DisplayStepper() {
	//toont texten aan de hand van de programmeer stand en laatste positie van de draaielementen
	switch (programfase) {
	case 0: //in bedrijf
		digit[0] = Letter('b');
		digit[1] = Cijfer(10);
		digit[2] = Letter('p');

		if (stepstop == 0) {
			digit[3] = Letter('-');
		}
		else {
			digit[3] = Cijfer(stepstop);
		}
		break;

	case 1: //posities standen van de steppenmotor
		if (stepstop > 0) {
			digit[0] = Letter('b');	digit[1] = Letter('_'); digit[2] = Letter('+');
			DisplayGetal(stepstop, 3);
		}
		else {
			digit[0] = Letter('b');
			digit[1] = Letter('-'); digit[2] = Letter('-'); digit[2] = Letter('-');
		}

		break;
	case 2: //aantal stops van de stepper
		digit[0] = Letter('b'); //b(yj48-28)c(ount)
		digit[1] = Letter('c');
		DisplayNummer(stepaantalstops, 23);
		//digit[2] = displaycijfers[2];
		//digit[3] = displaycijfers[3];
		break;
	case 3: //min speed stepper
		digit[0] = Letter('b');
		digit[1] = Letter('_'); dots;
		DisplayNummer(speedminfactor, 23);//berekend de digits uit een nummer max 9999
		//digit[2] = displaycijfers[2];
		//digit[3] = displaycijfers[3];
		break;
	case 4: //max speed stepper
		digit[0] = Letter('b');
		digit[1] = Letter('|'); dots;//streepje boven
		DisplayNummer(speedmaxfactor, 23); //berekend de digits uit een nummer max 9999
		//digit[2] = displaycijfers[2];
		//digit[3] = displaycijfers[3];
		break;
	case 5: //afremfactor *10=accelspeed.
		digit[0] = Letter('b');
		digit[1] = Letter('-');  dots;
		DisplayNummer(afremfactor, 23); //berekend de digits uit een nummer max 9999
		//digit[2] = displaycijfers[2];
		//digit[3] = displaycijfers[3];
		break;
	case 6: //default richting stepper naar home
		digit[0] = Letter('b');
		digit[1] = Letter('='); dots;
		if (stephomerichting) {
			digit[2] = Cijfer(10);
			digit[3] = B11001100;
		}
		else {
			digit[2] = Cijfer(10);
			digit[3] = B11011000;
		}
		break;
	}
}
void DisplayCom() { //algemene instellingen knop1 heeft (nog) geen functie
	digit[0] = Letter('c');
	switch (programfase) {
		//***********************
	case 0: //in bedrijf
		digit[1] = Cijfer(10);
		digit[2] = Cijfer(10);
		digit[3] = Cijfer(10);
		break;
		//**********************
	case 1: //instellen DCC decoder adres
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
			DisplayNummer(_dccadres, 4);
			//for (int i = 0; i < 4; i++) {
			//	digit[i] = displaycijfers[i];
			//}
		}
		break;
		//*************************
	case 2: //nog niks
		if (confirm) {
			digit[0] = Cijfer(5);
			digit[1] = Letter('U');
			digit[2] = Letter('A');
			digit[3] = Letter('E');
		}
		else {
			digit[0] = Letter('F');
			digit[1] = Letter('A');
			digit[2] = Letter('C');
			digit[3] = Letter('t');
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
		_result = B11111000;
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
	case '_':
		_result = B11110111;
		break;
	case '+':
		_result = B11111110;
		break;
	case '-':
		_result = B10111111;
		break;
	case 'A':
		_result = B10001000;
		break;
	case 'b':
		_result = B10000011;
		break;
	case 'C':
		_result = B11000110;
		break;
	case 'E':
		_result = B10000110;
		break;
	case 'F':
		_result = B10001110;
		break;
	case 't':
		_result = B10000111;
		break;
	case 'o': //kleine letter 0
		_result = B10100011;
		break;
	case 'p':
		_result = B10001100;
		break;
	case 'U':
		_result = B11000001;
		break;
	case 'x': //aanduiding voor servo1
		_result = B10010110;
		break;
	case 'q': //is teken voor aanduiding van servo 2 
		_result = B10100110;
		break;
	case '=':  //twee streepjes
		_result = B10110111;
		break;
	case '>': //teken voor speed verticaal = teken
		_result = B11101011;
		break;

	case '|': //streepje boven
		_result = B11111110;
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
				if (_read & (1 << i)) { //knop losgelaten, sensor actief
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
	//Serial.print("on  "); Serial.println(_sw);
	switch (_sw) {
		//*****************************************SWITCH1***********
	case 0: //switch 1
		switch (actie) {
		case 1: //stepper		
			if (programfase == 0) StepperActie(); //alleen positie wisselen in bedrijfsstand
			break;
		case 2: //actie: Servo 1
			ServoActie(0);
			break;
		case 3: //actie servo 2
			ServoActie(1);
			break;
		case 4: //actie output 1
			OutputActie(0);
			break;
		case 5: //actie output 2
			OutputActie(1);
			break;
		case 6: //actie output 3
			OutputActie(2);
			break;
		case 7://actie output 4
			OutputActie(3);
			break;
		}

		break;

		//*********************************SWITCH 2**********
	case 1: //switch 2
		switch (actie) {
		case 0: //actie: common
			switch (programfase) {
			case 1:
				Prg_comdccadres(false);
				break;
			}
			break;
		case 1: //actie: stepper
			switch (programfase) {
			case 0:
				Actie_exe(false); //knop 1 op lagere actie zetten
				break;
			case 1: //positie stepper min, richting home switch
				prg_steppos(false);
				break;
			case 2: //Aantal stops
				prg_stepaantalstops(false);
				break;
			case 3: //min speed
				Prg_stepspeed(1, false);
				break;
			case 4: //minspeed
				Prg_stepspeed(0, false);
				break;
			case 5: //afremfactor
				Prg_stepspeed(2, false);
				break;
			case 6: //default richting stepper
				Prg_stephome(true);
				break;
			}
			break;
		case 2: //actie: Servo 1
			switch (programfase) {
			case 0:
				Actie_exe(false);
				break;
			case 1: //instellen positie servo 1
				Prg_servopos(0, false);
				break;
			case 2: //dec aantal stops servo 1
				Prg_servoaantalstops(0, false);
				break;
			case 3: //dec servo 1 speed
				Prg_servospeed(0, false);
				break;
			}
			break;
		case 3: //Actie Servo 2
			switch (programfase) {
			case 0:
				Actie_exe(false);
				break;
			case 1: //instellen postie servo 2
				Prg_servopos(1, false);
				break;
			case 2: //dec aantal stops servo 2
				Prg_servoaantalstops(1, false);
				break;
			case 3: //dec servo 2 speed
				Prg_servospeed(1, false);
				break;
			}

		case 4: //Actie output 1
			OutputSwitch(0,false);
			break;
		case 5: //Actie output 2
			OutputSwitch(1, false);
			break;
		case 6: //Actie output 3
			OutputSwitch(2, false);
			break;
		case 7: //Actie output 4
			OutputSwitch(3, false);
			break;
		}
		break;

		//***********************************SWITCH 3******
	case 2: //switch 3
		switch (actie) {
		case 0: //actie: common
			switch (programfase) {
			case 0:
				Actie_exe(true);
				break;
			case 1:
				Prg_comdccadres(true);
				break; //programfase 1
			case 2: //factory reset
				Factory();
			}
			break; //actie common

		case 1: //actie: stepper
			switch (programfase) {
			case 0:
				Actie_exe(true); //knop1 op hogere actie zetten
				break;
			case 1: //positie stepper plus van home switch 
				prg_steppos(true);
				break;
			case 2:
				prg_stepaantalstops(true);
				break;
			case 3: //maxspeed
				Prg_stepspeed(1, true);
				break;
			case 4://minspeed
				Prg_stepspeed(0, true);
				break;
			case 5://afremfactor, accelspeed
				Prg_stepspeed(2, true);
				break;
			case 6: //default draairichting.
				Prg_stephome(false);
				break;
			}
			break;

		case 2: //actie servo 1
			switch (programfase) {
			case 0:
				Actie_exe(true);
				break;
			case 1:
				Prg_servopos(0, true);
				break;
			case 2: //inc servo 1 aantalstops
				Prg_servoaantalstops(0, true);
				break;
			case 3: //inc servo 1 speed
				Prg_servospeed(0, true);
				break;
			}
			break;


		case 3: //actie servo 2
			switch (programfase) {
			case 0: //in bedrijf
				Actie_exe(true);   //even nog niet er zijn (nu) niet meer acties
				break;
			case 1: //instellen positie van servo 2
				Prg_servopos(1, true);
				break;
			case 2: //inc aantal stops servo 2
				Prg_servoaantalstops(1, true);
				break;
			case 3: //inc servo 2 speed
				Prg_servospeed(1, true);
				break;
			}
			break;
		case 4: //output 1 actie
			OutputSwitch(0, true);
			break;
		case 5: //output2  actie
			OutputSwitch(1, true);
			break;
		case 6: //output 3, actie
			OutputSwitch(2, true);
			break;
		case 7:
			OutputSwitch(3, true);
			break;
		}
		break;
		//***********************************
	case 3: //switch 4 program switch
		Prg_up();
		break;
		//**************************************
	case 4: //Hall sensor voor positie
		//Step_sensor(false);
		break;
		//***********************************
	case 5: //hall sensor home
		Step_sensor(false);
		break;
	}
}
void SWoff(byte _sw) {
	scrollcount[_sw] = 0; //reset teller voor het scrollen
	//Serial.print("off  "); Serial.println(_sw);
	switch (_sw) {
	case 4: //
		//	Step_sensor(true);
		break;
	case 5: //hall sensor home
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
	if (standstep > stepper[1] + stepper[0] + 100) {
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
void StepperActie() {
	//verplaats de stappenmotor
	Bezet(true); //zet de schijf als bezet.
	//DisplayMotor();
	//stand = _stand;  //misschien is de extra variablele niet nodig..?


	stepstop++; //wordt dubbel gedaan
	if (stepstop > stepaantalstops)stepstop = 1;
	standdoel = stepper[stepstop - 1];
	stepmovefase = 10;

	//hier is een timer nodig
	waitnext = 1; //naar Step_move()
	waittime = 100;
	DisplayShow();

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

//program acties
void Actie_exe(bool _plusmin) {
	if (_plusmin)actie++; else actie--;   //dit werkt dus!
	if (actie > AantalActies)actie = 0;
	DisplayShow();
}
void Prg_up() { //volgende programmamode
	programfase++;
	switch (actie) {
	case 0: //common, algemene zaken	
		ProgramCom();
		break;
	case 1://stepper instellingen
		ProgramStep();
		break;
	case 2:
		ProgramServo(0);
		break;
	case 3:
		ProgramServo(1);
		break;
	case 4:
		ProgramOutput(0);
		break;
	case 5:
		ProgramOutput(1);
		break;
	case 6:
		ProgramOutput(2);
		break;
	case 7:
		ProgramOutput(3);
		break;
	}
	DisplayShow();
}
void Prg_end() { //einde programmamode
	//huidige programmamode opslaan	
	//DisplayShow();  //??
	programfase = 0;
	scrollmask = 0;
	Eeprom_write();
}
void Prg_comdccadres(bool plusmin) {
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
void prg_steppos(bool _richting) { //false is naar home, true is van home af
	if (stepstop == 0)return; //uitstappen als geen positie van de stepper bekend is

	steprichting = stephomerichting;
	if (_richting)steprichting = !steprichting;
	byte _p = stepstop - 1;
	if (_richting) {
		if (stepper[_p] < 9999) {
			stepper[_p]++;
			Step_steps();
		}
	}
	else {
		if (stepper[_p] > 0) { stepper[_p]--; Step_steps(); }
	}
	DisplayNummer(stepper[_p], 4);
}
void prg_stepaantalstops(bool plusmin) {
	if (plusmin) {
		if (stepaantalstops < StepMaxStops)stepaantalstops++;
	}
	else {
		if (stepaantalstops > 2)stepaantalstops--;
	}
	DisplayShow();
}
void Prg_stepspeed(byte _type, bool _plusmin) { //_type 0=maxspeedfactor, type 1=minspeedfactor, type 2=speedfactor, accelleratiefactor
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
void Prg_stephome(bool _richting) {
	if (_richting) {
		stephomerichting = true;
	}
	else {
		stephomerichting = false;
	}
	DisplayShow();
}

//servo's
void Servo_exe() {
	byte _langzamer = 0;
	byte _sneller = 1;

	//servo's om en om doen
	servo++; //deze 'servo' is dus de public servo, let op dat je deze niet verwart met de private '_servo'. 
	if (servo > 1)servo = 0;
	//uitstappen
	if (servostop[servo] == 0)return;
	if (servostopcount[servo] > 100)return;

	//snelheid implementeren >10 versnellen <10 vertragen, niks doen bij 10.		
	_langzamer = 10 - servospeed[servo];

	//huidige positie en daaruit voor komende taak bepalen en uitvoeren
	if (servocurrent[servo] == servotarget[servo]) {
		servostopcount[servo]++;
	}
	else {
		servolangzaamcount[servo]++;
		if (servolangzaamcount[servo] > _langzamer) {
			servolangzaamcount[servo] = 0;

			if (servocurrent[servo] < servotarget[servo]) {
				servocurrent[servo]++;

			}
			else {
				servocurrent[servo]--;
			}
		}
	}

	//puls starten, ISR zal de puls stoppen na een periode tussen 1 en 2 msec.
	PORTB |= (1 << (4 + servo));
	TIMSK1 |= (1 << 1);
}
void ServoActie(byte _servo) {
	servostop[_servo]++;
	if (servostop[_servo] > servoaantalstops[_servo])servostop[_servo] = 1;

	waitnext = 10 + _servo;
	waittime = 10;
	DisplayShow();
}
void Servo1_move() {

	servotarget[0] = servo1pos[servostop[0] - 1];
	servostopcount[0] = 0;

	//Serial.println(servotarget[0]);

}
void Servo2_move() {
	servotarget[1] = servo2pos[servostop[1] - 1];
	servostopcount[1] = 0;
}
void DisplayServo(byte _servo) {

	//if (_servo == 0)digit[0] = Letter('x'); else digit[0] = Letter('q'); //x/q=teken voor servo 1 of 2 
	digit[0] = Cijfer(_servo + 1);

	switch (programfase) {
	case 0: //in bedrijf

		digit[1] = Cijfer(10);
		digit[2] = Letter('p');
		if (servostop[_servo] == 0)digit[3] = Letter('-'); else digit[3] = Cijfer(servostop[_servo]);
		break;

	case 1: //posities instellen
		//if (_servo == 0)digit[0] = Letter('x'); else digit[0] = Letter('q'); //q=teken voor servo 2
		if (servostop[_servo] == 0) {
			digit[1] = Letter('-'); digit[2] = Letter('-');
		}
		else {
			digit[1] = Letter('_');
			digit[2] = Letter('|'); //bovenstreepje

			if (servostop[_servo] == 0)digit[3] = Letter('-'); else digit[3] = Cijfer(servostop[_servo]);
		}
		break;
	case 2: //aantal stops van een servo instellen, default =2
		digit[1] = Letter('c');
		DisplayNummer(servoaantalstops[_servo], 23); //false=niet het display vullen met het gevonden getal
		//digit[2] = Cijfer(displaycijfers[2]);
		//digit[3] = Cijfer(displaycijfers[3]);
		dots;
		break;
	case 3: //snelheid van servoos
		digit[1] = Letter('>'); //rechtopstaande =teken
		DisplayNummer(servospeed[_servo], 23);
		//digit[2] = Cijfer(displaycijfers[2]);
		//digit[3] = Cijfer(displaycijfers[3]);
		dots;
		break;
	}
}
void ProgramServo(byte _servo) { //called from prg_up
	//programfase inc gebeurt in prg_up
	if (programfase > 3)programfase = 0;
	scrollmask = 0;
	switch (programfase) {
	case 0:
		Prg_end();
		break;
	case 1: //instellen posities
		scrollmask = B0110;
		break;
	case 2: //aantal stops
		break;
	case 3: //snelheid, speed
		break;
	}
	//displayshow volgt nu in prg_up, dit verwijst door naar displayservo
}
void Prg_servopos(byte _servo, bool _plusmin) {
	//Serial.print("servo: "); Serial.print(_servo); Serial.print("   plusmin: "); Serial.println(_plusmin);
	//aanpassen positie
	//servostop loopt van 0~4 0 stand is onbekend
	//servo1pos en servo2pos lopen van 0~3 , servo#pos[0] = positie van servostop[1];
	if (servostop[_servo] == 0)return; //uitstappen
	byte _servostop = servostop[_servo] - 1; //stop omzetten in variable die de juiste stop in de servo posities aanduid,

	if (_servo == 0) {
		if (_plusmin) {
			if (servo1pos[_servostop] < ServoMaxPositie) servo1pos[_servostop]++;
		}
		else {
			if (servo1pos[_servostop] > ServoMinPositie)servo1pos[_servostop]--;
		}
		DisplayNummer(servo1pos[_servostop], 4); //plaats nummer in het display
		Servo1_move();
	}
	else {
		if (_plusmin) {
			if (servo2pos[_servostop] < ServoMaxPositie)servo2pos[_servostop]++;
		}
		else {
			if (servo2pos[_servostop] > ServoMinPositie)servo2pos[_servostop]--;
		}
		DisplayNummer(servo2pos[_servostop], 4); //plaats nummer in het display
		Servo2_move();
	}
}
void Prg_servoaantalstops(byte _servo, bool _plusmin) {
	if (_plusmin) {
		if (servoaantalstops[_servo] < ServoMaxStops)servoaantalstops[_servo]++;
	}
	else {
		if (servoaantalstops[_servo] > 2)servoaantalstops[_servo]--;
	}
	DisplayShow();
}

void Prg_servospeed(byte _servo, bool _plusmin) {
	Serial.println(_plusmin);
	if (_plusmin) {
		if (servospeed[_servo] < 10)servospeed[_servo]++;
	}
	else {
		if (servospeed[_servo] > 1)servospeed[_servo]--;
	}
	DisplayShow();
}
//program modes
void ProgramCom() { //afhandelen program fase in common
	//called from Prg-up (verhoogt de programfase)
	if (programfase > 2)programfase = 0; //aantal programfases per program reeks verschillend
	switch (programfase) {
	case 0:
		Prg_end();
		break;
	case 1:  //instellen DCC adres
		scrollmask = B0110;
		dccprg = true;
		break;
	case 2: //nog niks, voorlopig ff factory
		scrollmask = 0;
		confirm = false;
		break;
	}
	//terug naar prg_up waarin daarna Displayshow>Displaycom
}
void ProgramStep() { //afhandelen programreeks voor stepper
	//called from prg_up
	if (programfase > 6)programfase = 0; //aantal programfases per program reeks verschillend
	scrollmask = 0;
	switch (programfase) {
	case 0:
		Prg_end();
		break;
	case 1: //positie instellen
		if (stepstop > 0) {
			scrollmask = B0110;
		}
		else {
			scrollmask = 0;
		}
		break;
	}
	//hierna displayshow>displaystepper
}

//outputs
void OutputSwitch(byte _out, bool _plusmin) {
	switch (programfase) {
	case 0: //in bedrijf
		Actie_exe(_plusmin);
		break;
	case 1: //instellen functie
		if (_plusmin) {
			if (outputfunctie[_out] < AantalOutputFuncties)outputfunctie[_out]++;
		}
		else {
			if (outputfunctie[_out] > 2)outputfunctie[_out]--;
		}
		break;
	}
	DisplayShow();
}
void OutputActie(byte _out) {
	DisplayShow();
}
void ProgramOutput(byte _out) {
	if (programfase > 1)programfase = 0;
	scrollmask = 0;
	switch(programfase){
	case 1: //instellen functie van de output
		break;
	}
}
void DisplayOutput(byte _out) {
	switch (programfase) {
	case 0:
	digit[0] = Letter('o');
		break;
	case 1:
		digit[0] = Letter('F');
		break;
	}

	digit[1] = Cijfer(_out + 1);
	dots;
	DisplayNummer(outputfunctie[_out], 23);
}
//Bezet
	void DisplayBezet() {
	}
//Alarm
	void DisplayAlarm() {

	}
	//sensor Home
	void DisplayHome() {

	}
	//Sensor
	void DisplaySensor() {

	}
	//seqence programma volgorde
	void DisplaySequence() {

	}