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
#define maxdcc 479  //maximaal aantal dcc decoderadressen
#define dots digit[1]&=~(1<<7); //de punten in het display, aanzetten
#define clear23 digit[2] = Cijfer(10); digit[3] = Cijfer(10); //laatste twee digits uit
#define line23 digit[2]=Letter('-');digit[3]=Letter('-');

#define On1 outputonoff |=(1 << 0); //betreft output1
//#define On1 PWM_output(4,true); //1==4
//#define On1 shiftbyte[1] |= (1 << 4); 
#define On2 outputonoff |= (1 << 1); 
#define On3 outputonoff |= (1 << 2); 
#define On4 outputonoff |= (1 << 3);
#define OnBezet PORTD |=(1<<7);
#define OnAlarm PORTD |=(1<<3);
#define Off1 outputonoff &= ~(1 << 0);
//#define Off1 PWM_output(4,false);
//#define Off1 shiftbyte[1] &=~(1 << 4); 
#define Off2 outputonoff &=~(1 << 1);
#define Off3 outputonoff &=~(1 << 2);
#define Off4 outputonoff &=~(1 << 3); 
#define OffBezet PORTD &=~(1<<7);
#define OffAlarm PORTD &=~(1<<3);
//#define Toggle1 shiftbyte[1] ^=(1 << 4); 
//#define Toggle2 shiftbyte[1] ^=(1 << 5); 
//#define Toggle3 shiftbyte[1] ^=(1 << 6); 
//#define Toggle4 shiftbyte[1] ^=(1 << 7); 
#define ToggleBezet PIND |=(1<<7);
#define ToggleAlarm PIND |=(1<<3);

const int LimitSpeed = 900; //snelheid begrenzen stepper
//const byte AantalProgramFases = 7; niet in gebruik
const byte AantalActies = 19; //19 zal de laatste blijven? uiteindelijk nog bepalen 
const byte AantalOutputOpties = 14; //0=niks 1=bezet 2=alarm 3=o1 4=o2 5=o3 6=o4 7~14 timers 1~8
const byte StepMaxStops = 8; //max  posities voor de stappenmotor, = 1 decoderadres vol
const int ServoMinPositie = 1000; //minimale positie voor de servo's 180graden
const int ServoMaxPositie = 5000; //idem maximaal
const int ServoMaxStops = 8; //aantal stops
const int ServoMaxSpeed = 50; //
const byte TimerAantalFases = 6; //maximaal aantal verschillende outputs lus van een timer

const byte servocyclespeed = 5; //servo pult komt om de 10 ms (2x servo x 5ms)

//variabelen
//outputs
byte pwmoncount[8];
byte outputonoff; //bit0=output1 bit 4=outbezet bit 5 is out alarm
byte pwmcount[8]; //duty cycle counter, de off tijd van het pwm signaal
byte pwmduty[8]; //duty cycle van de ppwm voor de output EEPROM 80 instelling
byte pwmdutycycle[8]; //dutycycle van een output om dit moment
unsigned int outputpwmspeed[8]; //snelheid van dec en inc van de dutycycle voor outputs  EEPROM 900 instelling
unsigned int outputpwmspeedcount[8]; //teller telt de periode tussen twee dec of inc van de duty cycle

unsigned int decoderadres = 1; //dccadres = (decoder adres-1) * 4 plus het channel(1~4) binair 0~3 dus deze ook plus 1 EEPROM 50
unsigned int dccadres = 1; //
byte dcctype; //hoofdinstelling van de DCC ontvangst 0=niet,4 8 16 32
bool dccprg = true; //true=text DCC false=Waarde dcc adres //nodig voor wisselen tussen text 'DCC' en ingestelde adres.

//max 32 adressen
byte dcctimer[32]; //timer voor dit adres
byte dcclaatstecommand[32]; //laatst ontvangen command op dit adres

//delen van het ontvangen dcc command als variabelen om veel doorgeven van deze variabelen tegen te gaan
byte dcckanaal;
bool dccport; //rechtdoor of afslaand
bool dcconoff; //aan of uit

byte stepaantalstops = 2; //EEprom 15+

unsigned long servopulscounter;

unsigned int servorq[2]; // positie waar servo bij b=volgende puls naar toe gaat
unsigned int servocurrent[2]; //werkelijke positie van de servo
unsigned int servotarget[2]; //uiteindelijke doel van de servo
unsigned int servointeruptcount[2]; //teller in de ISR
byte servostopcount[2]; //teller voor uitzetten servo puls
byte servostep[2]; //afstand in 1 servo cycle
byte servospeed[2]; //=is de maxmaal te behalen snelheid

byte servoaantalstops[2];
unsigned int servo1pos[8];
unsigned int servo2pos[8];

byte servostop[2];  //stop waar de servo nu in staat cq naar opweg is
byte servostopdisplay[2]; //ositie getoond in display
byte servo; //de actieve servo 0 of 1
byte servobezetoutput[2];
byte servotimeroutput[2][2]; //[]=servo 0/1 [0]=starttimer/[1]=end timer
byte servowaittime[2];


//temp
unsigned long testtimer = 0;
unsigned long testtijd = 0;


byte outputfase[4];

byte sensoroutput; //ingestelde output voor de sensor default is 0, niks
byte homeoutput; //ingestelde extra output voor home, default =0, home voor de stepper is vast ingesteld, dit komt eraan parallel
byte sensorfunctie[2];  //0=aan en uit zetten, 1=alleen aanzetten 2=alleen uitzetten  (0=home 1=sensor) 


byte Timer = 0; //8 bits 0=timer1, 1=timer2 2=timer3 enz.
byte timerscale[8]; //0=10ms(default)  1=100ms  2=1 seconde EEPROM 70~77
byte timerscalecounter[8]; //teller voor het aftellen van de prescaler 
byte timerprescaler[8]; //waarde van de prescaler 0,10 of 100
unsigned int timerontijd[8];
unsigned int timerofftijd[8];
unsigned int timercount[8]; //teller voor de timers 

byte timeroutput[8][6]; //2 timers, iedere timer 6 outputs (EEPROM)
byte timerkeuzeoutput[8]; //welke output ben je aan het instellen 
byte timercycles[8]; //hoe vaak de timer cycle wordt doorlopen 0= continue (default), 200-max (EEPROM)
byte timercyclecount[8]; //teller voor het aantal doorlopen cycles
bool timeronoff[8]; //timed de timer de ontime of de  offtime, true = offtime
byte timerfase[8]; //met welke fase, actie is de timer bezig
byte timeraantaloutputs[8];
byte timerstoppen = B11111111; //als false dan wordt timer alleen gewisseld nooit gestopt


byte actienastart; //welke actie er wordt ingestart na powerup, 0 (geen) is default.

unsigned long ServoTimer;
byte shiftbyte[2];
byte digit[4];
byte digitcount = 0;

byte actie; //wat is de ingestelde functie voor de knop1 actie knop
byte actieknopkeuze;
byte programfase = 0;

unsigned long slowtimer = 0;
unsigned int waittime = 0;
int waitnext = 0;

byte scrollcount[4]; //teller voor het ingedrukt houden van een knop
byte scrollmask = 0; //bepaald welke knoppen op welk moment moge scrollen
byte scrollspeed = 0;

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
bool stepdrive = false; //stepper draaien
bool stepdrivelaatst = false; //opslaan stepper draaien
//als de timer voor een servo loopt is het draaien van de stepper onderbroken.
bool  start = true; //eerste start
bool confirm = false;
bool stepperisbezet = false;

byte stepstop = 0; //begin positie van stepper na powerup

byte stepdoel = 0;
byte stepmovefase = 0;
byte stepbezetoutput; // EEPROM 21
byte stepoutputtimer[2]; //0=timer bij start 0=timer na stop
byte stepwaittime;
byte memreg;

unsigned int standstep = 0; //huidige stand van de stepper
unsigned int standdoel = 0; //stop doel voor de stepper

unsigned int stepper[StepMaxStops]; //ingestelde posities van de stepper

unsigned int standnahome = 0;
unsigned int displaycount = 0; //counter voor snelheid display 
byte displaycijfers[4]; //tijdelijke opslag berekende cijfers 0=1000tal 1=100 tal 2=10tal 3=eenheid

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
		DisplayShow(1);
	}
}
void Eeprom_write() {
	//algemeen
	memreg = 0xFF;
	if (stephomerichting == false)memreg &= ~(1 << 0);  //bit0=richting naar Home switch
	if (EEPROM.read(10) != memreg)EEPROM.write(10, memreg);

	//stappenmotor stops opslaan
	
	

	//kan ook met update, weet nu niet of dit sneller gaat
	if (EEPROM.read(11) != speedmaxfactor)EEPROM.write(11, speedmaxfactor);
	if (EEPROM.read(12) != speedminfactor)EEPROM.write(12, speedminfactor);
	if (EEPROM.read(13) != afremfactor)EEPROM.write(13, afremfactor);
	if (EEPROM.read(14) != actieknopkeuze)EEPROM.write(14, actieknopkeuze);
	//dcc
	if (EEPROM.read(15) != stepaantalstops)EEPROM.write(15, stepaantalstops);
	if (EEPROM.read(16) != servoaantalstops[0])EEPROM.write(16, servoaantalstops[0]);
	if (EEPROM.read(17) != servoaantalstops[1])EEPROM.write(17, servoaantalstops[1]);
	if (EEPROM.read(18) != servospeed[0])EEPROM.write(18, servospeed[0]);
	if (EEPROM.read(19) != servospeed[1])EEPROM.write(19, servospeed[1]);
	if (EEPROM.read(20) != stepbezetoutput)EEPROM.write(20, stepbezetoutput);
	if (EEPROM.read(21) != servobezetoutput[0])EEPROM.write(21, servobezetoutput[0]);
	if (EEPROM.read(22) != servobezetoutput[1])EEPROM.write(22, servobezetoutput[1]);
	EEPROM.update(23, homeoutput);
	EEPROM.update(24, sensoroutput);
	EEPROM.update(25, stepoutputtimer[0]);
	EEPROM.update(26, stepoutputtimer[1]);
	EEPROM.update(27, timerstoppen);

	EEPROM.update(28, servotimeroutput[0][0]);
	EEPROM.update(29, servotimeroutput[0][1]);
	EEPROM.update(30, servotimeroutput[1][0]);
	EEPROM.update(31, servotimeroutput[1][1]);
	EEPROM.update(32, dcctype);
	EEPROM.update(33, actienastart);
	EEPROM.update(34, sensorfunctie[0]);
	EEPROM.update(35, sensorfunctie[1]);

	EEPROM.put(50, decoderadres); //zijn 8 bytes


	//servo's
	for (byte i = 0; i < ServoMaxStops; i++) {
		EEPROM.put(300 + (i * 10), servo1pos[i]);
		EEPROM.put(400 + (i * 10), servo2pos[i]);
	}

	//timers en ander 8 bits data array
	for (byte i = 0; i < 8; i++) {
		EEPROM.put(500 + (i * 10), timerontijd[i]);
		EEPROM.put(600 + (i * 10), timerofftijd[i]);
		EEPROM.update(800 + i, timercycles[i]);

		for (byte op = 0; op < 6; op++) {
			EEPROM.update(700 + (i * 10) + op, timeroutput[i][op]);
		}
		EEPROM.update(70 + i, timerscale[i]);  //10ms, 100ms of 1 seconde
		EEPROM.update(80 + i, pwmduty[i]);  //dutyo cycle pwm outputs 1~100% (100==00)
		EEPROM.put(900 + (i * 10), outputpwmspeed[i]); //snelheid van verandering van de pwm dutycycle

		EEPROM.put(100 + (10*i), stepper[i]); //instellingen stops

	}

	Eeprom_read(); //data terug lezen.
}
void Eeprom_read() {
	memreg = EEPROM.read(10); //default =0xFF
	stephomerichting = memreg & (1 << 0); //startrichting stepper. 
	actieknopkeuze = EEPROM.read(14);
	if (actieknopkeuze > AantalActies)actieknopkeuze = 1; //default stepper
	if (!confirm) actie = actieknopkeuze;

	stepaantalstops = EEPROM.read(15); if (stepaantalstops > StepMaxStops)stepaantalstops = 2;
	servoaantalstops[0] = EEPROM.read(16);
	servoaantalstops[1] = EEPROM.read(17);
	servospeed[0] = EEPROM.read(18);
	servospeed[1] = EEPROM.read(19);
	servobezetoutput[0] = EEPROM.read(21);
	servobezetoutput[1] = EEPROM.read(22);
	stepoutputtimer[0] = EEPROM.read(25);
	stepoutputtimer[1] = EEPROM.read(26);
	timerstoppen = EEPROM.read(27);
	servotimeroutput[0][0] = EEPROM.read(28); if (servotimeroutput[0][0] > 50)servotimeroutput[0][0] = 0;
	servotimeroutput[0][1] = EEPROM.read(29); if (servotimeroutput[0][1] > 50)servotimeroutput[0][1] = 0;
	servotimeroutput[1][0] = EEPROM.read(30); if (servotimeroutput[1][0] > 50)servotimeroutput[1][0] = 0;
	servotimeroutput[1][1] = EEPROM.read(31); if (servotimeroutput[1][1] > 50)servotimeroutput[1][1] = 0;
	dcctype = EEPROM.read(32); if (dcctype == 0xFF)dcctype = 2;
	actienastart = EEPROM.read(33); if (actienastart == 0xFF)actienastart = 0;
	sensorfunctie[0] = EEPROM.read(34);
	sensorfunctie[1] + EEPROM.read(35);

	for (byte i = 0; i < 2; i++) {
		if (servoaantalstops[i] > ServoMaxStops)servoaantalstops[i] = 2;
		if (servospeed[i] > ServoMaxSpeed)servospeed[i] = 8; //servo default
		if (servobezetoutput[i] > AantalOutputOpties)servobezetoutput[i] = 0;
		servocurrent[i] = ServoMinPositie + ((ServoMaxPositie - ServoMinPositie) / 2);
		if (stepoutputtimer[i] > 10)stepoutputtimer[i] = 0;
		if (sensorfunctie[i] == 0xFF)sensorfunctie[i] = 0;
	}


	homeoutput = EEPROM.read(23); if (homeoutput > 50)homeoutput = 0;
	sensoroutput = EEPROM.read(24); if (sensoroutput > 50)sensoroutput = 0;
	//timers

	for (byte i = 0; i < 8; i++) {
		EEPROM.get(500 + (i * 10), timerontijd[i]);
		EEPROM.get(600 + (i * 10), timerofftijd[i]);
		timercycles[i] = EEPROM.read(800 + i);



		//1 voorbeeld maken op timer 1
		if (i == 0) { //timer 1
			if (timerontijd[0] > 9999)timerontijd[0] = 33; //33x20ms??? = ongeveer 90x per minuut
			if (timerofftijd[0] > 9999) timerofftijd[0] = 1;
		}
		else {
		if (timerontijd[i] > 9999)timerontijd[i] = 100; //33x20ms??? = ongeveer 90x per minuut
		if (timerofftijd[i] > 9999) timerofftijd[i] = 100;
		}
	if (timercycles[i] > 99)timercycles[i] = 0; //0=continue defaultwaarde

		for (byte op = 0; op < 6; op++) {
			timeroutput[i][op] = EEPROM.read(700 + (i * 10) + op);
			if (timeroutput[i][op] > 50) {
				timeroutput[i][op] = 0;
				//default instellingen timers	
				if (i == 0 && op == 0)timeroutput[0][0] = 4; //output1
				if (i == 0 && op == 1)timeroutput[0][1] = 5;//output2
			}
			//timer scales
			timerscale[i] = EEPROM.read(70 + i);
			if (timerscale[i] > 2)timerscale[i] = 0; //default 10ms per click
			//outputs duty cycle

			pwmduty[i] = EEPROM.read(80 + i);
			if (pwmduty[i] > 100)pwmduty[i] = 100; //dit moet nog worden omgekeerd, maar is logischer bij instellen.
			//pwmspeed[i] = EEPROM.read(90 + i);

			EEPROM.get(900 + (i * 10), outputpwmspeed[i]);
			if (outputpwmspeed[i] > 9999)outputpwmspeed[i] = 0; //aantal stappen van 10ms
		}

	}

	//stappenmotor
	stepbezetoutput = EEPROM.read(20); if (stepbezetoutput > 100)stepbezetoutput = 1; //1 moe dan bezet output worden, hier kunnen dus ook timers komen
	stephomerichting = memreg & (1 << 0); // bit 0 = opgeslagen stand stephomerichting true is standaard	

	for (byte i = 0; i < 8; i++) {
		EEPROM.get((i * 10) + 100, stepper[i]); //gebruikt 8 bytes
		if (stepper[i] > 9999)stepper[i] = 50 +(i*300); //optie or stepper[i]==0 weggehaald
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
	//standen servo 1, 
	unsigned int _steps = (ServoMaxPositie - ServoMinPositie) / 10;
	for (byte i = 0; i < ServoMaxStops; i++) {
		EEPROM.get(300 + (i * 10), servo1pos[i]);
		if (servo1pos[i] > ServoMaxPositie)servo1pos[i] = (ServoMinPositie + _steps) + (i * _steps);

		EEPROM.get(400 + (i * 10), servo2pos[i]);
		if (servo2pos[i] > ServoMaxPositie)servo2pos[i] = (ServoMinPositie + _steps) + (i * _steps);

		//Serial.println(servo1pos[i]);
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

	//Dcc.pin(0, 2, 1); //interrupt number 0; pin 2; pullup to pin2
	//Dcc.init(MAN_ID_DIY, 10, 0b10000000, 0); //bit7 true maakt accessoire decoder, bit6 false geeft decoder per decoderadres

	DDRC = 0; //port C A0~A3 as inputs 
	PORTC = B00001111; //pullup weerstanden op pin A0~A3, tbv. switches,   sensoren A4&A5 hebben een fysieke 10K pull down
	DDRB |= (63 << 0); //pinnen 8~13 als output
	PORTB &= ~(63 << 0); //zet pinnen 8~11 laag

	DDRD |= (1 << 7); //pins7,6,5,4 als output pin 7=bezet flag , pin 6 = SRCLK,  pin 5= RCLK,  pin 4=SER	pin3=alarm
	DDRD |= (1 << 6); //srclk
	DDRD |= (1 << 5); //rclk
	DDRD |= (1 << 4); //ser
	DDRD |= (1 << 3); //output Alarm


	//timer1  voor de servo's, puls van 10us maken (100xp=1ms 200xp=2ms)
	TCCR1A = 0; //Timer / Counter1 Control Register A
	//TCCR1B = 0;//  Timer / Counter1 Control Register B

	//TCCR1B |= (1 << WGM12); //WGM12 set, CTC mode (clear timer on compare met OCR1A)
	//TCCR1B |= (1 << 0); //CS10 prescaler
	//TCCR1B |= (1 << 1); //CS11 prescaler
	//TCCR1B |= (1 << 2); //CS12 prescaler
	// 
	//TCCR1C  //Timer / Counter1 Control Register C
	//TCNT1H and TCNT1L � Timer / Counter1 //de feitelijke teller

	//TCCR1B = 10;


	OCR1A = ServoMinPositie; //700
	//OCR1A = ServoMaxPositie; //5200
	//OCR1A = 5200;

	//OCR1AH = 60;
	//OCR1AL = 0;

	TIMSK1 |= (1 << 1);


	//timer voor timers 1ms
	TCCR2A = 0;  // Zet alle bits van register A op 0
	TCCR2B = 0;  // Zet alle bits van register B op 0
	// Timer 2 configuratie
	// Waveform Generation Mode: CTC (Clear Timer on Compare Match)
	// Clock Source: Internal clock, no prescaler (fastest clock)
	TCCR2A |= (1 << WGM21);
	TCCR2B |= (7 << 0);
	// Vergelijkswaarde instellen voor 1 ms puls (16 MHz / 1000 Hz)
	OCR2A = 155;  //geeft een puls van 9.994ms
	TIMSK2 |= (1 << OCIE2A); // Timer 2 interrupt inschakelen
	Init();


	//DCC aan of uit
	if (dcctype > 0) {
		Dcc.pin(0, 2, 1); //interrupt number 0; pin 2; pullup to pin2
		Dcc.init(MAN_ID_DIY, 10, 0b10000000, 0); //bit7 true maakt accessoire decoder, bit6 false geeft decoder per decoderadres
	}


	//autostart, vertraagd
	waitnext = 1;
	waittime = 100;
	Wait();
}
void Init() {
	Eeprom_read();
	//outputs
	for (byte i = 0; i < 8; i++) {
		pwmdutycycle[i] = 100;
	}
	start = true;
	servostop[0] = 0; servostop[1] = 0;
	programfase = 0;
	Timer = 0; //alle timers inactief
	shiftbyte[0] = B11111001;
	shiftbyte[1] = 0;
	Shift();
	DisplayShow(2);
}
void loop()
{
	Dcc.process();
	Step_exe();
	Display_exe();

	if (millis() - ServoTimer > servocyclespeed) {
		Servo_exe();
		ServoTimer = millis();
	}
	if (millis() - slowtimer > 8) { //timer op 10ms
		//if (Timer > 0) Timer_exe(); //alleen als er timers 'aan' staan
		slowtimer = millis();
		GPIOR0 ^= (1 << 0);
		if (GPIOR0 & (1 << 0)) { //timer 2x10=20ms
			SW_exe(); //uitlezen switches en  sensoren.
			Wait();
			WaitStepper();
			WaitServo();
			DccTimers();
		}
	}
}

ISR(TIMER1_COMPA_vect) {
	cli();
	TCCR1B = 0; //zet de timer weer uit
	TCNT1 = 0; //reset de counter?
	PORTB &= ~(1 << (4 + servo)); //zet servo pin weer laag

	//geef stepper weer vrij
	stepdrive = stepdrivelaatst;

	sei();
}

ISR(TIMER2_COMPA_vect) {
	if (Timer > 0)	Timer_exe(); //called om de 10ms
	//OutputPwm();
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

void WaitStepper() {
	//wachtlus voor het begin van een draaisessie
	if (stepwaittime > 0) {
		stepwaittime--;
		if (stepwaittime == 0) {
			stepperisbezet = true;

			if (stepoutputtimer[0] > 0) { //timer 1=bit0 in het Timer register
				if (Timer & (1 << stepoutputtimer[0] - 1)) { //timer loopt nog
					//wachtlus opnieuw starten 
					stepwaittime = 200;
				}
				else {  //timer is gestopt
					Step_move();
				}
			}
			else {
				Step_move();
			}
		}
	}
}
void WaitServo() {
	for (byte s = 0; s < 2; s++) {  //2 servo's
		if (servowaittime[s] > 0) { //wachttijd nog niet afgelopen	

			servowaittime[s]--; //verlaag de wachttijd

			if (servowaittime[s] == 0) { //wacht tijd afgelopen

				//Serial.println("servowait");

				if (servotimeroutput[s][0] > 0) { //er is een starttimer ingesteld

					if (Timer & (1 << servotimeroutput[s][0] - 1)) { //timers van 0~7  timer is actief
						servowaittime[s] = 50; //wachtlus opnieuw
					}
					else {
						if (s == 0) {
							Servo1_move();
						}
						else {
							Servo2_move();
						}
					}
				}
				else {  //geen timer ingesteld
					if (s == 0) {
						Servo1_move();
					}
					else {
						Servo2_move();
					}
				}
			}
		}
	}
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
			case 1: //autostart na wachttijd
				autostart();
				break;
			case 2: //display verversen, dcc adres bv.
				DisplayShow(3);
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
void NoodStop() {
	//hier nog iets voor maken
}

void autostart() {
	if (actienastart > 0) {
		AutoStartActie(actienastart);
	}
}
void AutoStartActie(byte _actie) {
	switch (_actie) {
	case 0:		break;
	case 1: //stepper
		StepperActie();
		break;
	case 2:
		ServoActie(0);
		break;
	case 3:
		ServoActie(1);
		break;
	case 4:
		On1;
		break;
	case 5:
		On2;
		break;
	case 6:
		On3;
		break;
	case 7:
		On4;
		break;
	case 8:
		OnBezet;
		break;
	case 9:
		OnAlarm;
		break;
	case 10:
		//home overslaan
		break;
	case 11:
		//sensor overslaan
		break;
	default: //timers 1~8
		TimerSwitch(0, _actie - 12);
		break;
	}
}

//DCC
void notifyDccAccTurnoutBoard(uint16_t BoardAddr, uint8_t OutputPair, uint8_t Direction, uint8_t OutputPower) {
	//Serial.print("boardadres "); Serial.print(BoardAddr);	Serial.print("   channel "); Serial.print(OutputPair); Serial.print("   port:"); Serial.println(Direction);
	//eerst filteren, terugbrengen naar 1~32 adressen
	//Serial.print("decoderadres "); Serial.println(decoderadres);
	unsigned int _decoder = 0; byte _adres;
	if (decoderadres <= BoardAddr) {
		_decoder = BoardAddr - decoderadres;
		if (_decoder < 8) { //max 8x4=32 dcc adressen
			_adres = (_decoder * 4) + OutputPair;  //merk op adressen hier van 0~31

			//Maak een uniek nummer van dit command, voorkom dat de command meerdere malen wordt uitgevoerd
			byte _unieknummer = (_decoder << 4) | (OutputPair << 2) | (Direction << 1) | OutputPower;
			//Serial.print("_uniek nummer= "); Serial.println(_unieknummer);	
			if (_unieknummer == dcclaatstecommand[_adres]) {
				//vorige command op dit adres  was hetzelfde command
				if (dcctimer[_adres] > 0)return; //dit command is dubbel
			}
			dcctimer[_adres] = 10; //stappen van 20ms,blokkeert zelfde command voor 200ms
			dcclaatstecommand[_adres] = _unieknummer;
			dcckanaal = _adres + 1;
			//dccport = false; if (Direction == 0)dccport = true; //hier is de rechtdoor of afslaand om te keren
			dccport = Direction;
			dcconoff = OutputPower;
			Dcc_exe();
		}
	}
}
void DccTimers() {
	for (byte i = 0; i < 32; i++) {
		if (dcctimer[i] > 0)dcctimer[i]--;
	}
}
void Dcc_exe() {
	Serial.print("Dcckanaal=   "); Serial.print(dcckanaal); Serial.print("  Dcc port= "); Serial.print(dccport); Serial.print("  aan of uit :"); Serial.println(dcconoff);
	switch (dcctype) {
	case 0: //dcc uit herstart is nu nodig
		return;
		break;
	case 1:  //type 4 4adressen alleen de 4 knoppen
		DccType4();
		break;
	case 2: //type 8 alle acties stepper, servooos uitgangen
		DccType8();
		break;
	case 3://type 16 als 8 en met 8 timers
		DccType16();
		break;
	case 4: //type 32 8x stepper, 2x8xservoos 6xoutputs ex sensor 8x timers
		DccType32();
		break;
	case 5: // type 56 als 32 maar stepper en servo standen als single  standen 
		break;
	}
}
void DccType4() {
	//port en onoff voorlopig even niet, omschakelen aanuit zetten overal reageert de module nu op
	switch (dcckanaal) {
	case 1:
		SWon(0);
		break;
	case 2:
		SWon(1);
		break;
	case 3:
		SWon(2);
		break;
	case 4:
		SWon(3);
		break;
	default:
		return;
		break;
	}
}
void DccType8() {
	switch (dcckanaal) {
	case 1: //stepper
		StepperActie();
		DisplayKort(1);
		break;
	case 2: //servo 1
		ServoActie(0);
		DisplayKort(2);
		break;
	case 3: //servo 2
		ServoActie(1);
		DisplayKort(3);
		break;
	case 4: //outAlarm
		OffAlarm;
		if (aanuit())OnAlarm;
		break;
	case 5: //out 1
		Off1;
		if (aanuit())On1;
		break;
	case 6: //out 2
		Off2;
		if (aanuit()) On2;
		break;
	case 7: //out 3
		Off3;
		if (aanuit())On3;
		break;
	case 8: //out 4
		Off4;
		if (aanuit())On4;
		break;
	default:
		return;
		break;
	}
}
void DccType16() {
	if (dcckanaal < 9) {
		DccType8();
	}
	else {
		DccTimers(9);
		//		if (dccport) { //timer starten
		//			TimerSwitch(0, dcckanaal - 9); //timers lopen van 0~7
		//			//controleren of echt aan gegaan, timers hebben heel veel toggle dingen onderweg, hier timer hard aan zetten.
		//			if (!(Timer & (1 << dcckanaal - 9))) {
		//				TimerSwitch(0, dcckanaal - 9); //timers lopen van 0~7				
		//			}
		//
		//		}
		//		else { //timer stoppen
		//			TimerStop(dcckanaal - 9, true);
		//			
		//		}
		//DisplayKort(dcckanaal+2); //dus waitnext optie 2~10
	}
}
void DccTimers(byte _offset) {
	if (dccport) { //timer starten
		TimerSwitch(0, dcckanaal - _offset); //timers lopen van 0~7
		//controleren of echt aan gegaan, timers hebben heel veel toggle dingen onderweg, hier timer hard aan zetten.
		if (!(Timer & (1 << dcckanaal - _offset))) {
			TimerSwitch(0, dcckanaal - _offset); //timers lopen van 0~7				
		}

	}
	else { //timer stoppen
		TimerStop(dcckanaal - _offset, true);

	}
	DisplayKort((dcckanaal - _offset) + 10); //dus waitnext optie 10~17
}
void DccType32() {
	byte _stand = 0;
	if (dcckanaal < 5) {
		DccType4();
	}
	else if (dcckanaal < 9) { //stepper
		_stand = ((dcckanaal - 5) * 2) + 1;
		if (dccport)_stand++; //1~8
		//Serial.println(_stand);
		StepDcc(_stand);
	}
	else if (dcckanaal < 13) { //servo1
		//doe iets met s1
		_stand = ((dcckanaal - 9) * 2) + 1;
		if (dccport)_stand++;
		ServoDcc(0, _stand);
		//Serial.println(_stand);
	}
	else if (dcckanaal < 17) {
		_stand = ((dcckanaal - 13) * 2) + 1;
		if (dccport)_stand++;
		ServoDcc(1, _stand);
		//Serial.println(_stand);

	}
	else {
		switch (dcckanaal) {
		case 17:
			Off1; if (aanuit())On1;
			break;
		case 18:
			Off2; if (aanuit())On2;
			break;
		case 19:
			Off3; if (aanuit())On3;
			break;
		case 20:
			Off4; if (aanuit())On4;
			break;
		case 21:
			OffBezet; if (aanuit())OnBezet;
			break;
		case 22:
			OffAlarm; if (aanuit())OnAlarm;
			break;
		case 23: //home sensor
			SensorActie(0, aanuit());
			break;
		case 24: //sensor
			SensorActie(1, aanuit());
			break;
		default:
			if (dcckanaal > 32)return;
			DccTimers(25);
			//timers
			break;
		}
	}
}

bool aanuit() {
	bool _aanuit = false;
	if (dccport)_aanuit = true;
	if (!dcconoff)_aanuit = false;
	//Serial.println(_aanuit);
	return _aanuit;
}

void DisplayKort(byte _actie) {

	//Serial.println(_actie);

	//toont even kort een door timer of dcc aangepaste stand van een output/actie
	DisplayClear();
	waitnext = 2; //displayshow
	waittime = 100;	 //timer zet het display weer terug
	switch (_actie) {
	case 1: //stepper
		digit[1] = Cijfer(5);
		digit[2] = Cijfer(stepstop);
		dots;
		break;
	case 2: //servo 1
		digit[1] = Cijfer(1);
		digit[2] = Cijfer(servostopdisplay[0]);
		dots;
		break;
	case 3:
		digit[1] = Cijfer(2);
		digit[2] = Cijfer(servostopdisplay[1]);
		dots;
		break;
	default:
		if (_actie > 9) {
			digit[0] = Letter('t');
			digit[1] = Cijfer(_actie - 9);
			if (Timer & (1 << (_actie - 10))) {
				digit[3] = Cijfer(1);
			}
			else {
				digit[3] = Cijfer(0);
			}
			dots;
		}
		break;
	}

}
//display
void Display_exe() {
	displaycount++;
	if (displaycount > 10) { //clockcycles was 500
		displaycount = 0;
		digitcount++;
		if (digitcount > 3)digitcount = 0;
		shiftbyte[1] &= ~(15 << 0); //clear bits 0~3
		shiftbyte[1] |= (1 << digitcount);
		shiftbyte[0] = digit[digitcount];

		PWM_exe(); //experimenteel 24jan
		Shift();

		//hier stond de shift() voor 24jan


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
void DisplayShow(byte _caller) {
	//caller is voor debug, om te weten wie de functie called, kan weg als alles goed werkt(laat maar staan erg handig)
	//Serial.print("displayshow   "); Serial.println(_caller);
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
		DisplayOutput(4); //bezet
		break;
	case 9:
		DisplayOutput(5); //alarm
		break;
	case 10:
		DisplayHome();
		break;
	case 11:
		DisplaySensor();
		break;
	default:
		//Hier een timer
		DisplayTimer(actie - 12);
		break;
	}
}
void DisplayGetal(byte _nummer, byte _digit) {
	byte _teken = 10;
	if (_nummer > 0)_teken = _nummer;
	digit[_digit] = Cijfer(_teken);
}
void DisplayClear() {
	for (byte i = 0; i < 4; i++) {
		digit[i] = 0xFF;
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
	case 'H':
		_result = B10001001;
		break;
	case 'o': //kleine letter 0
		_result = B10100011;
		break;
	case 'p':
		_result = B10001100;
		break;
	case 'u':
		_result = B11100011;
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
void DisplayAlias(byte _alias, bool _actie) { //vervang digit 2 e 3 voor een alias van een nummer
	switch (_alias) { //alle aanduidingen voor de acties 0~50
	case 0: //common
		if (_actie) {
			digit[2] = Cijfer(10);
			digit[3] = Letter('c');
		}
		else {
			digit[2] = Letter('-');
			digit[3] = Letter('-');
		}
		break;
	case 1: //stepper
		digit[2] = Cijfer(10);
		digit[3] = Cijfer(5); //S van stepper
		break;
	case 2: //servo 1
		digit[2] = Cijfer(10);
		digit[3] = Cijfer(1); //Servo 1
		break;
	case 3: //servo 2
		digit[2] = Cijfer(10);
		digit[3] = Cijfer(2); //Servo 2
		break;
	case 4:
		digit[2] = Letter('o');
		digit[3] = Cijfer(1); //output1   
		break;
	case 5:
		digit[2] = Letter('o');
		digit[3] = Cijfer(2); //output2
		break;
	case 6:
		digit[2] = Letter('o');
		digit[3] = Cijfer(3); //output3
		break;
	case 7:
		digit[2] = Letter('o');
		digit[3] = Cijfer(4); //output4
		break;
	case 8:
		digit[2] = Letter('o');
		digit[3] = Letter('b'); //output bezet
		break;
	case 9:
		digit[2] = Letter('o');
		digit[3] = Letter('A'); //output Alarm
		break;
	case 10:
		digit[2] = Letter('H');
		digit[3] = Letter('o');
		break;
	case 11:
		digit[2] = Cijfer(5);
		digit[3] = Letter('E');
		break;
	case 20: //sensor functie 0 (aan en uit)
		digit[2] = Letter('A');
		digit[3] = Letter('U');
		break;
	case 21: //sensorfunctie 1 (aan)
		digit[2] = Letter('A');
		digit[3] = Cijfer(10);
		break;
	case 22: //sensorfunctie 2 (uit)
		digit[2] = Cijfer(10);
		digit[3] = Letter('U');
		break;
	case 30: //timerscale 001
		digit[1] = Cijfer(0);
		digit[2] = Cijfer(0);
		digit[3] = Cijfer(1);
		break;
	case 31: //timerscale 01
		digit[1] = Cijfer(10);
		digit[2] = Cijfer(0);
		digit[3] = Cijfer(1);
		break;
	case 32: //timerscale 1
		digit[1] = Cijfer(10);
		digit[2] = Cijfer(10);
		digit[3] = Cijfer(1);
		break;
	case 100: //dcc type 0
		digit[2] = Cijfer(10);
		digit[3] = Letter('-');
		break;
	case 101: //dcc type 1 4adressen de knoppen
		digit[2] = Cijfer(10);
		digit[3] = Cijfer(4);
		break;
	case 102: //dcc type 8 (default) alle outputs
		digit[2] = Cijfer(10);
		digit[3] = Cijfer(8);
		break;
	case 103: //dcc type 16 defatult plus de timers
		digit[2] = Cijfer(1);
		digit[3] = Cijfer(6);
		break;
	case 104: //dcc type 28 alles
		digit[2] = Cijfer(3);
		digit[3] = Cijfer(2);
		break;
	default:
		if (_alias - 12 >= 0) {
			digit[2] = Letter('t');
			digit[3] = Cijfer(_alias - 11);
		}
		break;
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


			//DIT IS fout  scrollspeed blijft oplopen...

			if ((_read & (1 << i)) == false) { //knop ingedrukt

				if (scrollcount[i] < 60) {
					scrollcount[i]++;
					if (scrollcount[i] == 59) 	scrollspeed++;

				}
				if (scrollcount[i] > 20) SWon(i);
			}
		}
	}
	readlast = _read;
}
void SWon(byte _sw) {
	//Serial.print("on  "); Serial.println(_sw);
	switch (_sw) {
		//*****************************************SWITCH1****Actie knop*******
	case 0: //switch 1
		switch (actie) {
		case 0: //common actie ingesteld met start actie uitvoeren
			autostart();
			break;
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
		case 8:
			OutputActie(4); //actie output bezet
			break;
		case 9: //actie output Alarm
			OutputActie(5);
			break;
		case 10:
			SensorActie(0, true); //0=home sensor 1=sensor
			break;
		case 11:
			SensorActie(1, true);
			break;
			//hier eventueel timers 3 tot heel veel
		default:
			TimerSwitch(_sw, actie - 12);
			break;
		}

		break;

		//*********************************SWITCH 2**********
	case 1: //switch 2
		switch (actie) {
		case 0: //actie: common
			switch (programfase) {
			case 0:
				Actie_exe(false);
				break;
			case 1: //instellen actie op de actieknop
				Prg_comactieknop(false);
				break;
			case 2: //instellen actie na powerup(start)
				Prg_autostart(false);
				break;
			case 3:  //instellen DCC (decoder)adres
				Prg_comdccadres(false);
				break;
			case 4: //instellen dcc type (modus, maar ik kan geen m maken op een 7-segment)
				Prg_comdcctype(false);
				break;
				//5=factory
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
			case 7: //output keuze voor bezetstelling
				Prg_stepbezet(false);
				break;
			case 8:
				Prg_StepTimer(0, false);
				break;
			case 9:
				Prg_StepTimer(1, false);

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
			case 4: //dec servo 1 bezet melder naar output
				Prg_servobezetoutput(0, false);
				break;
			case 5:
				Prg_Servotimeroutput(0, 0, false); //=servo 1; starttimer; verlagen
				break;
			case 6:
				Prg_Servotimeroutput(0, 1, false); //=servo 1; einde timer; verlagen
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
			case 4: //dec servo 2 bezet output keuze
				Prg_servobezetoutput(1, false);
				break;
			case 5:
				Prg_Servotimeroutput(1, 0, false); //=servo 2; start timer, verlagen
				break;
			case 6:
				Prg_Servotimeroutput(1, 1, false); //=servo 2; einde timer, verlagen
				break;
			}
			break;
		case 4: //Actie output 1
			OutputSwitch(0, false);
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
		case 8: //Actie output Bezet
			OutputSwitch(4, false);
			break;
		case 9: //actie output Alarm
			OutputSwitch(5, false);
			break;
		case 10: //actie homesensor
			switch (programfase) {
			case 0:
				Actie_exe(false);
				break;
			case 1:
				Prg_Homeoutput(false);
				break;
			case 2:
				Prg_sensorfunctie(0, false);
				break;
			}
			break;
		case 11: //actie sensor
			switch (programfase) {
			case 0:
				Actie_exe(false);
				break;
			case 1:
				Prg_sensoroutput(false);
				break;
			case 2: //sensorfunctie
				Prg_sensorfunctie(1, false);
				break;

			}
			break;

		default:
			TimerSwitch(_sw, actie - 12);
			break;

			//meer timers kunnen hieronder
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
			case 1: //instellen actie op de actieknop
				Prg_comactieknop(true);
				break;
			case 2: //instellen actie na power-up
				Prg_autostart(true);
				break;
			case 3: //DCCadres
				Prg_comdccadres(true);
				break;
			case 4: //instellen DCC type
				Prg_comdcctype(true);
				break;
			case 5: //factory reset
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
			case 7: //output voor de bezet melding van de stepper
				Prg_stepbezet(true);
				break;
			case 8:
				Prg_StepTimer(0, true);
				break;
			case 9:
				Prg_StepTimer(1, true);
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
			case 4: //inc servo 1 keuze bezetmelding output
				Prg_servobezetoutput(0, true);
				break;
			case 5:
				Prg_Servotimeroutput(0, 0, true);//=servo 1; starttimer; verhogen
				break;
			case 6:
				Prg_Servotimeroutput(0, 1, true); //=servo 1; einde timer; verhogen
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
			case 4: //inc keuze bezet output servo 2
				Prg_servobezetoutput(1, true);
				break;
			case 5:
				Prg_Servotimeroutput(1, 0, true); //=servo 1; start timer, verhogen
				break;
			case 6:
				Prg_Servotimeroutput(1, 1, true); //=servo 2; einde timer. verhogen
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
		case 8: //output bezet, actie
			OutputSwitch(4, true);
			break;
		case 9: //output alarm actie
			OutputSwitch(5, true);
			break;
		case 10: //extra output home sensor
			switch (programfase) {
			case 0:
				Actie_exe(true);
				break;
			case 1:
				Prg_Homeoutput(true);
				break;
			case 2:
				Prg_sensorfunctie(0, true);
				break;
			}
			break;
		case 11: //output sensor
			switch (programfase) {
			case 0:
				Actie_exe(true);
				break;
			case 1:
				Prg_sensoroutput(true);
				break;
			case 2:
				Prg_sensorfunctie(1, true);
			}

			break;
		default:
			TimerSwitch(_sw, actie - 12);
			break;
			//meer timers kunnen hier
		}
		break;
		//***********************************
	case 3: //switch 4 program switch
		Prg_up();
		break;
		//**************************************SENSOREN******************


	case 4: //Sensor uit  (uit want hoogactief)
		if (sensorfunctie[1] != 1)  SensorActie(1, false); //1=aan dan wordt actie niet uitgezet alleen aangezet dus actief worden van de sensor
		break;
		//***********************************
	case 5: //hall sensor home, uit
		Step_sensor(false); //deze actie is vast ingesteld, home switch voor de stepper

		//subfunctie van de home sensor, beide gebruiken kan problemen geven met de stepper(hoeft niet)
		if (sensorfunctie[0] != 1)	SensorActie(0, false); //als bij sensor
		break;
	}
}
void SWoff(byte _sw) {
	scrollcount[_sw] = 0; //reset teller voor het scrollen
	scrollspeed = 0;
	switch (_sw) {
	case 0:  //knop 1 losgelaten
		switch (actie) {
		case 10: //Xtra home sensor
			SensorActie(0, false);
			break;
		case 11: //sensor
			SensorActie(1, false);
			break;
		}
		break;
	case 4: //sensor knop ingedrukt
		if (sensorfunctie[1] == 2) { //2=alleen uit sensor actief zet actie uit
			SensorActie(1, false);
		}
		else {
			SensorActie(1, true);
		}
		break;

	case 5: //hall sensor home
		Step_sensor(true); //home switch stepper
		//extra sub functie van de sensor

		if (sensorfunctie[0] == 2) {
			SensorActie(0, false);
		}
		else {
			SensorActie(0, true);
		}
		break;
	}
}

//common instellingen
void ProgramCom() { //afhandelen program fase in common
	//called from Prg-up (verhoogt de programfase)
	if (programfase > 5)programfase = 0; //aantal programfases per program reeks verschillend
	scrollmask = 0;
	switch (programfase) {
	case 0:
		Prg_end();
		break;
		//1 Keuze actie voor de actieknop
		//2keuze actie voor de autostart, wordt gedaan direct na powerup		
	case 3:  //instellen DCC adres
		scrollmask = B0110;
		dccprg = true;
		break;
		//4 keuze van dcc type, herstart hierna nodig
	case 5: //factory
		confirm = false;
		break;
	}
	//terug naar prg_up waarin daarna Displayshow>Displaycom
}
void DisplayCom() { //algemene instellingen knop1 heeft (nog) geen functie
	DisplayClear();
	digit[0] = Letter('c');
	switch (programfase) {
		//***********************
	case 0: //in bedrijf
		digit[1] = Cijfer(10);
		digit[2] = Cijfer(10);
		digit[3] = Cijfer(10);
		break;
		//**********************

	case 1://instellen actie op de actieknop(1)
		digit[0] = Letter('A');
		digit[1] = Letter('c');
		DisplayAlias(actieknopkeuze, true); //true bij actie keuze, false timeroutput keuze
		dots;
		break;
	case 2: //AS =actie na start, powerup
		digit[0] = Letter('A');
		digit[1] = Cijfer(5);
		DisplayAlias(actienastart, false);
		dots;
		break;

	case 3: //instellen DCC decoder adres
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
	case 4: //DCC modes instellen
		DisplayClear();
		digit[0] = Letter('d');
		digit[1] = Letter('t');
		DisplayAlias(dcctype + 100, false);
		dots;
		break;
	case 5: //factory
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

	DisplayShow(4);
	waittime = 200;
	waitnext = 2; //displayshow()
	dccprg = true;
}
void Prg_comactieknop(bool _plusmin) {
	if (_plusmin) {
		actieknopkeuze++;
		if (actieknopkeuze > AantalActies)actieknopkeuze = 0;
	}
	else {
		actieknopkeuze--;
		if (actieknopkeuze == 0xFF)actieknopkeuze = AantalActies;
	}
	DisplayShow(5);
}
void Prg_autostart(bool _plusmin) {
	if (_plusmin) {
		if (actienastart < AantalActies)actienastart++;
		if (actienastart == 10)actienastart = 12; //10(home) en 11(sensor) overslaan
	}
	else {
		if (actienastart > 0)actienastart--;
		if (actienastart == 11)actienastart = 9;
	}
	DisplayShow(40);
}
void Prg_comdcctype(bool _plusmin) {
	if (_plusmin) {
		if (dcctype < 4)dcctype++;
	}
	else {
		if (dcctype > 0)dcctype--;
	}
	DisplayShow(40);
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
void DisplayStepper() {
	//toont texten aan de hand van de programmeer stand en laatste positie van de draaielementen
	digit[0] = Cijfer(5);

	switch (programfase) {
	case 0: //in bedrijf
		//digit[0] = Cijfer(5);
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
		//digit[0] = Cijfer(5);
		if (stepstop > 0) {
			digit[1] = Letter('_'); digit[2] = Letter('+');
			DisplayGetal(stepstop, 3);
		}
		else {
			//digit[0] = Letter('b');
			digit[1] = Letter('-'); digit[2] = Letter('-'); digit[2] = Letter('-');
		}
		break;
	case 2: //aantal stops van de stepper
		//digit[0] = Letter('b'); //b(yj48-28)c(ount)
		digit[1] = Letter('c');
		DisplayNummer(stepaantalstops, 23);
		//digit[2] = displaycijfers[2];
		//digit[3] = displaycijfers[3];
		break;
	case 3: //min speed stepper
		//digit[0] = Letter('b');
		digit[1] = Letter('_'); dots;
		DisplayNummer(speedminfactor, 23);//berekend de digits uit een nummer max 9999
		//digit[2] = displaycijfers[2];
		//digit[3] = displaycijfers[3];
		break;
	case 4: //max speed stepper
		//digit[0] = Letter('b');
		digit[1] = Letter('|'); dots;//streepje boven
		DisplayNummer(speedmaxfactor, 23); //berekend de digits uit een nummer max 9999
		//digit[2] = displaycijfers[2];
		//digit[3] = displaycijfers[3];
		break;
	case 5: //afremfactor *10=accelspeed.
		//digit[0] = Letter('b');
		digit[1] = Letter('-');  dots;
		DisplayNummer(afremfactor, 23); //berekend de digits uit een nummer max 9999
		//digit[2] = displaycijfers[2];
		//digit[3] = displaycijfers[3];
		break;
	case 6: //default richting stepper naar home
		//digit[0] = Letter('b');
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
	case 7: //output van de bezet stelling
		//digit[0] = Letter('b');
		digit[1] = Letter('b');
		DisplayOutputkeuze(stepbezetoutput);
		break;
	case 8: //timer bij start instellen
		DisplayClear();
		digit[0] = Letter('t');
		digit[1] = Cijfer(5);
		if (stepoutputtimer[0] > 0) {
			digit[3] = Cijfer(stepoutputtimer[0]);
		}
		else {
			line23;
		}
		dots;
		break;
	case 9: //timer afsluitend instellen
		DisplayClear();
		digit[0] = Letter('t');
		digit[1] = Letter('E');
		if (stepoutputtimer[1] > 0) {
			digit[3] = Cijfer(stepoutputtimer[1]);
		}
		else {
			line23;
		}
		dots;
		break;
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
	if (stepperisbezet)return;
	stepstop++;
	if (stepstop > stepaantalstops)stepstop = 1;
	StepStart();
	DisplayShow(6);
}
void StepStart() {
	if (stepoutputtimer[0] > 0) {  //timer met voorgaande actie instarten
		TimerSwitch(0, stepoutputtimer[0] - 1);  //timer 1=0;
	}
	Output_exe(stepbezetoutput, 0, true); //bezet stellen, 0=caller stepper, stepbezetoutpu=welke output
	standdoel = stepper[stepstop - 1];  //stand in stappen voor de motor in stellen
	stepmovefase = 10; //begin draaiproces
	stepwaittime = 100; //timer voor wachttijd waarin requested positie nog kan worden gewisseld
}
void StepDcc(byte _stop) {
	if (_stop > stepaantalstops || stepperisbezet) return;
	stepstop = _stop;
	StepStart();
	DisplayKort(1);
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

	case 20: //doel bereikt, hier gaat de stepper stoppen
		stepdrive = false;
		// Stepoff(); verplaatst naar een algemene wacht lus na iedere puls
		Output_exe(stepbezetoutput, 0, false); //geef de schijf vrij, locs kunnen gaan rijden  0=caller stepper
		stepperisbezet = false;

		if (stepoutputtimer[1] > 0) {
			TimerSwitch(0, stepoutputtimer[1] - 1); //stepoutputtimer loopt van 1~8 timers van 0~7
		}
		DisplayShow(7);
		break;

	}
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
	DisplayShow(8);
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
	DisplayShow(9);
}
void Prg_stephome(bool _richting) {
	if (_richting) {
		stephomerichting = true;
	}
	else {
		stephomerichting = false;
	}
	DisplayShow(10);
}
void Prg_stepbezet(bool _plusmin) {
	if (_plusmin) {
		if (stepbezetoutput < AantalOutputOpties)stepbezetoutput++;
	}
	else {
		if (stepbezetoutput > 0)stepbezetoutput--;
	}
	DisplayShow(11);
}
void ProgramStep() { //afhandelen programreeks voor stepper
	//called from prg_up
	if (programfase > 9)programfase = 0; //aantal programfases per program reeks verschillend
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
void Prg_StepTimer(byte _tStE, bool _plusmin) {
	if (_plusmin) {
		if (stepoutputtimer[_tStE] < 8)stepoutputtimer[_tStE]++;
	}
	else {
		if (stepoutputtimer[_tStE] > 0)stepoutputtimer[_tStE]--;
	}
	DisplayShow(27);
}


//program acties
void Actie_exe(bool _plusmin) {
	if (_plusmin) {
		actie++;
	}
	else
	{
		actie--;
	}
	if (actie == 0xFF)actie = AantalActies;
	if (actie > AantalActies)actie = 0;
	DisplayShow(12);
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
	case 8:
		ProgramOutput(4);
		break;
	case 9:
		ProgramOutput(5);
		break;
		//8=output bezet; 9=output alarm 10=home sensor; 11=sensor
	case 10:
		ProgramSensors(0);
		break;
	case 11:
		ProgramSensors(1);
		break;

	default: //dit om niet 8 cases voor de timers te hoeven maken
		if (actie - 12 >= 0) {
			TimerSwitch(3, actie - 12);
		}
		break;
	}
	DisplayShow(13);
}
void Prg_end() { //einde programmamode
	programfase = 0;
	scrollmask = 0;
	confirm = true; //dit zorgt ervoor dat de actie knop na een programmeer sessie niet veranderd. 
	Eeprom_write();
	confirm = false; //reset de flag weer, wordt ook gebruikt bij factory reset
}


//servo's
void Servo_exe() {
	//called from loop om de 4ms
	unsigned int _afstand = 0;
	servo++; if (servo > 1)servo = 0;

	//uitstappen als er niks hoeft te worden gedaan
	if (servostop[servo] == 0)return;
	if (servostopcount[servo] > 100)return;

	//huidige positie en daaruit voor komende taak bepalen en uitvoeren
	if (servocurrent[servo] == servotarget[servo]) { //servo staat op nieuwe positie
		//servocurrent[servo] = servotarget[servo];  //eventuele verschillen weghalen


		servostopcount[servo]++;
		if (servostopcount[servo] > 100) {   //== 100) {
			Output_exe(servobezetoutput[servo], (servo + 1), false); //reset bezet servo (callers zijn 1 en 2)

			//******************einde timer aanroepen
			//Serial.println("timereinde");
			if (servotimeroutput[servo][1] > 0) {
				TimerSwitch(0, servotimeroutput[servo][1] - 1);

				if (!(Timer & (1 << servotimeroutput[servo][1] - 1))) {
					//Serial.println("timer is uit");
					//timer is niet aan, komt door dubbele instart, of servo omzetten terwijl servo weer draait, of timer stond nog aan
					//om en om aan en uit zetten is niet de bedoeling, alleen aan zetten.
					TimerSwitch(0, servotimeroutput[servo][1] - 1); //nogmaals timer starten
				}
			}
			//*************************************** einde timer aanroepen
		}
	}
	else { //servo staat nog niet op positie

		if (servostep[servo] < 1)servostep[servo] = 1;
		_afstand = abs(servocurrent[servo] - servotarget[servo]);

		if (servostep[servo] * servostep[servo] > _afstand) {
			if (servostep[servo] > 1) servostep[servo]--;
		}
		else {
			if (servostep[servo] < servospeed[servo]) servostep[servo]++; //versnellen van de beweging naar max speedelse {
		}


		if (servocurrent[servo] < servotarget[servo]) {
			servocurrent[servo] += servostep[servo];
		}
		else {
			servocurrent[servo] -= servostep[servo];
		}
	}
	//timer instellen starten
	stepdrivelaatst = stepdrive;
	stepdrive = false; //stepper onderbreken
	OCR1A = servocurrent[servo];
	PORTB |= (1 << (4 + servo));
	TCCR1B = 10; //start de timer1
}

void ServoActie(byte _servo) {
	servostopdisplay[_servo]++;
	if (servostopdisplay[_servo] > servoaantalstops[_servo])servostopdisplay[_servo] = 1;
	ServoStart(_servo);
	DisplayShow(14);
}
void ServoStart(byte _servo) {

	Output_exe(servobezetoutput[_servo], (_servo + 1), true); //1=caller1, servo1 2=caller 2 servo 2 (_servo+1)
	if (servotimeroutput[_servo][0] > 0) { //is er een starttimer ingesteld 
		TimerSwitch(0, servotimeroutput[_servo][0] - 1);  //timer 1=0; //schakel de timer in 
	}
	servowaittime[_servo] = 20; //
}
void ServoDcc(byte _servo, byte _stand) {
	servostopdisplay[_servo] = _stand;
	ServoStart(_servo);

	DisplayKort(2 + _servo); //2 of 3 dus
}
void Servo1_move() {
	servostop[0] = servostopdisplay[0]; //servostopdisplay ingesteld in servoactie.
	servotarget[0] = servo1pos[servostop[0] - 1];  //veranderd target en start de servo
	servostopcount[0] = 0;
	//DisplayShow(32);
}
void Servo2_move() {
	servostop[1] = servostopdisplay[1];
	servotarget[1] = servo2pos[servostop[1] - 1];
	servostopcount[1] = 0;
	//DisplayShow(33);
}


void DisplayServo(byte _servo) {

	//if (_servo == 0)digit[0] = Letter('x'); else digit[0] = Letter('q'); //x/q=teken voor servo 1 of 2 
	digit[0] = Cijfer(_servo + 1);

	switch (programfase) {
	case 0: //in bedrijf

		digit[1] = Cijfer(10);
		digit[2] = Letter('p');
		if (servostopdisplay[_servo] == 0)digit[3] = Letter('-'); else digit[3] = Cijfer(servostopdisplay[_servo]);
		break;

	case 1: //posities instellen
		//if (_servo == 0)digit[0] = Letter('x'); else digit[0] = Letter('q'); //q=teken voor servo 2
		if (servostop[_servo] == 0) {
			digit[1] = Letter('-'); digit[2] = Letter('-');
		}
		else {
			digit[1] = Letter('_');
			digit[2] = Letter('|'); //bovenstreepje

			if (servostopdisplay[_servo] == 0)digit[3] = Letter('-'); else digit[3] = Cijfer(servostopdisplay[_servo]);
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
		digit[1] = Letter('>'); //rechtopstaande =(is)teken
		DisplayNummer(servospeed[_servo], 23);
		dots;
		break;
	case 4: //bezet output keuze
		digit[1] = Letter('b');
		DisplayOutputkeuze(servobezetoutput[_servo]);
		break;
	case 5: //timer voor actie servo
		digit[0] = Letter('t');
		digit[1] = Cijfer(5);
		DisplayNummer(servotimeroutput[_servo][0], 23);
		dots;
		break;
	case 6: //timer na de actie van de servo
		digit[0] = Letter('t');
		digit[1] = Letter('E');
		DisplayNummer(servotimeroutput[_servo][1], 23);
		dots;
		break;
	}
}
void ProgramServo(byte _servo) { //called from prg_up
	//programfase inc gebeurt in prg_up
	if (programfase > 6)programfase = 0;
	scrollmask = 0;
	switch (programfase) {
	case 0:
		Prg_end();
		break;
	case 1: //instellen posities
		scrollmask = B0110;
		break;
		//3=aantal stops
	   //4=snelheid, speed
	   //5=output bij bezet	
	  //6=timer voor start servoactie
	   //7=timer na servo actie
	}
	//displayshow volgt nu in prg_up, dit verwijst door naar displayservo
}

void Prg_servopos(byte _servo, bool _plusmin) {
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
		Servo1_move(); //verzet de servo
	}
	else {
		if (_plusmin) {
			if (servo2pos[_servostop] < ServoMaxPositie)servo2pos[_servostop]++;
		}
		else {
			if (servo2pos[_servostop] > ServoMinPositie)servo2pos[_servostop]--;
		}
		DisplayNummer(servo2pos[_servostop], 4); //plaats nummer in het display
		Servo2_move(); //verzet de servo
	}
}
void Prg_servoaantalstops(byte _servo, bool _plusmin) {
	if (_plusmin) {
		if (servoaantalstops[_servo] < ServoMaxStops)servoaantalstops[_servo]++;
	}
	else {
		if (servoaantalstops[_servo] > 2)servoaantalstops[_servo]--;
	}
	DisplayShow(15);
}
void Prg_servobezetoutput(byte _servo, bool _plusmin) {
	if (_plusmin) {
		if (servobezetoutput[_servo] < AantalOutputOpties)servobezetoutput[_servo]++;
	}
	else {
		if (servobezetoutput[_servo] > 0)servobezetoutput[_servo]--;
	}
	DisplayShow(16);
}
void Prg_servospeed(byte _servo, bool _plusmin) {

	if (_plusmin) {
		if (servospeed[_servo] < ServoMaxSpeed)servospeed[_servo]++;
	}
	else {
		if (servospeed[_servo] > 1)servospeed[_servo]--;
	}
	DisplayShow(17);
}

void Prg_Servotimeroutput(byte _servo, byte _tse, bool _plusmin) { //tse=timer start of timer end
	if (_plusmin) {
		if (servotimeroutput[_servo][_tse] < 8) servotimeroutput[_servo][_tse]++;
	}
	else {
		if (servotimeroutput[_servo][_tse] > 0) servotimeroutput[_servo][_tse]--;
	}
	DisplayShow(31);
}
//program modes
//timers tbv de outputs? in stellen in common
void Output_exe(int _out, byte _caller, bool _onoff) { //0=stepper, 1=servo 1 2=servo2enz

	//kan een valdatie op de caller worden ingesteld? 19jan niet in gebruik
	//switch (_caller) {
	//case 0: 
	//break;
	//}
	//Serial.println(_out);

	switch (_out) {
	case 0:
		return;
		break;
	case 1:
		if (_onoff)PORTD |= (1 << 7); else PORTD &= ~(1 << 7);	break;
	case 2:
		if (_onoff)PORTD |= (1 << 3); else PORTD &= ~(1 << 3);	break;
	case 3:
		if (_onoff) { On1; }
		else Off1;	break;
	case 4:
		if (_onoff) { On2; }
		else Off2;	break;
	case 5:
		if (_onoff) { On3; }
		else Off3; break;
	case 6:
		if (_onoff) { On4; }
		else Off4; break;

	default:
		byte _timer = _out - 7;
		if (_onoff) {
			TimerSwitch(0, _timer);
		}
		else {
			TimerStop(_timer, true);
		}
		break;
	}
}
void OutputSwitch(byte _out, bool _plusmin) {
	//voorlopig hebben de outputs geen functies
	switch (programfase) {
	case 0:
		Actie_exe(_plusmin);
		break;
	case 1: //instellen pwm duty cycle
		Prg_OutputPwm(_out, _plusmin);
		break;
	case 2: //snelheid van pwm verandering
		Prg_OutputPwmspeed(_out, _plusmin);
		break;
	}
	DisplayShow(20);
}
void OutputActie(byte _out) {
	//aanzetten van de output
	if (_out < 4) {
		outputonoff ^= (1 << _out);

	}
	else {
		switch (_out) {
		case 4: //Bezet
			PIND |= (1 << 7);
			break;
		case 5: //alarm
			PIND |= (1 << 3);
			break;
		}
	}
	DisplayShow(18);
}
void OutputPwm() {
	//clock called from timer_exe periode 10ms
	//pwmdutycycle bepaald hoe hoog de led brand 0=niet 100 is voluit

	for (byte i = 0; i < 4; i++) { //check alle 4 outputs


		if (outputpwmspeed[i] > 0) {
			//tijd aftellen
			outputpwmspeedcount[i]++;
			if (outputpwmspeedcount[i] > outputpwmspeed[i]) {
				outputpwmspeedcount[i] = 0;

				//timer afgelopen, duty cycle aanpassen, 2 opties, output staat aan of uit
				//en nu wordt het pas echt ingewikkeld.....

				if (outputonoff & (1 << i)) {  //led moet sterker gaan branden
					if (pwmdutycycle[i] > (100 - pwmduty[i]))pwmdutycycle[i]--;
				}
				else { //led moet afzwakken naar 0
					if (pwmdutycycle[i] < 100)pwmdutycycle[i]++;
				}
			}
		}

		else {

			//geen timer voor inc en dec van de dutycycle ingesteld
			if (outputonoff & (1 << i)) {

				pwmdutycycle[i] = 100 - pwmduty[i]; //omgekeerd ingestelde waarde, bepaald hoe hoog de output staat. 0~100%
			}
			else {
				pwmdutycycle[i] = 100; //
			}
		}
	}
}
void ProgramOutput(byte _out) {
	switch (_out) {  //uitzondering geen instellinegen op de alarm en bezet output
	case 4:
		programfase = 0;
		return;
	case 5:
		programfase = 0;
		return;
	}

	if (programfase > 2)programfase = 0;
	scrollmask = 0;
	switch (programfase) {
	case 0:
		Prg_end();  //veranderimgem opslaan
		break;
	case 1:
		scrollmask = B0110;
		break;
	case 2:
		scrollmask = B0110;
		break;
	}
}
void DisplayOutput(byte _out) {
	//voorals nog hebben de outputs geen programmeerfuncties
	DisplayClear();
	switch (programfase) {
	case 0:  //in bedrijf
		digit[0] = Letter('o');
		if (_out < 4) {
			digit[1] = Cijfer(_out + 1);
		}
		else {
			switch (_out) {
			case 4:
				//Bezet output op PIN 7 (PORTD7)
				digit[1] = Letter('b');
				break;
			case 5:
				//Alarm output op PIN3 (PORTD3)
				digit[1] = Letter('A');
				break;
			}
		}
		//clear23; //zet de twee laatste digits uit
		break;
	case 1: //instellen duty cycle
		digit[0] = Letter('d');
		digit[1] = Letter('t');
		DisplayNummer(pwmduty[_out], 23);
		dots;
		break;
	case 2: //instellen dec en inc tijd, snelheid
		DisplayNummer(outputpwmspeed[_out], 4);
		break;
	}
	//	dots;
	//	DisplayNummer(outputfunctie[_out], 23);
}
void Prg_OutputPwm(byte _out, bool _plusmin) {
	byte _scrollsteps = ScrollSteps();
	if (_plusmin) {
		if (pwmduty[_out] < (101 - _scrollsteps))pwmduty[_out] += _scrollsteps;
	}
	else {
		if (pwmduty[_out] > _scrollsteps - 1)pwmduty[_out] -= _scrollsteps;
	}
}
byte ScrollSteps() {
	byte _result = 1;
	switch (scrollspeed) {
	case 1:
		_result = 10;
		break;
	}
	return _result;
}
void Prg_OutputPwmspeed(byte _out, bool _plusmin) {
	byte _scrollsteps = ScrollSteps();
	if (_plusmin) {
		if (outputpwmspeed[_out] < 10000 - _scrollsteps)outputpwmspeed[_out] += _scrollsteps;
	}
	else {
		if (outputpwmspeed[_out] > _scrollsteps - 1)outputpwmspeed[_out] -= _scrollsteps;
	}
}


void DisplayOutputkeuze(byte _output) {
	//toont gekozen output in digit 2 en digit 3

	digit[2] = Letter('o'); //komt 6x voor
	switch (_output) {
	case 0: //Geen bezet output
		DisplayNummer(0, 23);
		break;
	case 1:
		digit[3] = Letter('b');
		break;
	case 2:
		digit[3] = Letter('A');
		break;
	case 3:
		digit[3] = Cijfer(1);
		break;
	case 4:
		digit[3] = Cijfer(2);
		break;
	case 5:
		digit[3] = Cijfer(3);
		break;
	case 6:
		digit[3] = Cijfer(4);
		break;
		//case 7:
		//	digit[2] = Letter('t');
		//	digit[3] = Cijfer(1);
		//	break;
		//case 8:
		//	digit[2] = Letter('t');
		//	digit[3] = Cijfer(2);
		//	break;

	default: //voor de timers 1~8
		digit[2] = Letter('t');
		digit[3] = Cijfer(_output - 6);
		break;



	}
	dots;
}
void PWM_exe() {
	//	byte outputonoff; //bit0=output1 bit 4=outbezet bit 5 is out alarm
	//	byte pwmcount[8]; //duty cycle counter, de off tijd van het pwm signaal
	//	byte pwmduty[8]; //duty cycle van de ppwm voor de output EEPROM 80 instelling
	//	byte pwmdutycycle[8]; //dutycycle van een output om dit moment
	//	uint pwmspeed[8]; //snelheid van dec en inc van de dutycycle voor outputs  EEPROM 90 instelling

	OutputPwm();

	for (byte i = 0; i < 4; i++) { //alle outputs bekijken, voorlopig  nog niet bezet en alarm

		if (pwmdutycycle[i] == 100) { //direct output laag zetten
			shiftbyte[1] &= ~(1 << i + 4);
		}
		else 	if (pwmdutycycle[i] == 0) { //direct output hoog zetten
			shiftbyte[1] |= (1 << i + 4);
		}
		else if (shiftbyte[1] & (1 << i + 4)) { // output heeft een pwm puls
			//een hoge  dutycycle geeft een brede aan periode en een smalle uit periode
			//een lage duty cycle geef een smalle aanperiode en een brede uit periode
			//bij 50 is de breedte van aan en uit periodes gelijk.

			pwmoncount[i]++;  //aan periode aftellen
			if (pwmoncount[i] > (100 - pwmdutycycle[i])) { //breedte aan puls omgekeerd evenredig met de dutycycle
				pwmoncount[i] = 0;
				shiftbyte[1] &= ~(1 << (i + 4)); //output laag zetten 
			}
		}
		else { //output staat laag
			pwmcount[i]++; //uit periode aftellen
			if (pwmcount[i] > pwmdutycycle[i]) { //breedte uit periode evenredig aan duty
				pwmcount[i] = 0;
				shiftbyte[1] |= (1 << i + 4); //zet output weer aan
			}
		}
	}
}

//Timers
unsigned long verlopentijd; ///?????
byte count = 0;
void Timer_exe() {
	//called door ISR van timer 2 iedere 10ms, voorlopig straks tijdschaal instelbaar maken.


	for (byte i = 0; i < 8; i++) {  //8 timers
		if (Timer & (1 << i)) { //timer aan of uit, i=dus nu de gekozen timer

			//prescaler
			timerscalecounter[i]++;
			if (timerscalecounter[i] > timerprescaler[i]) {
				timerscalecounter[i] = 0; //reset de teller
			}
			else {
				break; //verlaat deze cycle
			}

			timercount[i]++;
			if (timeronoff[i]) { //on tijd aftellen

				if (timercount[i] > timerontijd[i]) { //timer afgelopen
					TimerNaOnTijd(i);
					timeronoff[i] = false;
					timercount[i] = 0;
				}
			}
			else { //off tijd tellen
				if (timercount[i] > timerofftijd[i]) {
					TimerNaOffTijd(i);
					timeronoff[i] = true;
					timercount[i] = 0;
				}
			}
		}
	}
}
void TimerNaOnTijd(byte _timer) {
	//timerontijd afgelopen, event call
	TimerActie(_timer, false);

}
void TimerNaOffTijd(byte _timer) {
	//Timerofftijd afgelopen event call
	timerfase[_timer]++;
	if (timerfase[_timer] >= timeraantaloutputs[_timer]) {
		//Alle fases doorlopen 
		if (timercycles[_timer] > 0) {
			timercyclecount[_timer]++;
			if (timercyclecount[_timer] >= timercycles[_timer]) {
				TimerStop(_timer, false);
				timercyclecount[_timer] = 0;
				return;
			}
		}
		timerfase[_timer] = 0; //terug naar eerste fase/ output
	}
	TimerActie(_timer, true);
}

void TimerStop(byte _timer, bool _timersstoppen) {

	//stopt een timer en zet outputs weer terug
	byte _output;

	Timer &= ~(1 << _timer);

	//Alle fases, outputs die deze timer op is ingesteld naat off, uit stand zetten.
	//iedere output, fase haaft daar eiegen reactieop
	for (byte i = 0; i < TimerAantalFases; i++) {
		if (timeroutput[_timer][i] > 0) {
			timerfase[_timer] = i;
			_output = timeroutput[_timer][i];

			//uitzondering voor timer die timers aansturen, als aantal cycles > 0 dan stopt de aangestuurde timer niet.			

			//if((_output < 12 )|| (_timersstoppen==true))	
			TimerActie(_timer, false);
		}
	}
}
void DisplayTimer(byte _timer) {
	//called from Displayshow() 
	DisplayClear();
	switch (programfase) {
	case 0:
		digit[0] = Letter('t');
		digit[1] = Cijfer(_timer + 1);
		break;
	case 1: //timer scaler 
		digit[0] = Letter('C');
		DisplayAlias(30 + timerscale[_timer], true);
		break;
	case 2: //on tijd instellen
		DisplayNummer(timerontijd[_timer], 4);
		break;
	case 3: //offtijd instellen
		DisplayNummer(timerofftijd[_timer], 4);
		break;
	case 4: //outputs(6) instellen, met knop 1 outputchannel te kiezen
		digit[0] = Letter('C');
		digit[1] = Cijfer(timerkeuzeoutput[_timer] + 1);  //tonen als 1tot6		
		DisplayAlias(timeroutput[_timer][timerkeuzeoutput[_timer]], false); //in outputkeuze niet de actie c(ommom) kiezen maar een streepje
		dots;
		break;
	case 5: //aantal cycles instellen
		digit[0] = Letter('c');
		digit[1] = Letter('A');
		DisplayNummer(timercycles[_timer], 23);
		dots;
		break;
	case 6:  //na activatie door een timer, start/stop of toggle aan/uit (geen stop)
		DisplayClear();
		if (timerstoppen & (1 << _timer)) {
			//timer starten en stoppen door andere timer
			digit[1] = Letter('U'); digit[2] = Letter('U'); dots;
		}
		else {
			//Stand timer (aan/uit) alleen laten wisselen door andere timer
			digit[1] = Letter('A'); digit[2] = Letter('U');  dots;
		}
		break;
	}
}
void TimerActie(byte _timer, bool _natijd) { //_stepper false=stepper en servo's niet stellen 
	//_natijd=false called na aflopen ontijd, true called na aflopen offtijd

	byte _output = 0;
	switch (timeroutput[_timer][timerfase[_timer]]) { //is de output waar de timer nu op staat
	case 0: //geen actie
		break;
	case 1: //stepper
		if (_natijd)StepperActie(); //alleen na de ontijd timeractie.
		break;
	case 2://servo 1
		if (_natijd)ServoActie(0);

		break;
	case 3: //servo 2
		if (_natijd)ServoActie(1);
		break;
	case 4: //out 1
		if (_natijd) { On1; }
		else { Off1; }
		break;
	case 5: //out2
		if (_natijd) { On2; }
		else { Off2; }
		break;
	case 6: //out 3
		if (_natijd) { On3; }
		else { Off3; }
		break;
	case 7:  //out 4
		if (_natijd) { On4; }
		else { Off4; }
		break;
	case 8: //out bezet
		if (_natijd) { OnBezet; }
		else { OffBezet; }
		break;
	case 9: //out alarm
		if (_natijd) { OnAlarm; }
		else { OffAlarm; }
		break;
		//case 10: //timer 1
		//	TimerSwitch(0, 0);
		//	break;
		//case 11: //timer 2
		//	TimerSwitch(0, 1);
		//	break;
	default:
		_output = timeroutput[_timer][timerfase[_timer]] - 12;
		if (_natijd) {
			TimerSwitch(0, _output);
		}
		else {

			if (timerstoppen & (1 << _output))  TimerStop(_output, true);
		}
		break;
	}
}
void TimerSwitch(byte _sw, byte _timer) {
	//Programfase is al verhoogd in voorgaande Prgup. Prgup wordt gecalled bij druk op knop4 en verwijst naar deze void.
	if (programfase > 6)programfase = 0;
	byte _scrollsteps = 0;
	switch (scrollspeed) {
	case 0:
		_scrollsteps = 1;
		break;
	case 1:
		_scrollsteps = 10;
		break;
	default:
		//Serial.println(scrollspeed);
		break;
	}

	switch (_sw) {
	case 0: //**************switch 1		
		switch (programfase) {
		case 0:
			Timer ^= (1 << _timer); //zet timer om aan/uit start timer
			//Timer |= (1 << _timer); //zet de timer aan
			//init
			if (!(Timer & (1 << _timer))) {
				TimerStop(_timer, true);
				return;
			}
			timeronoff[_timer] = true; //starten in de on time 
			timercyclecount[_timer] = 0; //reset teller voor het aantal cycli
			timerfase[_timer] = 0; //instellen op eerste actie in de acties voor deze timer
			timercount[_timer] = 0; //reset de teller
			timerscalecounter[_timer] = 0;
			switch (timerscale[_timer]) {
			case 0:
				timerprescaler[_timer] = 0; //count=10ms (default)
				break;
			case 1:
				timerprescaler[_timer] = 10; //count=100ms
				break;
			case 2:
				timerprescaler[_timer] = 100; //1 count = 1 seconde
				break;
			}

			//aantal ingestelde outputs bepalen
			timeraantaloutputs[_timer] = 0;
			for (byte i = 0; i < 6; i++) { //hier nog max 6 outputs van de timer bekijken
				if (timeroutput[_timer][i] > 0) {
					timeraantaloutputs[_timer]++;
					TimerActie(_timer, false); //zet de i  output uit,
					//onderstaande snap ik niet meer, i is hier toch al de timerfase
					//if (timerfase[_timer] < TimerAantalFases) timerfase[_timer]++; //volgende output instellen, laatste niet anders gaattie uit de array grootte
				}
			}

			timerfase[_timer] = 0;//fase van de timer weer op de eerste output zetten			
			if (Timer & (1 << _timer)) TimerActie(_timer, true);

			break;

		case 4: //keuze timeroutput	
			//volgende outputkanaal alleen te kiezen als voorgaande(deze dus) niet nul is maar een output bevat.
			//if (timeroutput[_timer][timerkeuzeoutput[_timer]] > 0) {
			timerkeuzeoutput[_timer]++;
			if (timerkeuzeoutput[_timer] > 5)timerkeuzeoutput[_timer] = 0;
			//DisplayTimer(_timer);
			break;
		}
		DisplayShow(20);
		break;

	case 1: //**************switch 2
		switch (programfase) {
		case 0: //in bedrijf lagere actie voor de actie knop instellen
			Actie_exe(false);
			break;
		case 1: //instellen scaler voor de timer
			if (timerscale[_timer] > 0)timerscale[_timer]--;
			break;
		case 2: //dec on time
			if (timerontijd[_timer] > _scrollsteps)timerontijd[_timer] -= _scrollsteps;

			break;
		case 3: //dec off time
			if (timerofftijd[_timer] > _scrollsteps)timerofftijd[_timer] -= _scrollsteps;
			break;
		case 4: //dec channel output kiezen 
			if (timeroutput[_timer][timerkeuzeoutput[_timer]] > 0) timeroutput[_timer][timerkeuzeoutput[_timer]]--;
			break;
		case 5: //dec aantgal timer cycles 0=default is doorgaan, continue
			if (timercycles[_timer] > 0)timercycles[_timer]--;
			break;
		case 6: //startstop of toggle aansturing door een andere timer
			timerstoppen |= (1 << _timer);
			break;
		}
		DisplayShow(21);
		break;
	case 2: //**************switch 3
		switch (programfase) {
		case 0: //in bedrijf, hogere actie voor de actie knop instellen
			Actie_exe(true);
			break;
		case 1: //instellen timer prescaler
			if (timerscale[_timer] < 2)timerscale[_timer]++;
			break;
		case 2: //inc on time
			if (timerontijd[_timer] < 10000 - _scrollsteps)timerontijd[_timer] += _scrollsteps;
			break;
		case 3: //inc off time
			if (timerofftijd[_timer] < (10000 - _scrollsteps))timerofftijd[_timer] += _scrollsteps;
			break;
		case 4: //inc channel output kiezen (aantalacties-2???)
			if (timeroutput[_timer][timerkeuzeoutput[_timer]] < AantalActies) timeroutput[_timer][timerkeuzeoutput[_timer]]++;
			break;
		case 5: //inc aantalcycles
			if (timercycles[_timer] < 99)timercycles[_timer]++;
			break;
		case 6:
			timerstoppen &= ~(1 << _timer);
			break;
		}
		DisplayShow(22);
		break;

	case 3: //*************switch 4, Program switch
		scrollmask = 0;
		switch (programfase) {
		case 0:
			Prg_end();
			break;
		case 1: //timer scale instellen
			break;
		case 2:
			scrollmask = B0111;
			break;
		case 3:
			scrollmask = B0110;
			break;
		case 4:
			timerkeuzeoutput[_timer] = 0;
			break;
		case 5: //cycles
			scrollmask = B0110;
			break;
		case 6: //start/stop door timer aansturing
			break;
		}
		break;
	}
	//DisplayShow(19); //deze algemene displayshow genereerd dubbele uitvoering van displayshow
}
void Prg_timertijd(byte _timer, bool _updown, bool _onofftijd) {
	//nog niet in gebruik?
}

//Sensoren
void DisplayHome() {
	switch (programfase) {
	case 0:
		digit[0] = Letter('H');
		digit[1] = Letter('o');
		clear23;
		break;
	case 1:
		digit[0] = Letter('o');
		digit[1] = Cijfer(10);
		DisplayAlias(homeoutput, false);
		dots;
		break;
	case 2: //instellen sub functie van de home aan/uit of alleen aan of alleen uit
		digit[0] = Letter('F');
		digit[1] = Letter('u');
		DisplayAlias(20 + sensorfunctie[0], true);
		break;
	}
}
void Prg_Homeoutput(bool _plusmin) {
	if (_plusmin) {
		if (homeoutput < AantalActies)homeoutput++;
		if (homeoutput == 1)homeoutput++; //stepper overslaan
		if (homeoutput == 10 or homeoutput == 11)homeoutput = 12; //sensoren niet als output instellen 10=Ho 11=Se
	}
	else {
		if (homeoutput > 0)homeoutput--;
		if (homeoutput == 1)homeoutput--;
		if (homeoutput == 10 or homeoutput == 11)homeoutput = 9;
	}
	DisplayShow(26);
}
void HomeActie() {
	//actie aangeroepen door de knop1 als actie op home staat. 
}

void DisplaySensor() {
	switch (programfase) {
	case 0: //in bedrijf
		digit[0] = Cijfer(5);
		digit[1] = Letter('E');
		clear23;
		break;
	case 1: //instellen output
		digit[0] = Letter('o');
		digit[1] = Cijfer(10);
		DisplayAlias(sensoroutput, false); //geen actie maar een output, dus actie 0 wordt --
		dots;
		break;
	case 2: //instellen functie aan, uit of aanuit
		DisplayClear();
		digit[0] = Letter('F');
		digit[1] = Letter('u');
		dots;
		DisplayAlias(20 + sensorfunctie[1], true);
		break;
	}
}
void Prg_sensoroutput(bool _plusmin) {
	if (_plusmin) {
		if (sensoroutput < AantalActies)sensoroutput++;
		if (sensoroutput == 10 or sensoroutput == 11)sensoroutput = 12; //sensoren niet als output instellen 10=Ho 11=Se
	}
	else {
		if (sensoroutput > 0)sensoroutput--;
		if (sensoroutput == 10 or sensoroutput == 11)sensoroutput = 9;
	}
	DisplayShow(25);
}
void Prg_sensorfunctie(byte _sensor, bool _plusmin) {
	if (_plusmin) {
		if (sensorfunctie[_sensor] < 2)sensorfunctie[_sensor]++;
	}
	else {
		if (sensorfunctie[_sensor] > 0)sensorfunctie[_sensor]--;
	}
	DisplayShow(50);
}
void ProgramSensors(byte _sensor) { //home en sensor 0=home 1=sensor
	scrollmask = 0;
	if (programfase > 2)programfase = 0;
	switch (programfase) {
	case 0:
		Prg_end(); //einde programmeer mode en opslaan data
		break;
	}
}
void SensorActie(byte _sensor, bool _onoff) {
	Serial.println(_onoff);
	//actie aangeroepen door de knop1 als actie op sensor staat. En door de aansluiting Sensor hoog actief
	//if (_onoff == false)return; //ff niks doen op uitgaan van de sensor
	byte _output;
	if (_sensor == 0) { //home
		_output = homeoutput;
	}
	else { //sensor
		_output = sensoroutput;
	}

	switch (_output) {
	case 1: //stepper
		if (_sensor == 1 && _onoff) StepperActie();  //alleen aansturing van sensor mogelijk, niet met Home
		break;
	case 2: //servo1
		if (_onoff)ServoActie(0);
		break;
	case 3: //servo2
		if (_onoff)ServoActie(1);
		break;
	case 4: //out1
		Off1;
		if (_onoff)On1;
		break;
	case 5:
		Off2;
		if (_onoff)On2;
		break;
	case 6:
		Off3;
		if (_onoff)On3;
		break;
	case 7:
		Off4;
		if (_onoff)On4;
		break;
	case 8:
		OffBezet;
		if (_onoff)OnBezet;
		break;
	case 9:
		OffAlarm;
		if (_onoff)OnAlarm;
		break;
	default:
		//Timers, kunnen alleen worden aangezet door een sensor, uitzetten kan door het aantal cyclus niet op continue (0) te zetten, 
		//of de timer te laten stoppen door een andere timer.
		if (_output > 11 && _output < 20) {
			byte _timer = _output - 12;
			if ((Timer & (1 << _timer)) == false) TimerSwitch(0, _timer);
		}
		break;
	}
}


