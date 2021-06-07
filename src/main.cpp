/* De sensoren staan op de volgorde 4  2  1  3  5


                                          6 

// Turn on motor A & B
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    delay(2000);

    // Now change motor directions
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
*/ 
#include <Arduino.h>
#include <servo.h>
#include <Stepper.h>
#include <Adafruit_VL53L0X.h>

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
int rechtsom = 0;
int linksom = 0;
int stopcount = 0;
int diabolo = 0;
int i = 0;
#define DS1 22
#define DS2 24
#define DS3 26
#define DS4 28
#define DS5 30
#define DS6 32

int StepperBaseRPM = 5;
int bl = 1;
int wh = 0;
int start = 0;
int s1, s2, s3, s4, s5, s6;
int countR = 0;
int rechtsomdiabol = 0; //Rechtsomdiabol is om de steps van de stepper te onthouden
int rechtsomdiaboltot = 0;
int hoogtediabol = 10; //Hoogte diabolo kan nog veranderen
int StepperBasedelay = 160; //Delay is nog niet goed bepaald
int StepperHeaddelay = 750; //Delay is nog niet goed bepaald
int StepperBaseRPM = 55; //RPM is nog niet bepaald
int StepperHeadRPM = 20; //RPM is nog niet bepaald
int roulettediabol = 9;
int roulettediabolL = 0;
int roulettediabolR = 0;
int roulettetot = 0;
int readdiabolo = 0;
int countL = 0;
// de end stepper
Stepper StepperBase = Stepper(200, 45 ,47 ,49 ,51); //nog random digital pins gezet
Stepper StepperHead = Stepper(200, 25, 27 , 29, 31); //nog random digital pins gezet

//servo
Servo Gripper;
Servo RopeServo;
int servohoog = 90;
int servolaag = 0;
int servovast = 90;
int servolos = 0;

VL53L0X_RangingMeasurementData_t measure;
//pins voor de servo's
#define Gripperpin 3 //de pins moeten nog gekozen worden
#define RopeServoPin 4 //de pins moeten nog gekozen worden

int zone[11] = {0, 183, 368, 552, 732, 916, 1100, 1284, 1468, 1652, 1832};
int zone2[11] = {2568, 2752, 2932, 3116, 3300, 3484, 3668, 3852, 4032, 4216, 4400};

void Rsteps(){
  while(countR == 0){}
  StepperBase.step(zone[i]);
  i++;
  // shutdown pins moet nog worden toegepast
  measure.RangeMilliMeter;
  if(measure.RangeMilliMeter <= hoogtediabol){
    rechtsomdiabol = zone[i]; //rechtsomdiabol wordt gebruikt om data op te slaan
    Serial.println("Ziet diabol recht in de draai beweging");
    countR = 1;
  }
  delay(5);
  // shutdown pins moet nog worden toegepast
  measure.RangeMilliMeter;
  if(measure.RangeMilliMeter <= hoogtediabol){
    rechtsomdiabol = zone[i]; //rechtsomdiabol wordt gebruikt om data op te slaan
     Serial.println("Ziet diabol rechts in de draai beweging");
    countR = 1;
  }
  delay(5);
  // shutdown pins moet nog worden toegepast
  measure.RangeMilliMeter;
  if(measure.RangeMilliMeter <= hoogtediabol){
    rechtsomdiabol = zone[i]; //rechtsomdiabol wordt gebruikt om data op te slaan
     Serial.println("Ziet diabol rechts in de draai beweging");
    countR = 1;
  }
}

void Lsteps(){
  while(countL == 0){}
  StepperBase.step(zone2[i]);
  i++;
  // shutdown pins moet nog worden toegepast
  measure.RangeMilliMeter;
  if(measure.RangeMilliMeter <= hoogtediabol){
    rechtsomdiabol = zone2[i]; //rechtsomdiabol wordt gebruikt om data op te slaan
    Serial.println("Ziet diabol links in de draai beweging");
    countL = 1;
  }
  delay(5);
  // shutdown pins moet nog worden toegepast
  measure.RangeMilliMeter;
  if(measure.RangeMilliMeter <= hoogtediabol){
    rechtsomdiabol = zone2[i]; //rechtsomdiabol wordt gebruikt om data op te slaan
     Serial.println("Ziet diabol links in de draai beweging");
    countL = 1;
  }
  delay(5);
  // shutdown pins moet nog worden toegepast
  measure.RangeMilliMeter;
  if(measure.RangeMilliMeter <= hoogtediabol){
    rechtsomdiabol = zone2[i]; //rechtsomdiabol wordt gebruikt om data op te slaan
     Serial.println("Ziet diabol links in de draai beweging");
    countL = 1;
  }
}

void scandiaboloL( int i = 0){
  while(readdiabolo == 0){
    i=i+200; //steps gedeelt door 4 door cnc shield 200/4=50
    StepperHead.step(i);
    delay(StepperHeaddelay);
    measure.RangeMilliMeter; //shutdown pins moeten nog worden toegevoegd
    if(measure.RangeMilliMeter >= roulettediabol){
    roulettediabolL = i;
    readdiabolo = 1;
    }
  }
}

void scandiaboloR( int i = 0){
  while(readdiabolo == 0){
    StepperHead.step(0);
    delay(1000);
    i=i-200; //steps gedeelt door 4 door cnc shield -200/4=-50
    StepperHead.step(i);
    delay(StepperHeaddelay);
    measure.RangeMilliMeter; //shutdown pins moeten nog worden toegevoegd
    if(measure.RangeMilliMeter >= roulettediabol){
    roulettediabolR = i;
    readdiabolo = 1;
    }
  }
}

void stop() {
  Serial.println("auto stopt");
  digitalWrite(9, HIGH);
  digitalWrite(8, HIGH);
}

void homeposition(){
Serial.println("Staat in homeposition");
// Servo staat nog op random graden
Gripper.write(servolos);
RopeServo.write(servohoog);
StepperBase.step(0);
StepperHead.step(0); 
}


void statesensoren(){
    s6 = digitalRead(DS6);
    s5 = digitalRead(DS5);
    s4 = digitalRead(DS4);
    s3 = digitalRead(DS3);
    s2 = digitalRead(DS2);
    s1 = digitalRead(DS1);
}


//start functie hij begint niet zonder dat dit waar is 
void Start(){
  homeposition();
  stop();
  while(start == 0){
    statesensoren();
    if ((s2 == bl) && (s3 == bl) && (s1 == bl) && (s4 == wh) && (s5 == wh)){
      start = 1;
    }
    else{
     Serial.println("De auto is een imbeciel");
     delay(10);
     
    }
  }
}



void scandiabolo(){
 if(measure.RangeMilliMeter <= 10){ //het getal 10 is random gekozen
 Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
 
 }
}



void gamestrategie(){
  homeposition();
  Rsteps();
  scandiaboloL();
  scandiaboloR();
  roulettetot= roulettediabolL - roulettediabolR;
  StepperHead.step(roulettetot);
  delay(1000);
  RopeServo.write(servolaag); //random getal
  Gripper.write(servovast); //random getal
  RopeServo.write(servohoog); //random getal
  StepperHead.step(0);
  rechtsomdiaboltot = rechtsomdiabol - 2200; //staat nu 180 gedraaid van waar de diabolo eerst stond
  StepperBase.step(rechtsomdiaboltot);
  delay(2000);
  homeposition();
  delay(2000); //zodat de diabolo in positie komt
  delay(3000); //moet volgends de reader
}

void straight() {
  Serial.println("Driving straight");
  //Driving forward
  //Motor A forward @ full speed
    digitalWrite(12, HIGH); //Establishes forward direction of Channel A
    digitalWrite(9, LOW);   //Disengage the Brake for Channel A
    analogWrite(3, 255);   //Spins the motor on Channel A at full speed

  //Motor B forward @ full speed
    digitalWrite(13, HIGH);  //Establishes forward direction of Channel B
    digitalWrite(8, LOW);   //Disengage the Brake for Channel B
    analogWrite(11, 255);    //Spins the motor on Channel B at full speed
}

void right() {
  Serial.println("Gaat naar rechts");
  //Driving right
  //Motor A forward @ full speed forward
    digitalWrite(9, LOW);   //Disengage the Brake for Channel A
    digitalWrite(12, LOW); //Establishes reverse direction of Channel A
    analogWrite(3, 255);   //Spins the motor on Channel A at full speed

  //Motor B forward @ full speed backward
    digitalWrite(8, LOW);   //Disengage the Brake for Channel B
    digitalWrite(13, HIGH);  //Establishes forward direction of Channel B
    analogWrite(11, 255);    //Spins the motor on Channel B at full speed backward
}

void left() {
  Serial.println("Gaat naar links");
  //Driving left
  //Motor A forward @ full speed backward
    digitalWrite(12, HIGH); //Establishes  direction of Channel A
    digitalWrite(9, LOW);   //Disengage the Brake for Channel A
    analogWrite(3, 255);   //Spins the motor on Channel A at full speed backward

  //Motor B forward @ full speed forward
    digitalWrite(13, LOW);  //Establishes forward direction of Channel B
    digitalWrite(8, LOW);   //Disengage the Brake for Channel B
    analogWrite(11, 255);     //Spins the motor on Channel B at full speed forward
}
  

void checkcorrectierechts(){
     //correctie rechts
    if(s3 == bl){
      Serial.println("corrigeert naar rechts");
      analogWrite(3, 20);     //Spins the motor on Channel A at full speed 
      digitalWrite(12, HIGH);  //Establishes forward direction of Channel A
    
      analogWrite(11, 255);     //Spins the motor on Channel B at full speed forward
      digitalWrite(13, HIGH);  //Establishes forward direction of Channel B
      }
}
void checkcorrectielinks(){
 //correctie links
    
    if(s2 == bl){
      Serial.println("corrigeert naar rechts");
      analogWrite(3, 255);     //Spins the motor on Channel A at full speed 
      digitalWrite(12, HIGH);  //Establishes forward direction of Channel A

      analogWrite(11, 20);     //Spins the motor on Channel B at full speed forward
      digitalWrite(13, HIGH);  //Establishes forward direction of Channel B
    }
}

void checkrechtsaf(){
  //rechts af
  if (s5 == bl){//rechter sensor 

  delay(5); // delay voor 2e lezing
   statesensoren();

    //kijken of het alsnog pauze of stop is 
    if(s4 == bl){// hij is nu pauze
    Serial.println("pauze");
      delay(350);
      statesensoren();
         if((s4 == bl) && (s5 == bl)){
           delay(5100);
          Serial.println("hij staat nu in zijn stop");
          stopcount = 1;
      }
      else{
      stop();
      gamestrategie();
      }
    }
    //anders is het een bocht
    else{
      s6 = digitalRead(DS6);
    //1e stap hij moet door rijden totdat sensor 6 wit is
      while (s6 == bl){
          s6 = digitalRead(DS6);
          Serial.println("S6 is zwart rechtsom");
      }
       Serial.println("S6 is wit");
        stop();
        delay(200);
        right();
        //vanaf hier moet hij opzoek naar de lijn en kijken of sensor 1 weer zwart word
        s1 = digitalRead(DS1);
        while(s1 == wh){
          s1 = digitalRead(DS1);
          Serial.println("S1 wit"); 
        } 
        Serial.println("S1 zwart");
        stop();
    }
  }
}

void checklinksaf(){
  // links af
 if (s4 == bl){//rechter sensor
    //1e stap hij moet door rijden totdat sensor 6 wit is
    Serial.println("Ik ben een imbiciel");
    delay(5); // delay voor 2e lezing
    statesensoren();
    
    //kijken of het alsnog pauze of stop is 
    if(s5 == bl){// hij is nu pauze
      Serial.println("pauze");
      delay(350);
      statesensoren();
         if((s4 == bl) && (s5 == bl)){
           delay(5100);
          Serial.println("hij staat nu in zijn stop");
          stopcount = 1;
      }
      else{
      stop();
      gamestrategie();
      }
    }
   else{
    s6 = digitalRead(DS6);
    // hij moet hier nog een check uitvoeren

      while (s6 == bl){
          s6 = digitalRead(DS6);
          Serial.println("S6 is zwart linksom");
      }
      Serial.println("S6 is wit");
      stop();
      delay(200);
      left();
      //vanaf hier zoek hij weer de lijn
      s1 = digitalRead(DS1);
      while(s1 == wh){
        s1 = digitalRead(DS1);
        Serial.println("S1 wit");
      }
      Serial.println("S1 zwart");
      stop();
   }
  }
  }


 void countstop(){
   if(stopcount == 1){
s1 = digitalRead(DS1);
while(s1 == bl){
     digitalWrite(12, HIGH); //Establishes forward direction of Channel A
    digitalWrite(9, LOW);   //Disengage the Brake for Channel A
    analogWrite(3, 135);   //Spins the motor on Channel A at full speed

  //Motor B forward @ full speed
    digitalWrite(13, HIGH);  //Establishes forward direction of Channel B
    digitalWrite(8, LOW);   //Disengage the Brake for Channel B
    analogWrite(11, 135);    //Spins the motor on Channel B at full speed
    s1 = digitalRead(DS1);
}
while (stopcount == 1)
{
  stop();
  Serial.println("hij is aan het einde van het traject");
}
}
}

void setup() {
  
  Serial.begin(9600); //Starts the serial monitor
  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
  //Setup Channel A
  pinMode(14, OUTPUT); //Initiates Motor Channel A pin
  pinMode(15, OUTPUT); //Initiates Brake Channel A pin

  //Setup Channel B
  pinMode(11, OUTPUT); //Initiates Motor Channel A pi
  pinMode(8, OUTPUT);  //Initiates Brake Channel A pin

  //servo pins
  Gripper.attach(Gripperpin);
  RopeServo.attach(RopeServoPin);

  //Setup sensor
  pinMode(DS1, INPUT);
  pinMode(DS2, INPUT);
  pinMode(DS3, INPUT);
  pinMode(DS4, INPUT);
  pinMode(DS5, INPUT);
  pinMode(DS6, INPUT);
  
  StepperBase.setSpeed(StepperBaseRPM);
  StepperHead.setSpeed(StepperHeadRPM);

Start();
}

void loop(){

  // assigns signal 1-6 from digital sensor 1-6
statesensoren();

//tijdelijke straight
straight();

checkrechtsaf(); //Als sensor 5 black ziet gaat deze rechtsaf
statesensoren(); 
checklinksaf(); //Als sensor 4 black ziet gaat deze rechtsaf
statesensoren();

countstop();

statesensoren();

checkcorrectielinks(); // Als sensor 2 black ziet corrigeert deze naar rechts
 
checkcorrectierechts(); // Als sensor 3 black ziet corrigeert deze naar links
}