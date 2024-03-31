//#include <SPI.h>
#include <AccelStepper.h>
#include <math.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>
 


#define STEP_DRV1 22
#define DIR_DRV1 23

#define STEP_DRV2 4
#define DIR_DRV2 15

#define ENABLE_DRV1_2 21

#define inPin1 33
#define inPin2 25

#define colorInPin 13
#define blueLED 16
#define yellowLED 18

#define WiFiLED 17

#define microstepingConst 1 //1 for no microsteping

#define TotalTimeMS 100000 // 100k ms = 100s
#define SIMAStartTimeMS 1000 // 90k ms = 90s menjam radi svega na 4000

int pos = 0;    // variable to store the servo position
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
int servoPin = 27;
Servo myservo;  // create servo object to control a servo


unsigned long previousWiFiTime = 0;
unsigned long previousDisplayTime = 0;
unsigned long startMotorTime = 0;
unsigned long TOTAL_TIME_MS = 90000;
unsigned long matchStartTime = 0;

double wheelVolume = 169.64;  //milimeters
double spunPathPerStep = 0.85;  //milimeters
double wheelToWheel_distance = 85;
double wheelRadius = 27;
int a=0;
int b=0;
int flag=0;

int flagJustOnce = 0;

unsigned long numberOfPoints = 0;

char color = 0; // 0 za plavo 1 za zuto


// Define the stepper motor and the pins that is connected to
AccelStepper stepperR(1, STEP_DRV1, DIR_DRV1); // (Typeof driver: with 2 pins, STEP, DIR)
AccelStepper stepperL(1, STEP_DRV2, DIR_DRV2); // (Typeof driver: with 2 pins, STEP, DIR)


//SERVER SETUP
// Replace with your network credentials
const char* ssid     = "memristor";
const char* password = "ftnmemristor";

// UDP server parameters
WiFiUDP udp;
unsigned int localUdpPort = 8888;
unsigned long UDP_TX_PACKET_MAX_SIZE = 32;

void setup() {

  Serial.begin(112500);

  
  pinMode(STEP_DRV1, OUTPUT);
  pinMode(DIR_DRV1, OUTPUT);
  pinMode(ENABLE_DRV1_2, OUTPUT);

  pinMode(STEP_DRV2, OUTPUT);
  pinMode(DIR_DRV2, OUTPUT);

  pinMode(inPin1, INPUT);
  pinMode(inPin2, INPUT);

  pinMode(colorInPin, INPUT);
  pinMode(blueLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);



  delay(50);

  //digitalWrite(ENABLE_DRV1_2, HIGH); // HIGH gasi drajvere LOW pali

  //Serial.print("--------");
  //Serial.println(digitalRead(colorInPin));
  
  //colorInPin = digitalRead(colorInPin);

  	// Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	myservo.setPeriodHertz(50);    // standard 50 hz servo
	myservo.attach(servoPin, 400, 2400); // attaches the servo on pin 18 to the servo object
	// using default min/max of 1000us and 2000us
	// different servos may require different min/max settings
	// for an accurate 0 to 180 sweep

  myservo.write(0);

  if(digitalRead(colorInPin) == HIGH) {
    Serial.println("High");
    color = 0;
    digitalWrite(blueLED, HIGH);
    digitalWrite(yellowLED, LOW);
  }

  else {
    Serial.println("Low");
    color = 1;
    digitalWrite(blueLED, LOW );
    digitalWrite(yellowLED, HIGH);
  }
  
  //  **** ovaj deo mozda ne sme tu da stoji ako nije spojena serijska
   while (! Serial) {
    delay(1);
  }
  //  ****

  // Connect to Wi-Fi network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  //upaliti LED diodu
  pinMode(WiFiLED, OUTPUT);
  digitalWrite(WiFiLED, HIGH);

  // Start UDP server
  if (!udp.begin(localUdpPort)) {
    Serial.println("Failed to start UDP server");
    while (1) {}
  }

  Serial.print("Local IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("UDP server started on port ");
  Serial.println(localUdpPort);

  Serial.println("Waiting for 1212 to start loop.");

   // Wait for "1" to be received
  bool received = false;
  while (!received) {
    if (udp.parsePacket()) {
      //----------------------------------------------------------MISLIM DA OVDE TREBA DA SE STAVI unsigned long matchStartTime = milis();
      matchStartTime = millis();

      char buffer[128];
      Serial.print("Received flag: ");
      Serial.println(buffer);
      udp.read(buffer, 128);
      buffer[4] = 0;
      if (strcmp(buffer, "1212") == 0) {
        received = true;
      }
    }
  } 

  matchStartTime = millis();    //mozda treba i ovde da stoji ovo

  stepperR.setCurrentPosition(0); // Set the current position to 0 steps
  stepperL.setCurrentPosition(0); // Set the current position to 0 steps

  digitalWrite(ENABLE_DRV1_2, HIGH); // HIGH gasi drajvere LOW pali



  delay(100);

  Serial.println("Starting loop.");

}

void loop() {

  //delay(3000);  //treba zakomentarisati ako se koristi wifi
  //digitalWrite(ENABLE_DRV1_2, LOW); //takodje

  Serial.print("--------");
  Serial.println(digitalRead(colorInPin));
  

  if(digitalRead(colorInPin) == HIGH) {
    Serial.println("High");
    color = 0;
    digitalWrite(blueLED, HIGH);
    digitalWrite(yellowLED, LOW);
  }

  else {
    Serial.println("Low");
    color = 1;
    digitalWrite(blueLED, LOW );
    digitalWrite(yellowLED, HIGH);
  }


  unsigned long currentTime = millis();  

  while(currentTime - matchStartTime < SIMAStartTimeMS && flagJustOnce == 0) {   //ovaj uslov treba zameniti u neku while petlju sa invertovanim uslovom i ocitavanjem unutar petlje currentTime = milis();
    digitalWrite(ENABLE_DRV1_2, HIGH);   // drzi ugasene drajvere
    currentTime = millis();
    //digitalWrite()                    //upali LED diodu
  }

  if(flagJustOnce == 0) {

  digitalWrite(ENABLE_DRV1_2, LOW);   // pali drajvere

  Serial.println("PROSLO 90 Sekundi");
  
  currentTime = millis();
  
  //*******************************************************************************************************OVDE POZIVATI TAKTIKU*************************************************************************//
  
  //sredinaCosak(2400, 1200, 700, currentTime-matchStartTime);            //---------- SIMA 2
  //naProtivnickuStranu(3000, 1200, 700);                                 //---------- SIMA 3
    plenterPolje(1800, 1400, 900);    // na 1700, 1200, 700 radio lepo    ------------ SIMA 1
  //plenterPoljeV2(1800, 1400, 900);                                // ----------------- SIMA 1 treba da se menja, ne radi lepo
  //forwardMotion(2000, 1200);
  
  //*******************************************************************************************************OVDE POZIVATI TAKTIKU************************************************************************//

	for (pos = 0; pos <= 180; pos++) { // goes from 0 degrees to 180 degrees
		// in steps of 1 degree
		myservo.write(pos);    // tell servo to go to position in variable 'pos'
		//delay(15);             // waits 15ms for the servo to reach the position
	}

  flagJustOnce = 1;
  }

  //currentTime = millis();
  while(currentTime - matchStartTime > SIMAStartTimeMS + 7000)  {
    
    currentTime = millis();

    Serial.print("Sensor Data: ");
    char message[32];
    sprintf(message, "%lu", numberOfPoints);
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.print(message);
    udp.endPacket();
    
    Serial.println("Treba da je poslato");

    if(currentTime - matchStartTime > SIMAStartTimeMS + 9500) {
      digitalWrite(ENABLE_DRV1_2, HIGH); //ugasi drivere
    }
  }
while(1);
 // naProtivnickuStranu(2000, 1200, 700);   // ovo trci na prvorodjenom Simi -- mora da se podesi desni senzor, previse rano hvata
 // sredinaCosak(2400, 1200, 700, 100);   // ne treba 100 nego vreme starta Sime, ali to kasnije
 // forwardMotion(800, 1200);
}

void forwardMotion(double distance, int speed)
{
  double numberOfSteps = distance / spunPathPerStep;    //Number of steps calculated from required distance
  numberOfSteps = (int)numberOfSteps*2*microstepingConst;     //Multiplied by 2 because of 2:1 gear ratio
  
  //Serial.println(numberOfSteps);

   stepperR.setAcceleration(1200*microstepingConst); 
   stepperL.setAcceleration(1200*microstepingConst);

  stepperR.setMaxSpeed(speed*microstepingConst); // Set maximum speed value for the stepper
  stepperL.setMaxSpeed(speed*microstepingConst); // Set maximum speed value for the stepper
  /*floki se igra*/
  //stepperR.setMaxSpeed(2000); // Set maximum speed value for the stepper
  //stepperL.setMaxSpeed(2000); // Set maximum speed value for the stepper
  //stepperR.setAcceleration(8000); //5000
  //stepperL.setAcceleration(8000);//5000
  /*floki se igra end*/
  stepperR.moveTo(numberOfSteps);
  stepperL.moveTo(-numberOfSteps);

  Serial.println(numberOfSteps);

  while (stepperR.currentPosition() != numberOfSteps && stepperL.currentPosition() != -numberOfSteps) {

    stepperR.run();  // Move or step the motor implementing accelerations and decelerations to achieve the target position. Non-blocking function
    stepperL.run();
    //delay(5);
    /*if(digitalRead(inPin1) == 1 || digitalRead(inPin2) == 1)
     {
    Detection(numberOfSteps, numberOfSteps, speed, speed);
     }*/
    Detection(numberOfSteps, numberOfSteps, speed, speed);
   }

   stepperR.setCurrentPosition(0);
   stepperL.setCurrentPosition(0);
}

void backwardMotion(double distance)
{
  double numberOfSteps = distance / spunPathPerStep;    //Number of steps calculated from required distance
  numberOfSteps = (int)numberOfSteps*2;     //Multiplied by 2 because of 2:1 gear ratio

  //Serial.println(numberOfSteps);

  stepperR.setMaxSpeed(1000); // Set maximum speed value for the stepper
  stepperR.setAcceleration(50); // Set acceleration value for the stepper

  stepperL.setMaxSpeed(1000); // Set maximum speed value for the stepper
  stepperL.setAcceleration(50); // Set acceleration value for the stepper

  stepperR.moveTo(-numberOfSteps);
  stepperL.moveTo(numberOfSteps);

  while (stepperR.currentPosition() != -numberOfSteps && stepperL.currentPosition() != numberOfSteps) {
    stepperR.run();  // Move or step the motor implementing accelerations and decelerations to achieve the target position. Non-blocking function
    stepperL.run();
     
  }
   stepperR.setCurrentPosition(0);
   stepperL.setCurrentPosition(0);
}

void test(double distance)
{
  const int konst = 2500; //za 1900 distance je 1000
  const int spunPathDifference = 125;

  double numberOfStepsLevi = distance / spunPathPerStep;    //Number of steps calculated from required distance
  numberOfStepsLevi = (int)numberOfStepsLevi*2;     //Multiplied by 2 because of 2:1 gear ratio
  
  //desni tocak predje oko 134mm manje od levog
  double numberOfStepsDesni = (distance-spunPathDifference) / spunPathPerStep;    //Number of steps calculated from required distance
  numberOfStepsDesni = (int)numberOfStepsDesni*2;     //Multiplied by 2 because of 2:1 gear ratio

  //Serial.println(numberOfSteps);

   stepperR.setAcceleration(1200); 
   stepperL.setAcceleration(1200);

  stepperR.setMaxSpeed(1000); // Set maximum speed value for the stepper
  stepperL.setMaxSpeed(1000); // Set maximum speed value for the stepper

  stepperR.moveTo(numberOfStepsDesni); //faktor korekcije, jer levi prelazi vise pa onda desni na kraju namota kako ne treba
  stepperL.moveTo(-numberOfStepsLevi);

  int zastavica = 0;
  double actualPathDifference = 0;

  while (stepperR.currentPosition() != numberOfStepsDesni && stepperL.currentPosition() != -numberOfStepsLevi) {

    if(stepperR.currentPosition() >= numberOfStepsLevi-konst && zastavica == 0) {
      stepperR.setMaxSpeed(100);
      stepperL.setMaxSpeed(3000);
      zastavica = 1;
    }
    
    //skreci dok se ne podesi razlika u predjenim putevima izmedju tockova
    actualPathDifference = (stepperL.currentPosition()-stepperR.currentPosition())/2*spunPathPerStep;
    if(((actualPathDifference >= spunPathDifference - 2) || (actualPathDifference <= spunPathDifference + 2)) && zastavica > 0) {
      stepperR.setMaxSpeed(1000);
      stepperL.setMaxSpeed(1000);
      zastavica = -1;
    }
  
    stepperR.run();  // Move or step the motor implementing accelerations and decelerations to achieve the target position. Non-blocking function
    stepperL.run();

    Detection(numberOfStepsDesni, numberOfStepsLevi, stepperR.speed(), stepperL.speed());
   }


   stepperR.setCurrentPosition(0);
   stepperL.setCurrentPosition(0);
}

void sredinaCosak(double distance, int speed, int accel, unsigned long time) //accel 700
{
  const int konst2 = 3000;               // bilo 3100 -- odredjuje kada pocinje skretanje, sto manje to duze ide pravo
  const int spunPathDifference2 = 111; // bilo 125 -- smanjenje skracuje skretanje

  double numberOfStepsLevi = distance / spunPathPerStep;    //Number of steps calculated from required distance
  numberOfStepsLevi = (int)numberOfStepsLevi*2*microstepingConst;     //Multiplied by 2 because of 2:1 gear ratio
  //sa novim drajverima ide u rikverc, pa cu ovde korigovati
  //numberOfStepsLevi *= -1;

  //desni tocak predje oko 134mm manje od levog
  double numberOfStepsDesni = (distance-spunPathDifference2) / spunPathPerStep;    //Number of steps calculated from required distance & faktor korekcije, jer levi prelazi vise pa onda desni na kraju namota kako ne treba
  numberOfStepsDesni = (int)numberOfStepsDesni*2*microstepingConst;     //Multiplied by 2 because of 2:1 gear ratio
  //sa novim drajverima ide u rikverc, pa cu ovde korigovati
  //numberOfStepsDesni *= -1;

  //accel *= microstepingConst * (-1);
  //speed *= microstepingConst * (-1);
  //Serial.println(numberOfSteps);

  unsigned long functionTime = 0;
  int flagFinish = 0;
  int maxSpeedR = 0;
  int maxSpeedL = 0;

   stepperR.setAcceleration(accel); //na 700 lepo radio
   stepperL.setAcceleration(accel);

  stepperR.setMaxSpeed(speed); // Set maximum speed value for the stepper 1200 lepo radio
  stepperL.setMaxSpeed(speed); // Set maximum speed value for the stepper
  maxSpeedR = speed;
  maxSpeedL = speed;


  stepperR.moveTo(numberOfStepsDesni); 
  stepperL.moveTo(-numberOfStepsLevi);

  int zastavica = 0;
  double actualPathDifference = 0;

  while (stepperR.currentPosition() != numberOfStepsDesni && stepperL.currentPosition() != -numberOfStepsLevi) {

    if(stepperR.currentPosition() >= numberOfStepsLevi-konst2 && zastavica == 0) {  // bilo: stepperR.currentPosition() >= numberOfStepsLevi-konst2 && zastavica == 0
      stepperR.setMaxSpeed(speed-(int)(speed/12));
      stepperL.setMaxSpeed(speed+(int)(speed/12));
      maxSpeedR = speed - (int)(speed/12);
      maxSpeedL = speed + (int)(speed/12);
      zastavica = 1;
    }
    
    //skreci dok se ne podesi razlika u predjenim putevima izmedju tockova
    actualPathDifference = ((-stepperL.currentPosition()-stepperR.currentPosition())/(2* microstepingConst))*spunPathPerStep;
    //Serial.println(stepperL.currentPosition());
    if(((actualPathDifference >= spunPathDifference2 - 2) && (actualPathDifference <= spunPathDifference2 + 2)) && zastavica > 0) {
      stepperR.setMaxSpeed(speed);
      stepperL.setMaxSpeed(speed);
      maxSpeedR = speed;
      maxSpeedL = speed;
      zastavica = -1;
    }
  
    stepperR.run();  // Move or step the motor implementing accelerations and decelerations to achieve the target position. Non-blocking function
    stepperL.run();

    //Detection(numberOfStepsDesni, numberOfStepsLevi, maxSpeedR, maxSpeedL);

    functionTime = millis();
   /* if(functionTime - time > SIMAStartTimeMS + 9000){
      flagFinish = 1;
      break;
    }*/

   }
  if(flagFinish == 0){
    stepperR.setCurrentPosition(0);
    stepperL.setCurrentPosition(0);
  }
  else{
    //dodati nesto da se vrati da nije zavrsio radnju
  }
}

void naProtivnickuStranu(double distance, int speed, int accel) 
{

  const int firstKneeConst = 5000;      // bilo 3100 -- odredjuje pocetak skretanja, tj koliko dugo ide pravo pre nego sto pocne da skrece -> sto vece (mozda i manje, msm da je manje ali treba probati) to duze ide pravo
  const int secondKneeConst = 3000;     // msm da se ne koristi
  const int spunPathDifference3 = 107;  // odredjuje koliko dugo skrece, tj. koliki luk pravi -- smanjenje skracuje skretanje, tj luk
  const int firstArcLength = 117;       // trebalo bi da smanjenje skracuje skretanje || koliko prva krivina traje
  const int secondArcLength = 90;       // trebalo bi da smanjenje skracuje skretanje || koliko druga krivina traje

  uint32_t ocitaoSam = 0;

  double numberOfStepsLevi = (distance- spunPathDifference3) / spunPathPerStep;    //Number of steps calculated from required distance
  numberOfStepsLevi = (int)numberOfStepsLevi*2*microstepingConst;                  //Multiplied by 2 because of 2:1 gear ratio

  double numberOfStepsDesni = distance / spunPathPerStep;               //Number of steps calculated from required distance
  numberOfStepsDesni = (int)numberOfStepsDesni*2*microstepingConst;     //Multiplied by 2 because of 2:1 gear ratio

  unsigned long functionTime = 0;
  int flagFinish = 0;
  int maxSpeedR = 0;
  int maxSpeedL = 0;

  stepperR.setAcceleration(accel); //na 700 lepo radio
  stepperL.setAcceleration(accel);

  stepperR.setMaxSpeed(speed); // Set maximum speed value for the stepper 1200 lepo radio // bilo speed + speed/8
  stepperL.setMaxSpeed(speed); // Set maximum speed value for the stepper
  maxSpeedR = speed;
  maxSpeedL = speed;
  
  stepperR.moveTo(numberOfStepsDesni); 
  stepperL.moveTo(-numberOfStepsLevi);

  int zastavica = 0;
  double actualPathDifference = 0;

  while (stepperR.currentPosition() != numberOfStepsDesni && stepperL.currentPosition() != -numberOfStepsLevi)
  {

    actualPathDifference = ((-stepperL.currentPosition()-stepperR.currentPosition())/(2* microstepingConst))*spunPathPerStep;
    //Serial.println(actualPathDifference);

    /*if(((actualPathDifference >= -firstArcLength - 2) && (actualPathDifference <= -firstArcLength + 2)) && zastavica == 0) {  // bilo zastavica == 0
      stepperR.setMaxSpeed(speed);
      stepperL.setMaxSpeed(speed);
      zastavica = -1;
    }*/

     if(stepperR.currentPosition() >= numberOfStepsLevi-firstKneeConst && zastavica == 0) {  // bilo: stepperR.currentPosition() >= numberOfStepsLevi-konst2 && zastavica == 0
      stepperR.setMaxSpeed(speed+(int)(speed/12));
      stepperL.setMaxSpeed(speed-(int)(speed/12));
      maxSpeedR = speed + (int)(speed/12);
      maxSpeedL = speed - (int)(speed/12);
      zastavica = 1;
    }

    if(((actualPathDifference >= -firstArcLength - 2) && (actualPathDifference <= -firstArcLength + 2)) && zastavica > 0) {  // bilo zastavica == 0
      stepperR.setMaxSpeed(speed);
      stepperL.setMaxSpeed(speed);
      maxSpeedR = speed;
      maxSpeedL = speed;
      zastavica = -1;
    }

    /*if(stepperL.currentPosition() >= numberOfStepsLevi-firstKneeConst && zastavica < 0) {  // bilo: stepperR.currentPosition() >= numberOfStepsLevi-konst2 && zastavica == 0
      stepperR.setMaxSpeed(speed-(int)(speed/12));
      stepperL.setMaxSpeed(speed+(int)(speed/12));
      zastavica = 1;
    }

    if(((actualPathDifference >= secondArcLength - 2) && (actualPathDifference <= secondArcLength + 2)) && zastavica > 0) {
      
      stepperR.setMaxSpeed(speed+speed/12); // Set maximum speed value for the stepper 1200 lepo radio
      stepperL.setMaxSpeed(speed-speed/12); // Set maximum speed value for the stepper
      //zastavica = -1;
    }*/
    
    stepperR.run();  // Move or step the motor implementing accelerations and decelerations to achieve the target position. Non-blocking function
    stepperL.run();
    
    if(digitalRead(inPin1) == 1 || digitalRead(inPin2) == 1)
    {
      ocitaoSam++;
    }

    else
    {
      ocitaoSam = 0;
    }

    if(ocitaoSam > 1000)
    {
      Detection(numberOfStepsDesni, numberOfStepsLevi, maxSpeedR, maxSpeedL);
    }

    }

  stepperR.setCurrentPosition(0);
  stepperL.setCurrentPosition(0);

}

void plenterPolje(double distance, int speed, int accel)
{
  const int konst2 = 2900;               // bilo 3100 -- odredjuje kada pocinje skretanje, sto manje to duze ide pravo
  const int spunPathDifference2 = 50; // bilo 125 -- smanjenje skracuje skretanje

  uint32_t ocitaoSam = 0;

  double numberOfStepsLevi = (distance-spunPathDifference2) / spunPathPerStep;    //Number of steps calculated from required distance
  numberOfStepsLevi = (int)numberOfStepsLevi*2*microstepingConst;     //Multiplied by 2 because of 2:1 gear ratio
  //sa novim drajverima ide u rikverc, pa cu ovde korigovati
  //numberOfStepsLevi *= -1;

  //desni tocak predje oko 134mm manje od levog
  double numberOfStepsDesni = distance / spunPathPerStep;    //Number of steps calculated from required distance & faktor korekcije, jer levi prelazi vise pa onda desni na kraju namota kako ne treba
  numberOfStepsDesni = (int)numberOfStepsDesni*2*microstepingConst;     //Multiplied by 2 because of 2:1 gear ratio
  //sa novim drajverima ide u rikverc, pa cu ovde korigovati
  //numberOfStepsDesni *= -1;

  //accel *= microstepingConst * (-1);
  //speed *= microstepingConst * (-1);
  //Serial.println(numberOfSteps);

  unsigned long functionTime = 0;
  int flagFinish = 0;
  int maxSpeedR = 0;
  int maxSpeedL = 0;

  stepperR.setAcceleration(accel); //na 700 lepo radio
  stepperL.setAcceleration(accel);

  stepperR.setMaxSpeed(speed); // Set maximum speed value for the stepper 1200 lepo radio
  stepperL.setMaxSpeed(speed); // Set maximum speed value for the stepper
  maxSpeedR = speed;
  maxSpeedL = speed;


  stepperR.moveTo(numberOfStepsDesni); 
  stepperL.moveTo(-numberOfStepsLevi);

  int zastavica = 0;
  double actualPathDifference = 0;

  while (stepperR.currentPosition() != numberOfStepsDesni && stepperL.currentPosition() != -numberOfStepsLevi) {

    if(stepperR.currentPosition() >= numberOfStepsLevi-konst2 && zastavica == 0) {  // bilo: stepperR.currentPosition() >= numberOfStepsLevi-konst2 && zastavica == 0
      stepperR.setMaxSpeed(speed+(int)(speed/12));
      stepperL.setMaxSpeed(speed-(int)(speed/12));
      maxSpeedR = speed - (int)(speed/12);
      maxSpeedL = speed + (int)(speed/12);
      zastavica = 1;
    }
    
    //skreci dok se ne podesi razlika u predjenim putevima izmedju tockova
    actualPathDifference = ((-stepperL.currentPosition()-stepperR.currentPosition())/(2* microstepingConst))*spunPathPerStep;
    //Serial.println(stepperL.currentPosition());
    if(((actualPathDifference >= -spunPathDifference2 - 2) && (actualPathDifference <= -spunPathDifference2 + 2)) && zastavica > 0) {
      stepperR.setMaxSpeed(speed);
      stepperL.setMaxSpeed(speed);
      maxSpeedR = speed;
      maxSpeedL = speed;
      zastavica = -1;
    }
  
    stepperR.run();  // Move or step the motor implementing accelerations and decelerations to achieve the target position. Non-blocking function
    stepperL.run();

    if(digitalRead(inPin1) == 1 || digitalRead(inPin2) == 1)
    {
      ocitaoSam++;
    }

    else
    {
      ocitaoSam = 0;
    }

    if(ocitaoSam > 1000)
    {
      Detection(numberOfStepsDesni, numberOfStepsLevi, maxSpeedR, maxSpeedL);
    }
    
    functionTime = millis();
   /* if(functionTime - time > SIMAStartTimeMS + 9000){
      flagFinish = 1;
      break;
    }*/

   }
  if(flagFinish == 0){
    stepperR.setCurrentPosition(0);
    stepperL.setCurrentPosition(0);
  }
  else{
    //dodati nesto da se vrati da nije zavrsio radnju
  }
}

void plenterPoljeV2(double distance, int speed, int accel)
{
  const int konst2 = 2500;               // bilo 3100 -- odredjuje kada pocinje skretanje, sto manje to duze ide pravo
  const int konst3 = 4000;               //
  const int spunPathDifference2 = 50; // bilo 125 -- smanjenje skracuje skretanje

  uint32_t ocitaoSam = 0;

  double numberOfStepsLevi = (distance-spunPathDifference2) / spunPathPerStep;    //Number of steps calculated from required distance
  numberOfStepsLevi = (int)numberOfStepsLevi*2*microstepingConst;     //Multiplied by 2 because of 2:1 gear ratio
  //sa novim drajverima ide u rikverc, pa cu ovde korigovati
  //numberOfStepsLevi *= -1;

  //desni tocak predje oko 134mm manje od levog
  double numberOfStepsDesni = distance / spunPathPerStep;    //Number of steps calculated from required distance & faktor korekcije, jer levi prelazi vise pa onda desni na kraju namota kako ne treba
  numberOfStepsDesni = (int)numberOfStepsDesni*2*microstepingConst;     //Multiplied by 2 because of 2:1 gear ratio
  //sa novim drajverima ide u rikverc, pa cu ovde korigovati
  //numberOfStepsDesni *= -1;

  //accel *= microstepingConst * (-1); EE156/2019
  //speed *= microstepingConst * (-1);
  //Serial.println(numberOfSteps);

  unsigned long functionTime = 0;
  int flagFinish = 0;
  int maxSpeedR = 0;
  int maxSpeedL = 0;

  stepperR.setAcceleration(accel); //na 700 lepo radio
  stepperL.setAcceleration(accel);

  stepperR.setMaxSpeed(speed); // Set maximum speed value for the stepper 1200 lepo radio
  stepperL.setMaxSpeed(speed); // Set maximum speed value for the stepper
  maxSpeedR = speed;
  maxSpeedL = speed;


  stepperR.moveTo(numberOfStepsDesni); 
  stepperL.moveTo(-numberOfStepsLevi);

  int zastavica = 0;
  double actualPathDifference = 0;

  while (stepperR.currentPosition() != numberOfStepsDesni && stepperL.currentPosition() != -numberOfStepsLevi) {

    if(stepperR.currentPosition() >= numberOfStepsLevi-konst2 && zastavica == 0) {  // bilo: stepperR.currentPosition() >= numberOfStepsLevi-konst2 && zastavica == 0
      stepperR.setMaxSpeed(speed+(int)(speed/7));
      stepperL.setMaxSpeed(speed-(int)(speed/7));
      maxSpeedR = speed - (int)(speed/7);
      maxSpeedL = speed + (int)(speed/7);
      zastavica = 1;
    }
    
    //skreci dok se ne podesi razlika u predjenim putevima izmedju tockova
    actualPathDifference = ((-stepperL.currentPosition()-stepperR.currentPosition())/(2* microstepingConst))*spunPathPerStep;
    //Serial.println(stepperL.currentPosition());
    if(((actualPathDifference >= -spunPathDifference2 - 2) && (actualPathDifference <= -spunPathDifference2 + 2)) && zastavica == 1) {
      stepperR.setMaxSpeed(speed-(int)(speed/7));
      stepperL.setMaxSpeed(speed+(int)(speed/7));
      maxSpeedR = speed-(int)(speed/8);
      maxSpeedL = speed+(int)(speed/8);
      zastavica = 2;
    }

    if(stepperR.currentPosition() >= numberOfStepsLevi - konst3 && zastavica == 2) {
      stepperR.setMaxSpeed(speed);
      stepperL.setMaxSpeed(speed);
      maxSpeedR = speed;
      maxSpeedL = speed;
      zastavica = -1;
    }

    if(((actualPathDifference >= -2) && (actualPathDifference <= + 2)) && zastavica == -1) {
      stepperR.setMaxSpeed(speed);
      stepperL.setMaxSpeed(speed);
      maxSpeedR = speed;
      maxSpeedL = speed;
      zastavica = 3;
    }
      
  
    stepperR.run();  // Move or step the motor implementing accelerations and decelerations to achieve the target position. Non-blocking function
    stepperL.run();

    if(digitalRead(inPin1) == 1 || digitalRead(inPin2) == 1)
    {
      ocitaoSam++;
    }

    else
    {
      ocitaoSam = 0;
    }

    if(ocitaoSam > 1000)
    {
      Detection(numberOfStepsDesni, numberOfStepsLevi, maxSpeedR, maxSpeedL);
    }
    
    functionTime = millis();
   /* if(functionTime - time > SIMAStartTimeMS + 9000){
      flagFinish = 1;
      break;
    }*/

   }
  if(flagFinish == 0){
    stepperR.setCurrentPosition(0);
    stepperL.setCurrentPosition(0);
  }
  else{
    //dodati nesto da se vrati da nije zavrsio radnju
  }
}

void polukrug(double distance, int speed, int accel)
{
  const int spunPathDifference2 = 250; // bilo 125 -- smanjenje skracuje skretanje

  uint32_t ocitaoSam = 0;

  double numberOfStepsLevi = (distance) / spunPathPerStep;    //Number of steps calculated from required distance
  numberOfStepsLevi = (int)numberOfStepsLevi*2*microstepingConst;     //Multiplied by 2 because of 2:1 gear ratio
  //sa novim drajverima ide u rikverc, pa cu ovde korigovati
  //numberOfStepsLevi *= -1;

  //desni tocak predje oko 134mm manje od levog
  double numberOfStepsDesni = (distance-spunPathDifference2) / spunPathPerStep;    //Number of steps calculated from required distance & faktor korekcije, jer levi prelazi vise pa onda desni na kraju namota kako ne treba
  numberOfStepsDesni = (int)numberOfStepsDesni*2*microstepingConst;     //Multiplied by 2 because of 2:1 gear ratio
  //sa novim drajverima ide u rikverc, pa cu ovde korigovati
  //numberOfStepsDesni *= -1;

  //accel *= microstepingConst * (-1);
  //speed *= microstepingConst * (-1);
  //Serial.println(numberOfSteps);

  unsigned long functionTime = 0;
  int flagFinish = 0;
  int maxSpeedR = 0;
  int maxSpeedL = 0;

  stepperR.setAcceleration(accel-accel/3); //na 700 lepo radio
  stepperL.setAcceleration(accel+accel/3);

  stepperR.setMaxSpeed(speed-speed/3); // Set maximum speed value for the stepper 1200 lepo radio
  stepperL.setMaxSpeed(speed+speed/3); // Set maximum speed value for the stepper
  maxSpeedR = speed;
  maxSpeedL = speed;


  stepperR.moveTo(numberOfStepsDesni); 
  stepperL.moveTo(-numberOfStepsLevi);

  int zastavica = 0;
  double actualPathDifference = 0;

  while (stepperR.currentPosition() != numberOfStepsDesni && stepperL.currentPosition() != -numberOfStepsLevi) {
    
    //skreci dok se ne podesi razlika u predjenim putevima izmedju tockova
    actualPathDifference = ((-stepperL.currentPosition()-stepperR.currentPosition())/(2* microstepingConst))*spunPathPerStep;
    //Serial.println(stepperL.currentPosition());
    if(((actualPathDifference >= -spunPathDifference2 - 2) && (actualPathDifference <= -spunPathDifference2 + 2)) && zastavica == 0) {
      stepperR.setMaxSpeed(speed);
      stepperL.setMaxSpeed(speed);
      maxSpeedR = speed;
      maxSpeedL = speed;
      zastavica = -1;
    }
  
    stepperR.run();  // Move or step the motor implementing accelerations and decelerations to achieve the target position. Non-blocking function
    stepperL.run();

    if(digitalRead(inPin1) == 1 || digitalRead(inPin2) == 1)
    {
      ocitaoSam++;
    }

    else
    {
      ocitaoSam = 0;
    }

    if(ocitaoSam > 1000)
    {
      Detection(numberOfStepsDesni, numberOfStepsLevi, maxSpeedR, maxSpeedL);
    }
    
    functionTime = millis();
   /* if(functionTime - time > SIMAStartTimeMS + 9000){
      flagFinish = 1;
      break;
    }*/

   }
  if(flagFinish == 0){
    stepperR.setCurrentPosition(0);
    stepperL.setCurrentPosition(0);
  }
  else{
    //dodati nesto da se vrati da nije zavrsio radnju
  }
}

void Rotation(int degrees)
{
  double circularArc = (wheelToWheel_distance*3.14) * degrees/360;
  double ratio = circularArc/wheelVolume;
  double numberOfSteps = ratio * 400;    //Calculting number of steps from desired angle
  numberOfSteps = (int)numberOfSteps;     

  //Serial.println(numberOfSteps);

  stepperR.setMaxSpeed(2500); // Set maximum speed value for the stepper
  stepperR.setAcceleration(1000); // Set acceleration value for the stepper
  stepperL.setMaxSpeed(2500); // Set maximum speed value for the stepper
  stepperL.setAcceleration(1000); // Set acceleration value for the stepper
  

  stepperR.moveTo(numberOfSteps);
  stepperL.moveTo(numberOfSteps);

  while (stepperR.currentPosition() != numberOfSteps && stepperL.currentPosition() != numberOfSteps) {
    stepperR.run();  // Move or step the motor implementing accelerations and decelerations to achieve the target position. Non-blocking function
    stepperL.run();
     
  }
   stepperR.setCurrentPosition(0);
   stepperL.setCurrentPosition(0);
}

void Detection(int stepsNumberR, int stepsNumberL, int speedR, int speedL)// cekaj dok se ne pomeri 
{

  if(digitalRead(inPin1) == 1 || digitalRead(inPin2) == 1)
  {
      a=stepperR.currentPosition();
      b=stepperL.currentPosition();
      
    
  }

  while(digitalRead(inPin1) == 1 || digitalRead(inPin2) == 1)  // proveriti da li se menja na 0 ili 1 kad detektuje
    { 
      //Serial.print("Prepreka");
      stepperR.stop();
      stepperL.stop();
      flag = 1;

    }

   if(flag == 1)
    {
           stepperR.setCurrentPosition(a);
           stepperL.setCurrentPosition(b);
           //stepperR.setMaxSpeed(speedR); // pravi gresku kada u skretanju naidje na prepreku, pa dok zauka on ide pravolinijski
           //stepperL.setMaxSpeed(speedL);
           stepperR.setSpeed(speedR);
           stepperL.setSpeed(speedL);
           stepperR.moveTo(stepsNumberR);
           stepperL.moveTo(-stepsNumberL);
           flag = 0;
      
    }

}