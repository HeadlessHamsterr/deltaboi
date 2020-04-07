#include <Arduino.h>
#include <Servo.h>
#include "mydef.h"
#include <LiquidCrystal.h> 
#include "RTClib.h"

//#define test
#define demo  //Versie van het programma dat draait zonder lcd, knoppen en RTC aan te sluiten. Arm beweegt naar alle pillenbakjes en het uitgave-bakje.

void polsCalc(int x, int y, int z); //Functie voor het berekenen van de coördinaten van de polsen voor een gekozen coördinaat
void elbowACalc(int gradenA); //Functie voor het berekenen van het coördinaat van elleboog A voor een bepaald aantal graden
void elbowBCalc(int gradenB); //Functie voor het berekenen van het coördinaat van elleboog B voor een bepaald aantal graden
void elbowCCalc(int gradenC); //Functie voor het berekenen van het coördinaat van elleboog C voor een bepaald aantal graden
float distanceCalc(float polsX, float polsY, float elbowX, float elbowY, float polsZ, float elbowZ);  //Functie om de afstand tussen een pols en een elleboog berekenen
void degreeCalc(int x, int y, int z); //Functie voor het bepalen van de juiste waardes voor de servo's voor een gekozen coördinaat
void move(int x, int y, int z); //Functie voor het maken van een beweging naar een coördinaat
void debug(); //Alle waardes uit alle functies printen
void jaw(bool close); //Functie voor het openen en sluiten van de klauw
void solenoid(bool extend); //Functie voor het activeren en deactiveren van de solenoid
void startup(); //Functie die gedraaid wordt aan bij het opstarten waarbij de te pakken bakjes worden geselecteerd
void selectPil(char bakje); //Functie voor het programmeren welke bakjes wanneer moeten worden gepakt
void pakBakje(char bakje);  //Functie voor het pakken van het geselecteerde bakje
void playSound(); //Functie om een geluidssignaal te sturen als de pillen klaar liggen

//RTC Module object aanmaken
RTC_DS1307 rtc;

const int rs = 5, en = 4, d4 = 3, d5 = 2, d6 = 6, d7 = 7; //Variabelen moeten const ints zijn voor de LiquidCrystal library, defines werken niet


//LCD object aanmaken
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


//Servo objecten aanmaken
Servo servoA;
Servo servoB;
Servo servoC;
Servo servoJaw;

bool actiefA = false; //Variabele om aan te geven of een bakje gepakt moet worden of niet
bool actiefB = false;
bool actiefC = false;
bool actiefD = false;

char uurA;  //Variabele voor het uur wanneer het bakje gepakt moet worden
char minuutA; //Variabele voro de minuut wanneer het bakje gepakt moet worden
char uurB;
char minuutB;
char uurC;
char minuutC;
char uurD;
char minuutD;

void setup(){
  Serial.begin(9600);

  pinMode(servoPinA, OUTPUT);
  pinMode(servoPinB, OUTPUT);
  pinMode(servoPinC, OUTPUT);
  pinMode(servoPinJaw, OUTPUT);
  pinMode(solPin, OUTPUT);
  pinMode(limSwitch, INPUT);
  pinMode(knopA, INPUT);
  pinMode(knopB, INPUT);
  pinMode(knopC, INPUT);
  pinMode(knopD, INPUT);
  pinMode(knopUp, INPUT);
  pinMode(knopDown, INPUT);

#ifndef demo
  if(!rtc.begin()){ //Als de RTC niet beschikbaar is, kan het programma niet beginnen
    Serial.println("RTC niet gevonden");
    while(1);
  }

  if(!rtc.isrunning()){ //Als de RTC wel beschikbaar is maar niet draait, kan het programma niet beginnen
    Serial.println("RTC draait niet!");
    while(1);
  }

  rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); //De tijd van de RTC gelijk zetten met de momentele tijd
#endif
  servoA.attach(servoPinA);
  servoB.attach(servoPinB);
  servoC.attach(servoPinC);
  servoJaw.attach(servoPinJaw);

  servoJaw.write(openJawPos); //Klauw openen

  lcd.begin(16, 2); //LCD initializeren
  lcd.clear();  //LCD voor de zekerheid leeg maken

  #ifdef test
    debug();
    servoA.write(testGraden);
    servoB.write(testGraden);
    servoC.write(testGraden);
  #endif

  #ifndef demo
    startup();  //Bakjes selecteren
  #endif

  #ifdef demo
    Serial.println("Bakje A berekenen: ");
    delay(1000);
    move(bakjeAX, bakjeAY, bakjeAZ);  //Bewegen naar de locatie van bakje A
    Serial.println("Bakje B berekenen: ");
    delay(1000);
    move(bakjeBX, bakjeBY, bakjeBZ);  //Bewegen naar de locatie van bakje B
    Serial.println("Bakje C berekenen: ");
    delay(1000);
    move(bakjeCX, bakjeCY, bakjeCZ);  //Bewegen naar de locatie van bakje C
    Serial.println("Bakje D berekenen: ");
    delay(1000);
    move(bakjeDX, bakjeDY, bakjeDZ);  //Bewegen naar de locatie van bakje D
    Serial.println("Uitgave bakje berekenen: ");
    delay(1000);
    move(uitgaveX, uitgaveY, uitgaveZ);  //Bewegen naar de locatie van de pillen uitgave
    Serial.println("Alle posities berekend!");
  #endif
}

void loop(){
  #ifndef demo
    DateTime now = rtc.now(); //Tijd gelijkzetten aan de momentele tijd van de RTC

    if(actiefA && now.hour() == uurA && now.minute() == minuutA){ //Als bakje A geselecteerd is en de tijd goed is, bakje pakken
      pakBakje('A');
    }
    if(actiefB && now.hour() == uurB && now.minute() == minuutB){ //Als bakje B geselecteerd is en de tijd goed is, bakje pakken
      pakBakje('B');
    }
    if(actiefC && now.hour() == uurC && now.minute() == minuutC){ //Als bakje C geselecteerd is en de tijd goed is, bakje pakken
      pakBakje('C');
    }
    if(actiefD && now.hour() == uurD && now.minute() == minuutD){ //Als bakje D geselecteerd is en de tijd goed is, bakje pakken
      pakBakje('D');
    }
  #endif
}

//Functie voor het berekenen van de coördinaten van de polsen voor een gekozen coördinaat
void polsCalc(int x, int y, int z){
  polsAX = x + L_TOP; //X-coordinaat van pols A
  polsAY = y; //Y-coordinaat van pols A

  polsZ = z;//Z_BASIS - z - Z_TOOL_OFFSET;  //Z-coordinaat van de polsen

  polsBX = x - (cos(rad60) * L_TOP);  //X-coordinaat van pols B
  polsBY = y + (sin(rad60) * L_TOP);  //Y-coordinaat van pols B

  polsCX = x - (cos(rad60) * L_TOP);  //X-coordinaat van pols C
  polsCY = y - (sin(rad60) * L_TOP);  //Y-coordinaat van pols C
}

//Functie voor het berekenen van het coördinaat van elleboog A voor een bepaald aantal graden
void elbowACalc(int gradenA){
  radialen.a = gradenA * radOmreken;  //Waarde van servo A in graden omzetten naar radialen voor de goniometrische functies
  elbowAX = SERVO_A_X + (L_ARM_1 * cos(radialen.a));  //X-coordinaat van elleboog A berekenen
  elbowAY = SERVO_A_Y;  //Y-coordinaat van elleboog A berekenen
  elbowAZ = SERVO_Z + (L_ARM_1 * sin(radialen.a));  //Z-coordinaat van elleboog A berekenen
}

//Functie voor het berekenen van het coördinaat van elleboog B voor een bepaald aantal graden
void elbowBCalc(int gradenB){
  radialen.b = gradenB * radOmreken;  //Waarde van servo B in graden omzetten naar radialen voor de goniometrische functies
  elbowBX = SERVO_B_X - (cos(rad60) * L_ARM_1 * cos(radialen.b)); //X-coordinaat van elleboog B berekenen
  elbowBY = SERVO_B_Y + (sin(rad60) * L_ARM_1 * cos(radialen.b)); //Y-coordinaat van elleboog B berekenen
  elbowBZ = SERVO_Z + (L_ARM_1 * sin(radialen.b));  //Z-coordinaat van elleboog B berekenen
}

//Functie voor het berekenen van het coördinaat van elleboog C voor een bepaald aantal graden
void elbowCCalc(int gradenC){
  radialen.c = gradenC * radOmreken;  //Waarde van servo C in graden omzetten naar radialen voor de goniometrische functies
  elbowCX = SERVO_C_X - (cos(rad60) * L_ARM_1 * cos(radialen.c)); //X-coordinaat van elleboog C berekenen
  elbowCY = SERVO_C_Y - (sin(rad60) * L_ARM_1 * cos(radialen.c)); //Y-coordinaat van elleboog C berekenen
  elbowCZ = SERVO_Z + (L_ARM_1 * sin(radialen.c));  //Z-coordinaat van elleboog C berekenen
}

//Functie om de afstand tussen een pols en een elleboog berekenen
float distanceCalc(float polsX, float polsY, float elbowX, float elbowY, float polsZ, float elbowZ){
  float distance = sqrt(pow((elbowX - polsX), 2) + pow((elbowY - polsY), 2) + pow((elbowZ - polsZ), 2));
  return distance;
}

//Functie voor het bepalen van de juiste waardes voor de servo's voor een gekozen coördinaat
void degreeCalc(int x, int y, int z){
  Serial.println("Pols berekenen");
  gradenMax = {MAX, MAX, MAX};  //Bovengrens instellen als maximale aantal graden
  gradenMin = {MIN, MIN, MIN};  //Ondergrens instellen als minimale aantal graden
  graden.a = (gradenMax.a + gradenMin.a) / 2;  //Begin waardes voor servo's instellen midden tussen de minimale en maximale waardes
  graden.b = (gradenMax.b + gradenMin.b) / 2;
  graden.c = (gradenMax.c + gradenMin.c) / 2;
  polsCalc(x, y, z);  //Coördinaten van de polsen berekenen
  elbowACalc(graden.a); //Coördinaten van elleboog A berekenen

  float distance = distanceCalc(polsAX, polsAY, elbowAX, elbowAY, polsZ, elbowAZ);  //Afstand tussen pols A en elleboog A berekenen
  Serial.print("Graden A = ");
  Serial.println(graden.a);
  Serial.print("Distance A = ");
  Serial.println(distance);

  while (L_ARM_2 - distance < -resolution || L_ARM_2 - distance > resolution){  //Controleren of de berekende afstand van de arm binnen de grenzen valt
    if(L_ARM_2 - distance < -resolution){ //De berekende afstand is groter dan de arm
      //gradenMax.a =  graden.a; //Bovengrens van aantal graden aanpassen naar momentele positie
      //graden.a = (gradenMax.a + gradenMin.a) / 2; //Waarde kiezen in het midden tussen de maximale en minimale waarde
      graden.a--;
      Serial.print("Graden A = ");
      Serial.println(graden.a);
      elbowACalc(graden.a); //Opnieuw coördinaten van elleboog A berekenen
      distance = distanceCalc(polsAX, polsAY, elbowAX, elbowAY, polsZ, elbowAZ);  //Opnieuw de afstand tussen pols A en elleboog A berekenen
      Serial.print("Distance A = ");
      Serial.println(distance);
    }else if(L_ARM_2 - distance > resolution){  //De berekende afstand is kleiner dan de arm
      //gradenMin.a = graden.a; //Ondergrens van aantal graden aanpassen naar momentele positie
      //graden.a = (gradenMax.a + gradenMin.a) / 2;  //Waarde kiezen in het midden tussen de maximale en minimale waarde
      graden.a++;
      elbowACalc(graden.a); //Opnieuw coördinaten van elleboog A berekenen
      Serial.print("Graden A = ");
      Serial.println(graden.a);
      distance = distanceCalc(polsAX, polsAY, elbowAX, elbowAY, polsZ, elbowAZ);  //Opnieuw de afstand tussen pols A en elleboog A berekenen
      Serial.print("Distance A = ");
      Serial.println(distance);
    }
  }
  Serial.print("Graden A = ");  //Waarde voor servo A printen als de juiste waarde is gevonden
  Serial.println(graden.a);
  #ifdef test
    delay(1000);
  #endif
  elbowBCalc(graden.b);
  distance = distanceCalc(polsBX, polsBY, elbowBX, elbowBY, polsZ, elbowBZ);
  Serial.println(distance);

  while (L_ARM_2 - distance < -resolution || L_ARM_2 - distance > resolution){
    if(L_ARM_2 - distance < -resolution){
      //gradenMin.b =  graden.b; //Bovengrens van aantal graden aanpassen naar momentele positie
      //graden.b = (gradenMax.b + gradenMin.b) / 2; //Waarde kiezen in het midden tussen de maximale en minimale waarde
      graden.b--;
      Serial.print("Graden B = ");
      Serial.println(graden.b);
      elbowBCalc(graden.b);
      distance = distanceCalc(polsBX, polsBY, elbowBX, elbowBY, polsZ, elbowBZ);
      Serial.print("Distance B = ");
      Serial.println(distance);
    }else if(L_ARM_2 - distance > resolution){
      //gradenMax.b = graden.b; //Ondergrens van aantal graden aanpassen naar momentele positie
      //graden.b = (gradenMax.b + gradenMin.b) / 2;  //Waarde kiezen in het midden tussen de maximale en minimale waarde
      graden.b++;
      elbowBCalc(graden.b);
      Serial.print("Graden B = ");
      Serial.println(graden.b);
      distance = distanceCalc(polsBX, polsBY, elbowBX, elbowBY, polsZ, elbowBZ);
      Serial.print("Distance B = ");
      Serial.println(distance);
    }
  }
  Serial.print("Graden B = ");
  Serial.println(graden.b);
  #ifdef test
    delay(1000);
  #endif
  elbowCCalc(graden.c);
  distance = distanceCalc(polsCX, polsCY, elbowCX, elbowCY, polsZ, elbowCZ);
  Serial.println(distance);

  while (L_ARM_2 - distance < -resolution || L_ARM_2 - distance > resolution){
    if(L_ARM_2 - distance < -resolution){
      //gradenMax.c =  graden.c; //Bovengrens van aantal graden aanpassen naar momentele positie
      //graden.c = (gradenMax.c + gradenMin.c) / 2; //Waarde kiezen in het midden tussen de maximale en minimale waarde
      graden.c--;
      Serial.print("Graden C = ");
      Serial.println(graden.c);
      elbowCCalc(graden.c);
      distance = distanceCalc(polsCX, polsCY, elbowCX, elbowCY, polsZ, elbowCZ);
      Serial.print("Distance C = ");
      Serial.println(distance);
    }else if(L_ARM_2 - distance > resolution){
      //gradenMin.c = graden.c; //Ondergrens van aantal graden aanpassen naar momentele positie
      //graden.c = (gradenMax.c + gradenMin.c) / 2;  //Waarde kiezen in het midden tussen de maximale en minimale waarde
      graden.c++;
      elbowCCalc(graden.c);
      Serial.print("Graden C = ");
      Serial.println(graden.c);
      distance = distanceCalc(polsCX, polsCY, elbowCX, elbowCY, polsZ, elbowCZ);
      Serial.print("Distance C = ");
      Serial.println(distance);
    }
  }
}

//Functie voor het maken van een beweging naar een coördinaat
void move(int x, int y, int z){
  Serial.println("Moving"); //Bericht printen om te laten weten dat de coordinaten berekend worden
  Serial.print("X: ");  //Alle coordinaten printen
  Serial.println(x);
  Serial.print("Y: ");
  Serial.println(y);
  Serial.print("Z: ");
  Serial.println(z);
  degreeCalc(x, y, z);  //Graden berekenen

  Serial.print("Graden A = ");  //Alle graden printen
  Serial.println(graden.a);
  Serial.print("Graden B = ");
  Serial.println(graden.b);
  Serial.print("Graden C = ");
  Serial.println(graden.c);

  if (graden.a >= MIN && graden.a <= MAX){  //Als de berekende waardes tussen de minimale en maximale waardes liggen, servo's bewegen
    servoA.write(graden.a);
  }
  if (graden.b >= MIN && graden.b <= MAX){
    servoB.write(graden.b);
  }
  if (graden.c >= MIN && graden.c <= MAX){
    servoC.write(graden.c);
  }
  Serial.println("Moved");  //Bericht printen om te laten weten dat alle servo's zijn bewogen
}

//Alle waardes uit alle functies printen
void debug(){
  graden.a = 133; //Begin waardes voor servo's instellen midden tussen de minimale en maximale waardes (85 en 180)
  graden.b = 133;
  graden.c = 133;
  servoA.write(graden.a);
  servoB.write(graden.b);
  servoC.write(graden.c);
  elbowACalc(180); //Coördinaten elleboog A berekenen
  elbowBCalc(180); //Coördinaten elleboog B berekenen
  elbowCCalc(180); //Coördinaten elleboog C berekenen
  polsCalc(0, 0, 100);
  float distanceA = distanceCalc(polsAX, polsAY, elbowAX, elbowAY, polsZ, elbowAZ);
  float distanceB = distanceCalc(polsBX, polsBY, elbowBX, elbowBY, polsZ, elbowBZ);
  float distanceC = distanceCalc(polsCX, polsCY, elbowCX, elbowCY, polsZ, elbowCZ);
  Serial.print("PolsAX: "); //Alle berekende waardes printen
  Serial.println(polsAX);
  Serial.print("PolsAY: ");
  Serial.println(polsAY);
  Serial.println();
  Serial.print("PolsBX: ");
  Serial.println(polsBX);
  Serial.print("PolsBY: ");
  Serial.println(polsBY);
  Serial.println();
  Serial.print("PolsCX: ");
  Serial.println(polsCX);
  Serial.print("PolsCY: ");
  Serial.println(polsCY);
  Serial.println();
  Serial.print("PolsZ: ");
  Serial.println(polsZ);
  Serial.println();
  Serial.print("elbowAX: ");
  Serial.println(elbowAX);
  Serial.print("elbowAY: ");
  Serial.println(elbowAY);
  Serial.print("elbowAZ: ");
  Serial.println(elbowAZ);
  Serial.println();
  Serial.print("elbowBX: ");
  Serial.println(elbowBX);
  Serial.print("elbowBY: ");
  Serial.println(elbowBY);
  Serial.print("elbowBZ: ");
  Serial.println(elbowBZ);
  Serial.println();
  Serial.print("elbowCX: ");
  Serial.println(elbowCX);
  Serial.print("elbowCY: ");
  Serial.println(elbowCY);
  Serial.print("elbowCZ: ");
  Serial.println(elbowCZ);
  Serial.println();
  Serial.print("Distance A: ");
  Serial.println(distanceA);
  Serial.print("Distance B: ");
  Serial.println(distanceB);
  Serial.print("Distance C: ");
  Serial.println(distanceC);
}

//Functie voor het openen en sluiten van de klauw
void jaw(bool close){
  static int currentDeg = openJawPos; //Klauw is aan het begin open, dus aantal graden is de open positie

  if(close){  //Close is waar, dus klauw moet sluiten
    while(!digitalRead(limSwitch)){ //Schakelaar voor het detecteren of de klauw gesloten is, is niet geactiveerd
      currentDeg =+ addDeg; //Graden toe laten nemen
      servoJaw.write(currentDeg); //Servo bewegen
    }
  }else if(!close){ //Close is niet waar, dus klauw moet openen 
    servoJaw.write(openJawPos); //Servo bewegen naar open positie
    currentDeg = openJawPos;  //Momentele aantal graden bijwerken naar de open positie
  }
}

//Functie voor het activeren en deactiveren van de solenoid
void solenoid(bool extend){
  if(extend){ //Extend is waar, dus solenoid moet activeerd worden
    digitalWrite(solPin, HIGH); //Solenoid pin aanzetten
  }else if(!extend){  //Extend is niet waar, dus solenoid moet gedeactiveerd worden
    digitalWrite(solPin, LOW);  //Solenoid pin uitzetten
  }
}

//Functie die gedraaid wordt aan bij het opstarten waarbij de te pakken bakjes worden geselecteerd
void startup(){
  lcd.clear();  //LCD leegmaken
  lcd.setCursor(0, 0);  //Cursor linksboven zetten
  lcd.write("Kies een bakje");

  while(!digitalRead(knopA) && !digitalRead(knopB) && !digitalRead(knopC) && !digitalRead(knopD)){} //Geen van de bakjes selectie knoppen zijn ingedrukt
  lcd.clear();

  if(digitalRead(knopA)){ //Bakje A is geselecteerd
    lcd.write("Medicatiebakje A");
    lcd.setCursor(0, 1);
    lcd.write("Druk op A");
    delay(500); //Delay om ervoor te zorgen dat knopA is losgelaten en niet meteen de while loop wordt genegeerd
    while(!digitalRead(knopA)){}
    actiefA = true; //Bakje A is gekozen
    selectPil('A'); //Tijden van bakje A instellen
  }else if(digitalRead(knopB)){ //Bakje B is geselecteerd
    lcd.write("Medicatiebakje B");
    lcd.setCursor(0, 1);
    lcd.write("Druk op A");
    while(!digitalRead(knopA)){}
    actiefB = true; //Bakje B is gekozen
    selectPil('B'); //Tijden van bakje B instellen
  }else if(digitalRead(knopC)){ //Bakje C is geselecteerd
    lcd.write("Medicatiebakje C");
    lcd.setCursor(0, 1);
    lcd.write("Druk op A");
    while(!digitalRead(knopA)){}
    actiefC = true; //Bakje C is gekozen
    selectPil('C'); //Tijden van bakje C instellen
  }else if(digitalRead(knopD)){ //Bakje D is geselecteerd
    lcd.write("Medicatiebakje D");
    lcd.setCursor(0, 1);
    lcd.write("Druk op A");
    while(!digitalRead(knopA)){}
    actiefD = true; //Bakje D is gekozen
    selectPil('D'); //Tijden van bakje D instellen
  }
}

//Functie voor het programmeren welke bakjes wanneer moeten worden gepakt
void selectPil(char bakje){
  char vorigeKnopUp;
  char vorigeKnopDown;
  char vorigeKnopEnter;
  char uur = 0;
  char minuut = 0;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.write("Tijd van inname");
  lcd.setCursor(0, 1);
  lcd.write(uur);
  lcd.setCursor(2,1);
  lcd.write(":");
  lcd.setCursor(3, 1);
  lcd.write(minuut);

  while(!digitalRead(knopA)){ //A indrukken om de uren in te gaan stellen
    lcd.setCursor(0, 1);
    lcd.blink();  //Cursor laten knipperen om aan te geven dat de uren zijn geselecteerd
    if(digitalRead(knopUp) && !vorigeKnopUp){
      uur++;  
      if(uur > 23){
        uur = 0;
      }
      lcd.write("  ");
      lcd.write(uur);
    }else if(digitalRead(knopDown) && vorigeKnopDown){
      uur--;
      if(uur < 0){
        uur = 23;
      }
      lcd.write("  ");
      lcd.write(uur);
    }
    vorigeKnopUp = digitalRead(knopUp);
    vorigeKnopDown = digitalRead(knopDown);
  }
  vorigeKnopEnter = digitalRead(knopA);

  while(!digitalRead(knopA) && !vorigeKnopEnter){ //A weer indrukken om de minuten in te gaan stellen
    lcd.setCursor(3, 1);
    lcd.blink();  //Cursor laten knipperen om aan te geven dat de minuten zijn geselecteerd
    if(digitalRead(knopUp)){
      minuut++;
      if(minuut > 59){
        minuut = 0;
      }
      lcd.write("  ");
      lcd.write(minuut);
    }else if(digitalRead(knopDown)){
      minuut--;
      if(minuut < 0){
        minuut = 59;
      }
      lcd.write("  ");
      lcd.write(minuut);
    }
  }

  switch(bakje){
    case 'A': //Als bakje A ingesteld moest  worden
      uurA = uur; //Uren van A instellen naar de bepaalde waarde
      minuutA = minuut; //Minuten van A instellen naar de bepaalde waarde
    break;

    case 'B': //Als bakje B ingesteld moest  worden
      uurB = uur; //Uren van B instellen naar de bepaalde waarde
      minuutB = minuut; //Minuten van B instellen naar de bepaalde waarde
    break;

    case 'C': //Als bakje C ingesteld moest  worden
      uurC = uur; //Uren van C instellen naar de bepaalde waarde
      minuutC = minuut; //Minuten van C instellen naar de bepaalde waarde
    break;

    case 'D': //Als bakje D ingesteld moest  worden
      uurD = uur; //Uren van D instellen naar de bepaalde waarde
      minuutD = minuut; //Minuten van D instellen naar de bepaalde waarde
    break;
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.write("Druk A als klaar");
  lcd.setCursor(0, 1);
  lcd.write("Anders op B");

  while(!digitalRead(knopA) && !digitalRead(knopB)){
    if(digitalRead(knopB)){ //Als knop B is ingedrukt kan nog een bakje ingesteld worden
      startup();
    }
  }
}

//Functie voor het pakken van het geselecteerde bakje
void pakBakje(char bakje){
  switch (bakje){
    case 'A': //Bakje A moet gepakt worden
      move(bakjeAX, bakjeAY, (bakjeAZ+50)); //Bewegen naar iets boven het bakje zodat niks wordt omgegooid
      move(bakjeAX, bakjeAY, bakjeAZ);  //Naar beneden bewegen
      jaw(true);  //Klauw sluiten
      delay(500); //Delay om zeker te weten dat de klauw dicht is
      move(bakjeAX, bakjeAY, (bakjeAZ+50)); //Bewegen naar iets boven het bakje voor het bewegen naar het uitgave bakje zodat niks wordt omgegooid
      move(uitgaveX, uitgaveY, uitgaveZ); //Bewegen naar het uitgave punt
      solenoid(true); //Solenoid activeren zodat een pil uit het bakje komt
      delay(100); //Delay om zeker te weten dat de solenoid de knop van het bakje ingedrukt heeft
      solenoid(false);  //Solenoid weer inklappen
      move(bakjeAX, bakjeAY, (bakjeAZ+50)); //Terug bewegen naar de plek van het bakje
      move(bakjeAX, bakjeAY, bakjeAZ);
      jaw(false); //Klauw openen
      delay(500); //Klauw de tijd geven om te openen
      move(bakjeAX, bakjeAY, (bakjeAZ+50)); //Omhoog bewegen om ervoor te zorgen dat niks omgegooid wordt
      move(homeX, homeY, homeZ);  //Terug naar het begin punt bewegen
      playSound();
    break;
    case 'B': //Bakje B moet gepakt worden
      move(bakjeBX, bakjeBY, (bakjeBZ+50));
      move(bakjeBX, bakjeBY, bakjeBZ);
      jaw(true);
      delay(500);
      move(bakjeBX, bakjeBY, (bakjeBZ+50));
      move(uitgaveX, uitgaveY, (uitgaveZ+50));
      move(uitgaveX, uitgaveY, uitgaveZ);
      solenoid(true);
      delay(100);
      solenoid(false);
      move(bakjeBX, bakjeBY, (bakjeBZ+50));
      move(bakjeBX, bakjeBY, bakjeBZ);
      jaw(false);
      delay(500);
      move(bakjeBX, bakjeBY, (bakjeBZ+50));
      move(homeX, homeY, homeZ);
      playSound();
    break;
    case 'C': //Bakje C moet gepakt worden
      move(bakjeCX, bakjeCY, (bakjeCZ+50));
      move(bakjeCX, bakjeCY, bakjeCZ);
      jaw(true);
      delay(500);
      move(bakjeCX, bakjeCY, (bakjeCZ+50));
      move(uitgaveX, uitgaveY, (uitgaveZ+50));
      move(uitgaveX, uitgaveY, uitgaveZ);
      solenoid(true);
      delay(100);
      solenoid(false);
      move(bakjeCX, bakjeCY, (bakjeCZ+50));
      move(bakjeCX, bakjeCY, bakjeCZ);
      jaw(false);
      delay(500);
      move(bakjeCX, bakjeCY, (bakjeCZ+50));
      move(homeX, homeY, homeZ);
      playSound();
    break;
    case 'D': //Bakje D moet gepakt worden
      move(bakjeDX, bakjeDY, (bakjeDZ+50));
      move(bakjeDX, bakjeDY, bakjeDZ);
      jaw(true);
      delay(500);
      move(bakjeDX, bakjeDY, (bakjeDZ+50));
      move(uitgaveX, uitgaveY, (uitgaveZ+50));
      move(uitgaveX, uitgaveY, uitgaveZ);
      solenoid(true);
      delay(100);
      solenoid(false);
      move(bakjeDX, bakjeDY, (bakjeDZ+50));
      move(bakjeDX, bakjeDY, bakjeDZ);
      jaw(false);
      delay(500);
      move(bakjeDX, bakjeDY, (bakjeDZ+50));
      move(homeX, homeY, homeZ);
      playSound();
    break;
  }
}

void playSound(){
  /*
    Deze functie is nog niet gemaakt omdat wij niet op tijd een manier hebben gevonden om dit op een goede manier te doen.
    Ook zouden we pinnen te kort komen op de Arduino Uno, dus zou een Mega gebruikt moeten worden.
  */
}