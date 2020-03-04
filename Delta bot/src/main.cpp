#include <Arduino.h>
#include <Servo.h>
#include "mydef.h"

//#define test

void polsCalc(int x, int y, int z);
void elbowACalc(int gradenA);
void elbowBCalc(int gradenB);
void elbowCCalc(int gradenC);
float distanceCalc(float polsX, float polsY, float elbowX, float elbowY, float polsZ, float elbowZ);
void degreeCalc(int x, int y, int z);
void move(int x, int y, int z);
void debug();

//Servo objecten aanmaken
Servo servoA;
Servo servoB;
Servo servoC;

void setup(){
  Serial.begin(9600);

  pinMode(servoPinA, OUTPUT);
  pinMode(servoPinB, OUTPUT);
  pinMode(servoPinC, OUTPUT);

  servoA.attach(servoPinA);
  servoB.attach(servoPinB);
  servoC.attach(servoPinC);

  #ifdef test
    debug();
    servoA.write(testGraden);
    servoB.write(testGraden);
    servoC.write(testGraden);
  #endif
  #ifndef test
    //move(-50, -50, 100);
    move(9, 7, -134);
  #endif
}

void loop(){
}

//Functie voor het berekenen van de coördinaten van de polsen voor een gekozen coördinaat
void polsCalc(int x, int y, int z){
  polsAX = x + L_TOP;
  polsAY = y;

  polsZ = z;//Z_BASIS - z - Z_TOOL_OFFSET;

  polsBX = x - (cos(rad60) * L_TOP);
  polsBY = y + (sin(rad60) * L_TOP);

  polsCX = x - (cos(rad60) * L_TOP);
  polsCY = y - (sin(rad60) * L_TOP);
}

//Functie voor het berekenen van het coördinaat van elleboog A voor een bepaald aantal graden
void elbowACalc(int gradenA){
  radialen.a = gradenA * radOmreken;
  elbowAX = SERVO_A_X + (L_ARM_1 * cos(radialen.a));
  elbowAY = SERVO_A_Y;
  elbowAZ = SERVO_Z + (L_ARM_1 * sin(radialen.a));
}

//Functie voor het berekenen van het coördinaat van elleboog B voor een bepaald aantal graden
void elbowBCalc(int gradenB){
  radialen.b = gradenB * radOmreken;
  elbowBX = SERVO_B_X - (cos(rad60) * L_ARM_1 * cos(radialen.b));
  elbowBY = SERVO_B_Y + (sin(rad60) * L_ARM_1 * cos(radialen.b));
  elbowBZ = SERVO_Z + (L_ARM_1 * sin(radialen.b));
}

//Functie voor het berekenen van het coördinaat van elleboog C voor een bepaald aantal graden
void elbowCCalc(int gradenC){
  radialen.c = gradenC * radOmreken;
  elbowCX = SERVO_C_X - (cos(rad60) * L_ARM_1 * cos(radialen.c));
  elbowCY = SERVO_C_Y - (sin(rad60) * L_ARM_1 * cos(radialen.c));
  elbowCZ = SERVO_Z + (L_ARM_1 * sin(radialen.c));
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
      gradenMax.a =  graden.a; //Bovengrens van aantal graden aanpassen naar momentele positie
      graden.a = (gradenMax.a + gradenMin.a) / 2; //Waarde kiezen in het midden tussen de maximale en minimale waarde
      //graden.a--;
      Serial.print("Graden A = ");
      Serial.println(graden.a);
      elbowACalc(graden.a); //Opnieuw coördinaten van elleboog A berekenen
      distance = distanceCalc(polsAX, polsAY, elbowAX, elbowAY, polsZ, elbowAZ);  //Opnieuw de afstand tussen pols A en elleboog A berekenen
      Serial.print("Distance A = ");
      Serial.println(distance);
    }else if(L_ARM_2 - distance > resolution){  //De berekende afstand is kleiner dan de arm
      gradenMin.a = graden.a; //Ondergrens van aantal graden aanpassen naar momentele positie
      graden.a = (gradenMax.a + gradenMin.a) / 2;  //Waarde kiezen in het midden tussen de maximale en minimale waarde
      //graden.a++;
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
  delay(100);
  elbowBCalc(graden.b);
  distance = distanceCalc(polsBX, polsBY, elbowBX, elbowBY, polsZ, elbowBZ);
  Serial.println(distance);

  while (L_ARM_2 - distance < -resolution || L_ARM_2 - distance > resolution){
    if(L_ARM_2 - distance < -resolution){
      gradenMin.b =  graden.b; //Bovengrens van aantal graden aanpassen naar momentele positie
      graden.b = (gradenMax.b + gradenMin.b) / 2; //Waarde kiezen in het midden tussen de maximale en minimale waarde
      //graden.b--;
      Serial.print("Graden B = ");
      Serial.println(graden.b);
      elbowBCalc(graden.b);
      distance = distanceCalc(polsBX, polsBY, elbowBX, elbowBY, polsZ, elbowBZ);
      Serial.print("Distance B = ");
      Serial.println(distance);
    }else if(L_ARM_2 - distance > resolution){
      gradenMax.b = graden.b; //Ondergrens van aantal graden aanpassen naar momentele positie
      graden.b = (gradenMax.b + gradenMin.b) / 2;  //Waarde kiezen in het midden tussen de maximale en minimale waarde
      //graden.b++;
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
  delay(100);
  elbowCCalc(graden.c);
  distance = distanceCalc(polsCX, polsCY, elbowCX, elbowCY, polsZ, elbowCZ);
  Serial.println(distance);

  while (L_ARM_2 - distance < -resolution || L_ARM_2 - distance > resolution){
    if(L_ARM_2 - distance < -resolution){
      gradenMax.c =  graden.c; //Bovengrens van aantal graden aanpassen naar momentele positie
      graden.c = (gradenMax.c + gradenMin.c) / 2; //Waarde kiezen in het midden tussen de maximale en minimale waarde
      //graden.c--;
      Serial.print("Graden C = ");
      Serial.println(graden.c);
      elbowCCalc(graden.c);
      distance = distanceCalc(polsCX, polsCY, elbowCX, elbowCY, polsZ, elbowCZ);
      Serial.print("Distance C = ");
      Serial.println(distance);
    }else if(L_ARM_2 - distance > resolution){
      gradenMin.c = graden.c; //Ondergrens van aantal graden aanpassen naar momentele positie
      graden.c = (gradenMax.c + gradenMin.c) / 2;  //Waarde kiezen in het midden tussen de maximale en minimale waarde
      //graden.c++;
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
  Serial.println("Moving");
  Serial.print("X: ");
  Serial.println(x);
  Serial.print("Y: ");
  Serial.println(y);
  Serial.print("Z: ");
  Serial.println(z);
  degreeCalc(x, y, z);

  Serial.print("Graden A = ");
  Serial.println(graden.a);
  Serial.print("Graden B = ");
  Serial.println(graden.b);
  Serial.print("Graden C = ");
  Serial.println(graden.c);
/*
  if(graden.a > 180){
    graden.a = graden.a - 180;
  }
  if(graden.b > 180){
    graden.b = graden.b - 180;
  }
  if(graden.c > 180){
    graden.c = graden.c - 180;
  }*/

  if (graden.a >= MIN && graden.a <= MAX){
    servoA.write(graden.a);
  }
  if (graden.b >= MIN && graden.b <= MAX){
    servoB.write(graden.b);
  }
  if (graden.c >= MIN && graden.c <= MAX){
    servoC.write(graden.c);
  }
  Serial.println("Moved");
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
  Serial.print("PolsAX: ");
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