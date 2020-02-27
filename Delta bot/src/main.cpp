//Max 180 graden
//Min 85 graden

#include <Arduino.h>
#include <Servo.h>
#include "mydef.h"

//#define test

float polsAX = 0;
float polsAY = 0;
float polsBX = 0;
float polsBY = 0;
float polsCX = 0;
float polsCY = 0;
float polsZ = 0;

float elbowAX = 0;
float elbowAY = 0;
float elbowAZ = 0;
float elbowBX = 0;
float elbowBY = 0;
float elbowBZ = 0;
float elbowCX = 0;
float elbowCY = 0;
float elbowCZ = 0;

//Struct om de graden voor de servo's op te slaan
struct servos{
  int a;
  int b;
  int c;
};

servos graden;
servos vorigeGradenMax = {85, 85, 85}; //Hoogste aantal graden van de laatste waardes
servos vorigeGradenMin = {180, 180, 180}; //Laagste aantal graden van de laatste waardes

//Struct om de naar radialen omgerekende graden op te slaan
struct Radialen{
  float a;
  float b;
  float c;
};

Radialen radialen;

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
  #endif
  move(50, 50, 200);
}

void loop(){
}

//Functie voor het berekenen van de coördinaten van de polsen voor een gekozen coördinaat
void polsCalc(int x, int y, int z){
  polsAX = x + L_TOP;
  polsAY = y;

  polsZ = z;

  polsBX = x - (cos(rad60) * L_TOP);
  polsBY = y + (sin(rad60) * L_TOP);

  polsCX = x - (cos(rad60) * L_TOP);
  polsCY = y - (sin(rad60) * L_TOP);
}

//Functie voor het berekenen van het coördinaat van elleboog A voor een bepaald aantal graden
void elbowACalc(int gradenA){
  radialen.a = graden.a * radOmreken;
  elbowAX = SERVO_A_X + (L_ARM_1 * cos(radialen.a));
  elbowAY = SERVO_A_Y;
  elbowAZ = SERVO_Z + (L_ARM_1 * sin(radialen.a));
}

//Functie voor het berekenen van het coördinaat van elleboog B voor een bepaald aantal graden
void elbowBCalc(int gradenB){
  radialen.b = graden.b * radOmreken;
  elbowBX = SERVO_B_X - (cos(rad60) * L_ARM_1 * cos(radialen.b));
  elbowBY = SERVO_B_Y + (sin(rad60) * L_ARM_1 * cos(radialen.b));
  elbowBZ = SERVO_Z + (L_ARM_1 * sin(radialen.b));
}

//Functie voor het berekenen van het coördinaat van elleboog C voor een bepaald aantal graden
void elbowCCalc(int gradenC){
  radialen.c = graden.c * radOmreken;
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
  graden.a = (MAX + MIN) / 2;  //Begin waardes voor servo's instellen midden tussen de minimale en maximale waardes (85 en 180)
  graden.b = (MAX + MIN) / 2;
  graden.c = (MAX + MIN) / 2;
  polsCalc(x, y, z);  //Coördinaten van de polsen berekenen
  elbowACalc(graden.a); //Coördinaten van elleboog A berekenen

  float distance = distanceCalc(polsAX, polsAY, elbowAX, elbowAY, polsZ, elbowAZ);  //Afstand tussen pols A en elleboog A berekenen
  Serial.print("Graden A = ");
  Serial.println(graden.a);
  Serial.print("Distance A = ");
  Serial.println(distance);

  while (L_ARM_2 - distance < -resolution || L_ARM_2 - distance > resolution){  //Controleren of de berekende afstand van de arm binnen de grenzen valt
    if(L_ARM_2 - distance < -resolution){ //De berekende afstand is groter dan de arm
      graden.a = ;  //Graden kiezen in het midden tussen 0 en de vorige waarde
      Serial.print("Graden A = ");
      Serial.println(graden.a);
      elbowACalc(graden.a); //Opnieuw coördinaten van elleboog A berekenen
      distance = distanceCalc(polsAX, polsAY, elbowAX, elbowAY, polsZ, elbowAZ);  //Opnieuw de afstand tussen pols A en elleboog A berekenen
      Serial.print("Distance A = ");
      Serial.println(distance);
      delay(1000);
    }else if(L_ARM_2 - distance > resolution){  //De berekende afstand is kleiner dan de arm
      graden.a = (180 - graden.a)/2 + graden.a; //Graden kiezen in het midden tussen 180 en de vorige waarde
      elbowACalc(graden.a); //Opnieuw coördinaten van elleboog A berekenen
      Serial.print("Graden A = ");
      Serial.println(graden.a);
      distance = distanceCalc(polsAX, polsAY, elbowAX, elbowAY, polsZ, elbowAZ);  //Opnieuw de afstand tussen pols A en elleboog A berekenen
      Serial.print("Distance A = ");
      Serial.println(distance);
      delay(1000);
    }
  }
  Serial.print("Graden A = ");  //Waarde voor servo A printen als de juiste waarde is gevonden
  Serial.println(graden.a);
  delay(1000);
  elbowBCalc(graden.b);
  distance = distanceCalc(polsBX, polsBY, elbowBX, elbowBY, polsZ, elbowBZ);
  Serial.println(distance);

  while (L_ARM_2 - distance < -resolution || L_ARM_2 - distance > resolution){
    if(L_ARM_2 - distance < -resolution){
      graden.b = graden.b / 2;
      Serial.print("Graden B = ");
      Serial.println(graden.b);
      elbowACalc(graden.b);
      distance = distanceCalc(polsBX, polsBY, elbowBX, elbowBY, polsZ, elbowBZ);
      Serial.print("Distance B = ");
      Serial.println(distance);
      delay(1000);
    }else if(L_ARM_2 - distance > resolution){
      graden.b = (180 - graden.b)/2 + graden.b;
      elbowACalc(graden.b);
      Serial.print("Graden B = ");
      Serial.println(graden.b);
      distance = distanceCalc(polsBX, polsBY, elbowBX, elbowBY, polsZ, elbowBZ);
      Serial.print("Distance B = ");
      Serial.println(distance);
      delay(1000);
    }
  }
  Serial.print("Graden B = ");
  Serial.println(graden.b);
  delay(1000);
  elbowCCalc(graden.c);
  distance = distanceCalc(polsCX, polsCY, elbowCX, elbowCY, polsZ, elbowCZ);
  Serial.println(distance);

  while (L_ARM_2 - distance < -resolution || L_ARM_2 - distance > resolution){
    if(L_ARM_2 - distance < -resolution){
      graden.c = graden.c / 2;
      Serial.print("Graden C = ");
      Serial.println(graden.c);
      elbowACalc(graden.c);
      distance = distanceCalc(polsCX, polsCY, elbowCX, elbowCY, polsZ, elbowCZ);
      Serial.print("Distance C = ");
      Serial.println(distance);
      delay(1000);
    }else if(L_ARM_2 - distance > resolution){
      graden.c = (180 - graden.c)/2 + graden.c;
      elbowACalc(graden.c);
      Serial.print("Graden C = ");
      Serial.println(graden.c);
      distance = distanceCalc(polsCX, polsCY, elbowCX, elbowCY, polsZ, elbowCZ);
      Serial.print("Distance C = ");
      Serial.println(distance);
      delay(1000);
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

  /*if(graden.a < 0){
    graden.a = graden.a * -1;
  }
  if(graden.b < 0){
    graden.b = graden.b * -1;
  }
  if(graden.c < 0){
    graden.c = graden.c * -1;
  }

  graden.a = 180 - graden.a;
  graden.b = 180 - graden.b;
  graden.c = 180 - graden.c;

  if(graden.a < 0){
    graden.a = graden.a * -1;
  }
  if(graden.b < 0){
    graden.b = graden.b * -1;
  }
  if(graden.c < 0){
    graden.c = graden.c * -1;
  }*/

  Serial.print("Graden A = ");
  Serial.println(graden.a);
  Serial.print("Graden B = ");
  Serial.println(graden.b);
  Serial.print("Graden C = ");
  Serial.println(graden.c);

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
  elbowACalc(45); //Coördinaten elleboog A berekenen
  elbowBCalc(45); //Coördinaten elleboog B berekenen
  elbowCCalc(45); //Coördinaten elleboog C berekenen
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
}