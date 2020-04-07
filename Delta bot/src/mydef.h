#define servoPinA 13
#define servoPinB 12
#define servoPinC 11
#define servoPinJaw 10
#define solPin 9
#define limSwitch 8
#define knopA A0
#define knopB A1
#define knopC A2
#define knopD A3
#define knopUp A4
#define knopDown A5

#define SERVO_A_X -110
#define SERVO_A_Y 0
#define SERVO_B_X 55
#define SERVO_B_Y 95
#define SERVO_C_X 55
#define SERVO_C_Y -95
#define SERVO_Z 29
#define Z_BASIS 294
#define L_ARM_1 45
#define L_ARM_2 218
#define L_TOP 32
#define L_BASE 100
#define Z_TOOL_OFFSET 22
#define MIN 0
#define MAX 360
#define resolution 10
#define rad60 1.0471975
#define radOmreken 0.0174532
#define testGraden 25
#define addDeg 5
#define openJawPos 0

#define bakjeAX 50
#define bakjeAY 0
#define bakjeAZ -150

#define bakjeBX 50
#define bakjeBY 20
#define bakjeBZ -150

#define bakjeCX 50
#define bakjeCY 40
#define bakjeCZ -150

#define bakjeDX 50
#define bakjeDY 60
#define bakjeDZ -150

#define uitgaveX -50
#define uitgaveY 30
#define uitgaveZ -150

#define homeX 0
#define homeY 20
#define homeZ 120

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
  int jaw;
};

servos graden;
servos gradenMax; //Hoogste aantal graden van de laatste waardes
servos gradenMin; //Laagste aantal graden van de laatste waardes

//Struct om de naar radialen omgerekende graden op te slaan
struct Radialen{
  float a;
  float b;
  float c;
};

Radialen radialen;