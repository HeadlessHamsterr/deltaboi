#define servoPinA 12
#define servoPinB 11
#define servoPinC 10

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
servos gradenMax; //Hoogste aantal graden van de laatste waardes
servos gradenMin; //Laagste aantal graden van de laatste waardes

//Struct om de naar radialen omgerekende graden op te slaan
struct Radialen{
  float a;
  float b;
  float c;
};

Radialen radialen;