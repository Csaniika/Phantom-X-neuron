#include <BioloidController.h>  
#include <stdio.h>  
#include <stdlib.h>
#include <math.h>
#include <time.h>

//macros
#define NR 6
#define PI 3.14159265359
#define SERVONR 18

//                 ID>           1       2         3         4         5         6        7        8        9         10        11        12        13       14        15        16        17        18
#define ID_to_LEG (int[]){18, COXA[3], COXA[0], FEMUR[3], FEMUR[0], TIBIA[3], TIBIA[0], COXA[5], COXA[2], FEMUR[5], FEMUR[2], TIBIA[5], TIBIA[2], COXA[4], COXA[1], FEMUR[4], FEMUR[1], TIBIA[4], TIBIA[1] }
//           NR>coxa+femur    1   2  3  4   5  6  1   2   3  4  5   6
#define LEG_to_ID (int[]){12, 2, 14, 8, 1, 13, 7, 4, 16, 10, 3, 15, 9}


// ################################################### PARAMETERS ################################################### PARAMETERS ################################################### 

 // ################################################
 
 
float k = -0.6;
float tau = 0.3;
float a = 0.4;
float dt = 0.04;

float Phi[6];
float dPhi[6];

//csatolasok labak kozott
const int K[6][6]  = { {0,1,0,1,0,0}, {1,0,1,0,1,0}, {0,1,0,0,0,1}, {1,0,0,0,1,0}, {0,1,0,1,0,1}, {0,0,1,0,1,0} };
//const int K[6][6]  = { {1,1,1,1,1,1}, {1,1,1,1,1,1}, {1,1,1,1,1,1}, {1,1,1,1,1,1}, {1,1,1,1,1,1}, {1,1,1,1,1,1} };


float threshold_coxa = PI/12;
float threshold_femur = PI/10;

int speed_coxa = 0; // 0-1023 !!! 
int speed_femur = 0;
int torq_coxa = 400;
int torq_femur = 800;

// ###############################################


// ### Min and max values for servos, from Pheonix ###
//                  id>   2    14   8     1    13   7
const int COXA_min[6] = {225, 233, 229, 222, 226, 223};
const int COXA_max[6] = {792, 789, 794, 790, 789, 788};
//                  id>   4    16   10    3    15   9
const int FEMUR_min[6] = {164, 157, 156, 159, 158, 159};
const int FEMUR_max[6] = {862, 860, 857, 855, 858, 859};

//Tibia-k fixek, id>   6    18   12   5    17   11
const int TIBIA[6] = {250, 250, 250, 774, 774, 774};
int COXA[6] = {450, 512, 573, 573, 512, 450};
int FEMUR[6] = {670, 670, 670, 350, 350, 350};


const int duration = 120; //seconds
float begin_time;
float t;

int COXA_MIN[6] = {};
int COXA_MAX[6] = {};
int FEMUR_MIN[6] = {};
int FEMUR_MAX[6] = {};


BioloidController bioloid = BioloidController(1000000);

int print_freq = 0;
int current_pose[12];
int temp_current_pose[12] = {450, 512, 573, 573, 512, 450, 670, 670, 670, 350, 350, 350};
int current_speed[12];

void setup(){
  Serial.begin(115200);
  
  //radio_serach();
  
  //Serial.println("Connection ok!");
  //Serial.flush();
  begin_time = millis();
  //voltage_test();
  MINMAX_values(1); //1 előre, -1 hátra
  Phi_0values();
  
  //set_delay_time(50);
  //set_speed();
  set_speed(speed_coxa, speed_femur); 
  set_max_torq(torq_coxa, torq_femur); 
  bioloid.setup(SERVONR);
  
  //read_parameters();
  /*
  Serial.println("###########################");
  Serial.print("k = ");Serial.print(k);Serial.print(", tau = ");Serial.print(tau);Serial.print(", a = ");Serial.print(a);Serial.print(", dt = ");Serial.println(dt);
  Serial.flush();
 */
  //startpose with macro from COXA,FEMUR,TIBIA
  for ( int i = 1; i < SERVONR + 1 ; i++){
    bioloid.setNextPose(i, (int)ID_to_LEG[i]);
  }
  bioloid.readPose();
  bioloid.interpolateSetup(1000);
  while(bioloid.interpolating > 0){
    bioloid.interpolateStep();
    delay(3);
  }
  
}

void loop(){
  t = micros();
  for (int i = 0; i < NR; i++){
    Phi[i] += euler(dt, i);
  }
  for (int i = 0; i < NR; i++){
    COXA[i] = COXA_values(i);
    FEMUR[i] = FEMUR_values(i);
  }
  
  if (millis()-begin_time > 4 * 1000){
    make_next_pose();
    bioloid.writePose();
  }
  
  for(int i = 0; i < 12; i++){
    current_pose[i] = ax12GetRegister(LEG_to_ID[i+1], AX_PRESENT_POSITION_L, 2);
    pose_check(i);
    temp_current_pose[i] = current_pose[i];
  }
  
  //if (print_freq % 2 == 0){
  Serial.print(dt, 3);  
  Serial.print(" ");
  print_current_pose();
  print_next_pose(NR, COXA, FEMUR);
  print_current_speed();
    //print_data(NR, COXA, FEMUR);
  //}
  //print_freq++;
  dt = (micros()-t)/1000000.0000;
  


}

// ################################################### FUNCTIONS ################################################### FUNCTIONS ################################################### FUNCTIONS ################################################### 

void print_next_pose(int length, int *vector1, int *vector2){
  //Serial.print((millis()- begin_time) / 1000.0 );
  //Serial.print(" ");
  for (int i = 0; i < length; i++){
    Serial.print(vector1[i]);
    Serial.print(" ");
  }
  for (int i = 0; i < length; i++){
    Serial.print(vector2[i]);
    Serial.print(" ");
  }
  //Serial.println();
}

void print_current_pose(){
  Serial.print((millis()- begin_time) / 1000.0 );
  Serial.print(" ");
  for(int i = 0; i < 12; i++){
    Serial.print(current_pose[i]);
    Serial.print(" ");
  }
  //Serial.println(" ");
  //Serial.flush();  
}

void print_current_speed(){
  for(int i=0; i < 12; i++){
    current_speed[i] = ax12GetRegister(LEG_to_ID[i+1], AX_PRESENT_SPEED_L, 2);
    if (current_speed[i] < 0){
      current_speed[i] = 0;
    }
    Serial.print(current_speed[i]);
    Serial.print(" ");
    }
    Serial.println();
}


void Phi_0values(){
  for (int i = 0; i < NR; i++){
    Phi[i] = acos(2 * (float)(COXA[i]-COXA_MIN[i]) / (float)(COXA_MAX[i]-COXA_MIN[i]) - 1 ) + 3 * PI / 2.00 ;   
  }
}

float equation(int i){
  float temp = 1 + a * cos(Phi[i]);
  float temp2 = 0;
  for (int j = 0; j < NR; j++)
  {
    if ( i != j)
        temp2 += k * K[i][j] * sin(Phi[j] - Phi[i]);
  }
  return (temp + temp2) / tau;
}

float euler(float dt, int i){
  float equation(int i);
  dPhi[i] = dt * equation(i);
  return dPhi[i];
}

int COXA_values(int i){
  //printf("float test> %f ", COXA_MIN[i] + (1 + cos(Phi[i] - 3 * PI / 2) / 2 * (COXA_MAX[i]-COXA_MIN[i])) );
  return COXA_MIN[i] + (1 + cos(Phi[i] - 3 * PI / 2)) * (COXA_MAX[i]-COXA_MIN[i]) / 2;
  //return COXA_MIN[i] + (1 + cos(Phi[i])) * (COXA_MAX[i]-COXA_MIN[i]) / 2;
}

int FEMUR_values(int i){
  return FEMUR_MIN[i] + (1 + cos(Phi[i])) * (FEMUR_MAX[i]-FEMUR_MIN[i]) / 2;
}


void set_delay_time(int time){
 for ( int i = 1; i < SERVONR + 1 ; i++){
    ax12SetRegister(i, AX_RETURN_DELAY_TIME, time);
  }
 delay(1000);
 for ( int i = 1; i < SERVONR + 1 ; i++){
    Serial.println(ax12GetRegister(i, AX_RETURN_DELAY_TIME, 1));
  }
}

void set_max_torq(int tc, int tf){
  for ( int i = 1; i < NR + 1 ; i++){
    ax12SetRegister2(LEG_to_ID[i], AX_TORQUE_LIMIT_L, tc);
    delay(10);
  }
  for ( int i = 7; i < NR + 7 ; i++){
    ax12SetRegister2(LEG_to_ID[i], AX_TORQUE_LIMIT_L, tf);
    delay(10);
  }
 /*
 for ( int i = 1; i < SERVONR + 1 ; i++){
    Serial.println(ax12GetRegister(i, AX_TORQUE_LIMIT_L, 2));
  }
  */
}

void set_speed(int sc, int sf){
 for ( int i = 1; i < NR + 1 ; i++){
    ax12SetRegister2(LEG_to_ID[i], AX_GOAL_SPEED_L, sc);
    delay(10);
  }
  for ( int i = 7; i < NR + 7 ; i++){
    ax12SetRegister2(LEG_to_ID[i], AX_GOAL_SPEED_L, sf);
    delay(10);
  }
 /*
 for ( int i = 1; i < SERVONR + 1 ; i++){
    Serial.println(ax12GetRegister(i, AX_GOAL_SPEED_L, 2));
  }*/
}


void MINMAX_values(int dir){
    threshold_coxa = abs(rad_to_servo(threshold_coxa));
    threshold_femur = abs(rad_to_servo(threshold_femur));
    for (int i = 0; i < NR; i++){
        if (COXA[i] - threshold_coxa > COXA_min[i])
            COXA_MIN[i] = COXA[i] - threshold_coxa;
        else
            COXA_MIN[i] = COXA_min[i];
    
        if (COXA[i] + threshold_coxa < COXA_max[i])
            COXA_MAX[i] = COXA[i] + threshold_coxa;
        else
            COXA_MAX[i] = COXA_max[i];
    
        if (FEMUR[i] + threshold_femur < FEMUR_max[i])
            FEMUR_MAX[i] = FEMUR[i] + threshold_femur;
        else
            FEMUR_MAX[i] = FEMUR_max[i];
        
        if (FEMUR[i] - threshold_femur > FEMUR_min[i])
            FEMUR_MIN[i] = FEMUR[i] - threshold_femur;
        else
            FEMUR_MIN[i] = FEMUR_min[i];
    }
    //robot kétoldal tükrözése
        
    int minmax_temp;
    for (int i = 0; i < NR - 3 ; i++){
        minmax_temp = FEMUR_MIN[i];
        FEMUR_MIN[i] = FEMUR_MAX[i];
        FEMUR_MAX[i] = minmax_temp;
    }
    if (dir == 1){
        for (int i = 3; i < NR ; i++){
        minmax_temp = COXA_MIN[i];
        COXA_MIN[i] = COXA_MAX[i];
        COXA_MAX[i] = minmax_temp;
        }
    }
    if (dir == -1){
        for (int i = 0; i < NR -3 ; i++){
        minmax_temp = COXA_MIN[i];
        COXA_MIN[i] = COXA_MAX[i];
        COXA_MAX[i] = minmax_temp;
        }
    }
}


void pose_check(int i){
    if (i < 3){
      if(current_pose[i] < COXA_MIN[i] || current_pose[i] > COXA_MAX[i]){
        current_pose[i] = temp_current_pose[i];
      }
    }
    if (i < 6 && i >= 3){
      if(current_pose[i] > COXA_MIN[i] || current_pose[i] < COXA_MAX[i]){
        current_pose[i] = temp_current_pose[i];
      }
    }
    if (i < 9 && i >= 6 ){
      if (current_pose[i] > FEMUR_MIN[i - 6] || current_pose[i] < FEMUR_MAX[i - 6]){
        current_pose[i] = temp_current_pose[i];
        //Serial.println("####");
      }
    }
    if (i >= 9 ){
      if (current_pose[i] < FEMUR_MIN[i - 6] || current_pose[i] > FEMUR_MAX[i - 6]){
        current_pose[i] = temp_current_pose[i];
        //Serial.println("####");
      }
    }
}

void make_next_pose(){
  //bioloid konyvtarban uj fuggveny, hogy gyorsabb legyen
  for ( int i = 1; i < SERVONR + 1 ; i++){
    bioloid.setNextPose2(i, (int)ID_to_LEG[i]);
  }
}

int rad_to_servo(float rads){
  float val = (rads*100)/51 * 100;
  return (int) val;
}


void radio_serach(){
  int incomingByte = 0;
  while (incomingByte == 0){
    if (Serial.available() > 0) {
        incomingByte = Serial.read();
    }
  }
}
  

void voltage_test(){
  float voltage = (ax12GetRegister (1, AX_PRESENT_VOLTAGE, 1)) / 10.0;
  Serial.print ("System Voltage: ");
  Serial.print (voltage);
  Serial.println (" volts.");
  if (voltage < 10.0)
    while(1);
  Serial.flush();
  
}

void read_parameters(){
  Serial.println("Set parameters:");
  delay(1000);
  Serial.flush();
  k = 0;
  tau = 0;
  a = 0;
  dt = 0;

  Serial.print("tau = ");
  delay(500);
  while(tau == 0){
    if (Serial.available() > 0) {
       tau = Serial.parseFloat();
    }
  }
  
  Serial.print("k = ");
  while(k == 0){
    if (Serial.available() > 0) {
       k = -Serial.parseFloat();
    }
  }

  Serial.print("a = ");
  while(a == 0){
    if (Serial.available() > 0) {
       a = Serial.parseFloat();
    }
  }
  Serial.print("dt = ");
  while(dt == 0){
    if (Serial.available() > 0) {
       dt = Serial.parseFloat();
    }
  }
}







