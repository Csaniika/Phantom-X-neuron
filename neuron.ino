#include <stdio.h>  
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <BioloidController.h>  


#define NR 6
#define PI 3.14159265359
#define SERVONR 18

//                 ID>           1       2         3         4         5         6        7        8        9         10        11        12        13       14        15        16        17        18
#define ID_to_LEG (int[]){18, COXA[3], COXA[0], FEMUR[3], FEMUR[0], TIBIA[3], TIBIA[0], COXA[5], COXA[2], FEMUR[5], FEMUR[2], TIBIA[5], TIBIA[2], COXA[4], COXA[1], FEMUR[4], FEMUR[1], TIBIA[4], TIBIA[1] }
//           NR>coxa+femur    1   2  3  4   5  6  1   2   3  4  5   6
#define LEG_to_ID (int[]){18, 2, 14, 8, 1, 13, 7, 4, 16, 10, 3, 15, 9, 6, 18, 12, 5, 17, 11}

BioloidController bioloid = BioloidController(1000000);



// ################################################### PARAMETERS ################################################### PARAMETERS ################################################### 


const float tau_x[2] = {0.05, 0.025} ;
const float tau_b = 0.5 ;
const float a = 2.0;
float dt = 0.06;
const float w_k = -0.4;

float w_y = 2.5;
float w_s = 0.0;

float x[2][6] = {{2.0, -2.0,  2.0, -2.0,  2.0, -2.0},
                {-1.0,  1.0, -1.0,  1.0, -1.0,  1.0}};
float b[2][6] = {{4.0, 0.0, 4.0, 0.0, 4.0, 0.0},
                { 1.0, 4.0, 1.0, 4.0, 1.0, 4.0}};
/*
float x[2][6] = {{2.20, -4.95, -3.20, -1.85, -3.15, -4.60},
                 {-4.40, -4.00, 3.05, -2.25, -2.10, 3.50}}; 
float b[2][6] = {{4.20, -4.25, -3.85, 3.65, 4.35, 3.85},
                  {2.40, -0.80, 4.40, 4.95, -1.15, -3.90}}; 

float x[2][6] = {{4.80, 0.40, -0.05, 2.45, -0.20, 0.60},
                { 2.25, -4.60, -0.40, -4.70, 0.90, 2.85}}; 
float b[2][6] = {{-4.15, 2.80, -3.60, -2.05, 0.70, -0.70},
                {-2.30, 4.90, -2.75, 0.05, -4.75, -2.95}}; 
  */                
//moving thresholds both dirrections in radian
float threshold_coxa = PI/12;
float threshold_femur = PI/10;

int speed_coxa = 0; 
int speed_femur = 0;
int torq_coxa = 400;
int torq_femur = 800;
int torq_tibia = 800;


// ###############################################

// ### Min and max values for servos, from Pheonix ###
//                  id>   2    14   8     1    13   7
const int COXA_min[6] = {225, 230, 225, 225, 230, 225};
const int COXA_max[6] = {790, 790, 790, 790, 790, 790};
//                  id>   4    16   10    3    15   9
const int FEMUR_min[6] = {160, 160, 160, 160, 160, 160};
const int FEMUR_max[6] = {860, 860, 860, 860, 860, 860};

//
const int TIBIA[6] = {250, 250, 250, 774, 774, 774};
int COXA[6] = {450, 512, 573, 573, 512, 450};
int FEMUR[6] = {670, 670, 670, 350, 350, 350};

/*
// START POSE
//Tibia-k fixek, id>   6    18   12   5    17   11
const int TIBIA[6] = {512, 512, 512, 512, 512, 512};
int COXA[6] = {512, 512, 512, 512, 512, 512};
int FEMUR[6] = {512, 512, 512, 512, 512, 512};
*/
// ################################################### GLOBALVARIABLES ################################################### GLOBALVARIABLES ################################################### 
int COXA_MIN[6];
int COXA_MAX[6];
int FEMUR_MIN[6];
int FEMUR_MAX[6];

float dx[2][6];
float db[2][6];


unsigned long t, begin_time;
int print_freq = 0;
int current_pose[12];
int temp_current_pose[12] = {450, 512, 573, 573, 512, 450, 670, 670, 670, 350, 350, 350};
int current_speed[12];
int current_torq[12];

void setup(){
  Serial.begin(115200);
  
  //radio_search();
  //Serial.println("Connection ok!");Serial.flush();
  begin_time = millis();
  //voltage_test();
  MINMAX_values(1); //1 előre, -1 hátra neuronnal
  
  //set_delay_time(50); //ROM memory, egyszer kell megivni

  set_speed(speed_coxa, speed_femur); 
  set_max_torq(torq_coxa, torq_femur, torq_tibia); 
  bioloid.setup(SERVONR);
  read_parameters();
  
  
  //Serial.println("###########################");
  //Serial.print("k = ");Serial.print(k);Serial.print(", tau = ");Serial.print(tau);Serial.print(", a = ");Serial.print(a);Serial.print(", dt = ");Serial.println(dt);Serial.flush();
   
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
  euler(dt);
  COXA_FEMUR_values();
  if (millis()-begin_time > 3 * 1000){
    make_next_pose();
    bioloid.writePose();
  }
  for(int i = 0; i < 12; i++){
    current_pose[i] = ax12GetRegister(LEG_to_ID[i+1], AX_PRESENT_POSITION_L, 2);
    pose_check(i);
    temp_current_pose[i] = current_pose[i];
  }
  
  //print_current_pose();
  //print_current_speed();
  //if (print_freq % 2 == 0){
    Serial.print(dt, 3);  
    Serial.print(" ");
    print_current_pose();
    print_next_pose(NR, COXA, FEMUR);
    print_current_speed();
    //print_current_torq();
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

void print_current_torq(){
    for(int i=0; i < 12; i++){
    current_torq[i] = ax12GetRegister(LEG_to_ID[i+1], AX_PRESENT_LOAD_L, 2);
    Serial.print(current_torq[i]);
    Serial.print(" ");
    }
    Serial.println();
}


float y(float x, float b){
    return  1.0 / (1.0 + exp(a * (b - x)));
}

float s(int coxfem, int i){
    if (coxfem == 0)
      return (float)(current_pose[i] - COXA_MIN[i]) / (float)(COXA_MAX[i]-COXA_MIN[i]);
    if (coxfem == 1)
      return (float)(current_pose[i + 6] - FEMUR_MIN[i]) / (float)(FEMUR_MAX[i] - FEMUR_MIN[i]);
}

float equation1(int i, int coxfem){
    float temp0 = -x[coxfem][i];
    float temp1 = 0;
    float temp2 = 0;
    if (coxfem == 0){
        for (int j = 0; j < NR; j++){
          if (i != j){
            temp1 += y(x[0][j], b[0][j]);
          }
        }
        temp1 *= w_k ;
        temp2 = w_y * y(x[0][i], b[0][i]) + w_s * s(0, i);
    }
    else{
      temp1 = w_k * y(x[0][i], b[0][i])  ;
      temp2 = w_y * y(x[1][i], b[1][i]) + w_s * s(1, i) ; // 
    }
    return (temp0 + temp1 + temp2) / tau_x[coxfem];
}

float equation2(int i, int coxfem){
    return (y(x[coxfem][i], b[coxfem][i]) - 0.5) / tau_b;
}

float euler(float dt){
    for (int coxfem = 0; coxfem < 2; coxfem++){
        for (int i = 0; i < NR; i++){
            dx[coxfem][i] = dt * equation1(i, coxfem);
            db[coxfem][i] = dt * equation2(i, coxfem);
        }
    }
    for (int coxfem = 0; coxfem < 2; coxfem++){
        for (int i = 0; i < NR; i++){
            x[coxfem][i] += dx[coxfem][i];
            b[coxfem][i] += db[coxfem][i];
        }
    }
}

void COXA_FEMUR_values(){
    for (int i = 0; i < NR; i++){
        COXA[i] = COXA_MIN[i] + y(x[0][i], b[0][i]) * (COXA_MAX[i]-COXA_MIN[i]);
        FEMUR[i] = FEMUR_MIN[i] + y(x[1][i], b[1][i]) * (FEMUR_MAX[i]-FEMUR_MIN[i]);
        //printf("%f \n", y(x[0][i], b[0][i]));
        //printf("%f \n",y(x[1][i], b[1][i]));
    }
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

void set_max_torq(int tc, int tf, int tt){
  for ( int i = 1; i < NR + 1 ; i++){
    ax12SetRegister2(LEG_to_ID[i], AX_TORQUE_LIMIT_L, tc);
    delay(10);
  }
  for ( int i = 7; i < NR + 7 ; i++){
    ax12SetRegister2(LEG_to_ID[i], AX_TORQUE_LIMIT_L, tf);
    delay(10);
  }
  for ( int i = 13; i < NR + 13 ; i++){
    ax12SetRegister2(LEG_to_ID[i], AX_TORQUE_LIMIT_L, tt);
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
    for (int i = 0; i < NR - 3; i++){
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

void radio_search(){
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
  Serial.println();
  Serial.println("##################################");
  Serial.println("Set parameters:");
  w_y = -50.0;
  w_s = -50.0;
  int bi_s = -50;
  Serial.print("w_y = ");
  while(w_y == -50.0){
    if (Serial.available() > 0) {
       w_y = Serial.parseFloat();
    }
  }
  Serial.println(w_y);
  Serial.print("w_s = ");
  while(w_s == -50.0){
    if (Serial.available() > 0) {
       w_s = Serial.parseFloat();
    }
  }
  Serial.println(w_s);
  Serial.print("random startparam? = ");
  while(bi_s == -50.0){
    if (Serial.available() > 0) {
       bi_s = Serial.parseInt();
    }
  }
  randomSeed(micros() * 10.5);
  Serial.print(bi_s);
  Serial.println();
  if (bi_s == 1){
    random_start_param();
  }
}

void  random_start_param(){
  for ( int i = 0; i < 2 ; i++){
    for ( int j = 0; j < NR ; j++){
      x[i][j] = random(-100, 100) / 20.0;
      b[i][j] = random(-100, 100) / 20.0;
    }
  }
  for ( int i = 0; i < 2 ; i++){
    for ( int j = 0; j < NR ; j++){
      Serial.print(x[i][j]);
      Serial.print(' ');
    }
  }
  Serial.println();
  for ( int i = 0; i < 2 ; i++){
    for ( int j = 0; j < NR ; j++){
      Serial.print(b[i][j]);
      Serial.print(' ');
    }
  }
  Serial.println();
  
}

/*
const float k = 1;
const float tau_x = 0.2 ;
const float tau_b = 2;
const float tau_s = 1;
const float a = 4;
float dt = 1e-5;
const float w_y = 4.0;
const float w_s = 4.0;

float x[2][6] = {{2.0, -2.0, 2.0, -2.0, 2.0, -2.0},{-1.0, 1.0, -1.0, 1.0, -1.0, 1.0}};
float b[2][6] = {{4.0, 0.0, 4.0, 0.0, 4.0, 0.0},{1.0, 4.0, 1.0, 4.0, 1.0, 4.0}};

//csatolasok labak kozott
//const int K[6][6]  = { {1,1,1,1,1,1}, {1,1,1,1,1,1}, {1,1,1,1,1,1}, {1,1,1,1,1,1}, {1,1,1,1,1,1}, {1,1,1,1,1,1} };


const int K[6][6]  = {{ 1, -1,  0, -1,  0,  0},
                     { -1,  1, -1,  0, -1,  0},
                     {  0, -1,  1,  0,  0, -1},
                     { -1,  0,  0,  1, -1,  0},
                     {  0, -1,  0, -1,  1, -1},
                     {  0,  0, -1,  0, -1,  1}};


/*
const float K[6][6]  = {{1.0,   -1/5.0, -1/5.0, -1/5.0, -1/5.0, -1/5.0}, 
                       {-1/5.0,  1.0,   -1/5.0, -1/5.0, -1/5.0, -1/5.0},
                       {-1/5.0, -1/5.0,  1.0,   -1/5.0, -1/5.0, -1/5.0},
                       {-1/5.0, -1/5.0, -1/5.0,  1.0,   -1/5.0, -1/5.0},
                       {-1/5.0, -1/5.0, -1/5.0, -1/5.0,  1.0,   -1/5.0},
                       {-1/5.0, -1/5.0, -1/5.0, -1/5.0, -1/5.0,  1.0 }};
                    
const float K[6][6]  = {{1.0, kn,  kn,  kn,  kn,  kn}, 
                        {kn,  1.0, kn,  kn,  kn,  kn}, 
                        {kn,  kn,  1.0, kn,  kn,  kn}, 
                        {kn,  kn,  kn,  1.0, kn,  kn}, 
                        {kn,  kn,  kn,  kn,  1.0, kn}, 
                        {kn,  kn,  kn,  kn,  kn,  1.0}};
 */  

