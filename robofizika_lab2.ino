#include <BioloidController.h>  

BioloidController bioloid = BioloidController(1000000);
// set smart Servo baud rate + init Bioloid > az okos szervókkal 
// történö kommunikáció sebességének beálítása  (bit/s) + Biloloid inicializációja

#define LEG_to_ID (int[]){18, 2, 14, 8, 1, 13, 7, 4, 16, 10, 3, 15, 9, 6, 18, 12, 5, 17, 11}

int mins[] = {222, 225, 159, 164, 279, 158, 223, 229, 159, 156, 272, 155, 226, 233, 158, 157, 271, 157};
int maxs[] = {790, 792, 855, 862, 857, 747, 788, 794, 859, 857, 860, 747, 789, 789, 858, 860, 859, 743};


// pointer to the smart servo position vector
// A csuklók szögeit tartalmazó vektorra mutató pointer
//          Szervó ID-k            nr.  1    2    3    4    5    6    7    8    9    10   11  12   13   14   15   16   17   18
const PROGMEM uint16_t center[] = {18, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512 ,512, 512, 512, 512, 512, 512};
const PROGMEM uint16_t pose0[] =  {18, 573, 449, 362, 660, 789,	231, 448, 573, 363, 661, 792, 233, 509, 511, 359, 661, 804, 218};

const PROGMEM uint16_t pose1[] =  {18, 773, 649, 362, 660, 789,	231, 648, 773, 363, 661, 792, 233, 709, 711, 359, 661, 804, 218};
const PROGMEM uint16_t pose2[] =  {18, 691, 449, 624, 660, 662,	231, 267, 573, 675, 661, 602, 233, 453, 514, 590, 401, 675, 343};
const PROGMEM uint16_t pose3[] =  {18, 360, 449, 308, 660, 489,	231, 448, 366, 363, 701, 792, 559, 509, 511, 359, 661, 804, 218};
const PROGMEM uint16_t pose4[] =  {18, 360, 449, 308, 660, 489,	231, 448, 573, 363, 661, 792, 233, 509, 511, 359, 661, 804, 218};

const PROGMEM uint16_t move1[] =  {18, 674, 350, 477, 515, 687, 366, 362, 653, 504, 523, 655, 349, 512, 512, 478, 552, 681, 327};      
const PROGMEM uint16_t move2[] =  {18, 628, 320, 399, 479, 755, 398, 334, 642, 393, 493, 759, 385, 549, 542, 532, 629, 631, 260};   
const PROGMEM uint16_t move3[] =  {18, 635, 331, 546, 620, 616, 270, 337, 642, 537, 626, 628, 270, 543, 547, 403, 488, 749, 389};   

const PROGMEM uint16_t emel1[] =  {18, 657, 361, 458, 493, 709, 377, 359, 660, 456, 500, 710, 375, 523, 514, 520, 579, 651, 293};      
const PROGMEM uint16_t lep1[] =  {18, 630, 325, 455, 505, 701, 369, 329, 636, 470, 505, 692, 372, 548, 547, 522, 556, 643, 322};   
const PROGMEM uint16_t emel2[] =  {18, 628, 314, 512, 573, 645, 303, 329, 636, 538, 563, 630, 316, 547, 545, 452, 498, 713, 372};   
const PROGMEM uint16_t lep2[] =  {18, 705, 387, 523, 564, 635, 316, 405, 700, 525, 554, 640, 332, 472, 483, 456, 514, 710, 355};   
const PROGMEM uint16_t emel3[] =  {18, 697, 387, 468, 492, 696, 382, 393, 701, 459, 494, 709, 391, 483, 483, 528, 560, 637, 316};      

const PROGMEM uint16_t csop11[] =  {18, 657, 361, 518, 508, 627, 375, 364, 671, 524, 503, 635, 383, 514, 513, 515, 503, 638, 385};      
const PROGMEM uint16_t csop12[] =  {18, 667, 355, 470, 501, 679, 391, 368, 670, 466, 493, 691, 404, 521, 510, 521, 560, 631, 329};   
const PROGMEM uint16_t csop13[] =  {18, 628, 329, 462, 500, 683, 395, 338, 645, 466, 498, 692, 397, 541, 548, 528, 559, 621, 328};   
const PROGMEM uint16_t csop14[] =  {18, 632, 334, 530, 491, 608, 397, 330, 646, 539, 491, 618, 404, 539, 537, 513, 511, 638, 372};   
const PROGMEM uint16_t csop15[] =  {18, 629, 331, 523, 561, 627, 334, 328, 641, 538, 562, 615, 336, 540, 546, 451, 511, 704, 370}; 
const PROGMEM uint16_t csop16[] =  {18, 661, 362, 533, 557, 608, 330, 367, 672, 521, 545, 638, 343, 510, 515, 467, 514, 688, 370};  
const PROGMEM uint16_t csop17[] =  {18, 656, 351, 518, 487, 632, 402, 355, 667, 535, 488, 621, 410, 519, 518, 510, 509, 653, 377};  

const PROGMEM uint16_t csop21[] =  {18, 618, 310, 557, 458, 658, 360, 303, 624, 569, 464, 659, 357, 559, 576, 552, 470, 675, 348};      
const PROGMEM uint16_t csop22[] =  {18, 618, 312, 849, 470, 252, 346, 318, 623, 838, 468, 280, 353, 562, 568, 543, 189, 685, 739};   
const PROGMEM uint16_t csop23[] =  {18, 711, 413, 821, 472, 296, 340, 412, 726, 811, 453, 320, 374, 472, 470, 547, 202, 680, 715};   
const PROGMEM uint16_t csop24[] =  {18, 707, 401, 566, 478, 649, 338, 405, 711, 558, 457, 669, 364, 473, 471, 556, 471, 670, 342};   
const PROGMEM uint16_t csop25[] =  {18, 704, 409, 556, 164, 662, 779, 407, 721, 552, 153, 676, 790, 462, 471, 678, 474, 516, 339}; 
const PROGMEM uint16_t csop26[] =  {18, 613, 312, 568, 314, 646, 550, 318, 619, 559, 155, 668, 786, 560, 562, 849, 482, 259, 330};  

unsigned long t, begin_time;
int print_freq = 0;
int current_pose[18];



int temp[] = {674, 477, 687, 512, 478, 681, 362, 504, 655, 653, 523, 349, 512, 552, 327, 350, 515, 366};
int mapid[] = {1,   3,   5,   13,  15, 17,   7,   9,   11,  8,   10,  12,  14,  16,  18,  2,  4,    6};
// 1    3    5    13    15   17  7    9    11   8    10    12  14    16   18  2    4    6  
//674, 477, 687, 512, 478, 681, 362, 504, 655, 653, 523, 349, 512, 552, 327, 350, 515, 366
// a hexapod csuklóinak szögét tartlalmazó vektor
// elsö paraméter a beálítandó csuklók száma
// rendre a hexapod csuklóinak szögei adimenzionális egységben
// (a fok-ot vagy a radiánt dimenziónak tekintem, ha ezt kifogásoljátok
// akkor tekintsétek ugy, hogy egy átsálázott szög)

void setup()
{
  bioloid.setup(18);
  // csuklók számának megadása
  begin_time = millis();
  Serial.begin(115200);
  // Baud rate for serial communication
  // soros kommunikáció sebesége (bit/s) (baud rate > bit ráta)
  
  // dinamic memory allocation for joint angles
  // dinamikusan lefoglalt tömb a csuklószögeknek (a szögek nem fokban vannak hanem 
  // adimenzionális egységben > ez 0,1023 között vehet fel értéket)
  //bioloid.relaxServos();
  // szervok szög tartásának kikapcsolása > ez annyit jelent, hogy a szervó nem
  // fejt ki ellenállást a pillanatnyi szögének megtartása céljából
  
  delay(1000);
}

void loop()
{    
    int time = 1500;
    bioloid.loadPose(csop21);        
    bioloid.readPose();          // read in current servo positions to the curPose buffer
    // puffer kiolvasása
    bioloid.interpolateSetup(time);  // setup for interpolation from current->next over 2 second
    // szögek graduális változtatása a pillanatnyi pozicióból a cél pozicióba 1 másodperc alatt
    // addíg iteráljuk a következö lépéseket ameddig el nem érjük a cél poziciót
    while(bioloid.interpolating > 0) // do this while we have not reached our new pose
    {  
        bioloid.interpolateStep();     // move servos, if necessary.
        for(int i = 0; i < 18; i++){
          current_pose[i] = ax12GetRegister(LEG_to_ID[i+1], AX_PRESENT_POSITION_L, 2);
        }
        Serial.print(0);  
        Serial.print(" ");
        print_current_pose();
    }

    
    bioloid.loadPose(csop22);        // load the pose from FLASH, into the nextPose buffer
    // beálitandó szög betöltése a pufferbe
    bioloid.readPose();          // read in current servo positions to the curPose buffer
    // puffer kiolvasása
    bioloid.interpolateSetup(time);  // setup for interpolation from current->next over 1 second
    // szögek graduális változtatása a pillanatnyi pozicióból a cél pozicióba 1 másodperc alatt
    // addíg iteráljuk a következö lépéseket ameddig el nem érjük a cél poziciót
    while(bioloid.interpolating > 0) // do this while we have not reached our new pose
    {  
        bioloid.interpolateStep();     // move servos, if necessary.
        for(int i = 0; i < 18; i++){
          current_pose[i] = ax12GetRegister(LEG_to_ID[i+1], AX_PRESENT_POSITION_L, 2);
        }
        Serial.print(0);  
        Serial.print(" ");
        print_current_pose();
      }
      
    bioloid.loadPose(csop23);        // load the pose from FLASH, into the nextPose buffer
    // beálitandó szög betöltése a pufferbe
    bioloid.readPose();          // read in current servo positions to the curPose buffer
    // puffer kiolvasása
    bioloid.interpolateSetup(time);  // setup for interpolation from current->next over 1 second
    // szögek graduális változtatása a pillanatnyi pozicióból a cél pozicióba 1 másodperc alatt
    // addíg iteráljuk a következö lépéseket ameddig el nem érjük a cél poziciót
    while(bioloid.interpolating > 0) // do this while we have not reached our new pose
    {  
           bioloid.interpolateStep();     // move servos, if necessary.
        for(int i = 0; i < 18; i++){
          current_pose[i] = ax12GetRegister(LEG_to_ID[i+1], AX_PRESENT_POSITION_L, 2);
        }
        Serial.print(0);  
        Serial.print(" ");
        print_current_pose();
      }
    
    bioloid.loadPose(csop24);        // load the pose from FLASH, into the nextPose buffer
    // beálitandó szög betöltése a pufferbe
    bioloid.readPose();          // read in current servo positions to the curPose buffer
    // puffer kiolvasása
    bioloid.interpolateSetup(time);  // setup for interpolation from current->next over 1 second
    // szögek graduális változtatása a pillanatnyi pozicióból a cél pozicióba 1 másodperc alatt
    // addíg iteráljuk a következö lépéseket ameddig el nem érjük a cél poziciót
    while(bioloid.interpolating > 0) // do this while we have not reached our new pose
    {  
            bioloid.interpolateStep();     // move servos, if necessary.
        for(int i = 0; i < 18; i++){
          current_pose[i] = ax12GetRegister(LEG_to_ID[i+1], AX_PRESENT_POSITION_L, 2);
        }
        Serial.print(0);  
        Serial.print(" ");
        print_current_pose();
     }
    
    bioloid.loadPose(csop25);        // load the pose from FLASH, into the nextPose buffer
    // beálitandó szög betöltése a pufferbe
    bioloid.readPose();          // read in current servo positions to the curPose buffer
    // puffer kiolvasása
    bioloid.interpolateSetup(time);  // setup for interpolation from current->next over 1 second
    // szögek graduális változtatása a pillanatnyi pozicióból a cél pozicióba 1 másodperc alatt
    // addíg iteráljuk a következö lépéseket ameddig el nem érjük a cél poziciót
    while(bioloid.interpolating > 0) // do this while we have not reached our new pose
    {  
            bioloid.interpolateStep();     // move servos, if necessary.
        for(int i = 0; i < 18; i++){
          current_pose[i] = ax12GetRegister(LEG_to_ID[i+1], AX_PRESENT_POSITION_L, 2);
        }
        Serial.print(0);  
        Serial.print(" ");
        print_current_pose();
    }
    
    bioloid.loadPose(csop26);        // load the pose from FLASH, into the nextPose buffer
    // beálitandó szög betöltése a pufferbe
    bioloid.readPose();          // read in current servo positions to the curPose buffer
    // puffer kiolvasása
    bioloid.interpolateSetup(time);  // setup for interpolation from current->next over 1 second
    // szögek graduális változtatása a pillanatnyi pozicióból a cél pozicióba 1 másodperc alatt
    // addíg iteráljuk a következö lépéseket ameddig el nem érjük a cél poziciót
    while(bioloid.interpolating > 0) // do this while we have not reached our new pose
    {  
        bioloid.interpolateStep();     // move servos, if necessary.
        for(int i = 0; i < 18; i++){
          current_pose[i] = ax12GetRegister(LEG_to_ID[i+1], AX_PRESENT_POSITION_L, 2);
        }
        Serial.print(0);  
        Serial.print(" ");
        print_current_pose();
      }
  
    /*
        bioloid.loadPose(csop17);        // load the pose from FLASH, into the nextPose buffer
    // beálitandó szög betöltése a pufferbe
    bioloid.readPose();          // read in current servo positions to the curPose buffer
    // puffer kiolvasása
    bioloid.interpolateSetup(100);  // setup for interpolation from current->next over 1 second
    // szögek graduális változtatása a pillanatnyi pozicióból a cél pozicióba 1 másodperc alatt
    // addíg iteráljuk a következö lépéseket ameddig el nem érjük a cél poziciót
    while(bioloid.interpolating > 0) // do this while we have not reached our new pose
    {  
          bioloid.interpolateStep();     // move servos, if necessary.
        for(int i = 0; i < 18; i++){
          current_pose[i] = ax12GetRegister(LEG_to_ID[i+1], AX_PRESENT_POSITION_L, 2);
        }
        Serial.print(0);  
        Serial.print(" ");
        print_current_pose();
      }
      */
}

void print_current_pose(){
  Serial.print((millis()- begin_time) / 1000.0 );
  Serial.print(" ");
  for(int i = 0; i < 18; i++){
    Serial.print(current_pose[i]);
    Serial.print(" ");
  }
  Serial.println(" ");
  //Serial.flush();  
}
