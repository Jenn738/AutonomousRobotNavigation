/*
 * Robotver3mod.c - Program that runs the Engs 147 robot
 *
 * Dan Magoon, Jennifer Jain, Jonah Sternthal, Joe Leonor
 *
 * Spring 2018
 * June 4th 2018
 */

#memmap xmem    //Force compilation to extended memory when the program gets large
#use BL26XX.lib   //Import Library for BL2600

#define DATA   0x00ff
#define RD     8
#define WR     9
#define CS1    10
#define CS2    11
#define YX     12
#define CD     13
#define BP_RESET        0X01        // reset byte pointer
#define EFLAG_RESET     0X86        // reset E bit of flag register
#define CNT             0x01       // access control register
#define DAT             0x00       // access data register
#define BP_RESETB       0X81        // reset byte pointer (x and y)
#define CLOCK_DATA      2        // FCK frequency divider
#define CLOCK_SETUP     0X98     // transfer PR0 to PSC (x and y)
#define INPUT_SETUP        0XC7     // enable inputs A and B (x and y)
                                   // set indexing function to load OL and
                                   // A and B enable gate TWP 4/2/12
#define QUAD_X1            0XA8     // quadrature multiplier to 1 (x and y)
#define QUAD_X2            0XB0     // quadrature multiplier to 2 (x and y)
#define QUAD_X4            0XB8     // quadrature multiplier to 4 (x and y)
#define CNTR_RESET         0X02     // reset counter
#define CNTR_RESETB        0X82     // reset counter (x and y)
#define TRSFRPR_CNTR       0X08     // transfer preset register to counter
#define TRSFRCNTR_OL       0X90     // transfer CNTR to OL (x and y)
#define XYIDR_SETUP        0XE1     // set index cntrl register to active low
                                   // input. index input is pulled up to +5V
                                   // in hardware to disable index functions
                                   // TWP 4/2/2012
#define HI_Z_STATE  0xFF

/**************** CONSTANT DECLERATIONS ****************/
#define LEFT        1     // left wheel motor encoder ID
#define RIGHT       2     // right wheel motor encoder ID
#define TOP         3     // IR sensor motor encoder ID

// Robot Specifications
#define WHEELBASE 15.4     // cm wheelbase of bot
#define STEP 0.0160     //cm per encoder tick of wheel

// Mathematical Constants
#define PIE 3.14159265
#define HALFPIE 1.57079633

// Mapping Constants
#define BOXES 77     // initialize grid size to BOXES x BOXES
#define MAPSIZE 5929     // = BOXES*BOXES
#define BOXLENGTH 5     // cm length per box on grid

#define MAPFILTER 100     // points-per-box required for an obstacle to be live on map
#define REQWEIGHT 4     // points-per-box threashold for an obstacle and neighbors
#define DEWEIGHT 0     // amount below mapfilter to push down values of neighborless boxes
#define NEIGHBORS 2     // amount of neighbors required to be obstacle
#define LOOKAHEAD 10     // cm distance to lead bot in calculations

// Driving Speeds
#define VHIGH 1     // max vavg encoder ticks per time tick
#define VLOW 0.4     // min vavg encoder ticks per time tick
#define VNE 1.3     // velocity Never Exceed foreward or reverse on each wheel
#define RAMP 0.1     // ramp factor per time cycle

// Costate Timing Constants
#define CTRLTICKS 100     // length of sample time (1 tick = 1/1024 secs)
#define IRTICKS 15

// IR Sensor Parameters
#define IRW 2.2     // volts to IR sensor motor (proportional to sensor rotation speed)
#define MAXSIGHT 100     // max sight distance in cm at which sensor looks ahead
#define SENSORSPIN 300

// Start and End Positions
#define XSTART 20     //cm
#define YSTART 20     //cm
#define TSTART 0.7853981     //rad from x axis
#define XEND 355     //cm
#define YEND 355     //cm


/**************** GLOBAL TYPES ****************/
// stores the information regarding the current state of the bot
typedef struct botState {
  // bot position and velocity 
  float lvel;     // left wheel velocity (encoder ticks/time ticks)
  float rvel;     // right wheel velocity (encoder ticks/time ticks)
  float xpos;     // robot x position (cm)
  float ypos;     // robot y position (cm)
  float theta;     // robot heading angle (rad)
  long lpos;     // left wheel encoder position (encoder ticks)
  long rpos;     // right wheel encoder position (encoder ticks)
  float lerrtot;     // left wheel velocity error sum 
  float rerrtot;     // right wheel velocity error sum
  float lcmd;     // velocity command to left wheel (encoder ticks/time ticks)
  float rcmd;     // velocity command to right wheel (encoder ticks/time ticks)
  float lreq;     // velocity request to left wheel (encoder ticks/time ticks)
  float rreq;     // velocity request to right wheel (encoder ticks/time ticks)

  // sensor update
  long IRstartPos;     // sensor start position (ticks)
  long IRcurrPos;     // sensor current position (ticks)
  float IRcurrDist;     // obstacle distance calculated by sensor (cm)
} botState_t;

// stores an x,y float value
typedef struct vertex {
  float x;
  float y;
} vertex_t;

// stores an x,y int value
typedef struct point {
  int x;
  int y;
} point_t;


/**************** GLOBAL VARIABLES ****************/
 botState_t curr_state, *curr_statep;
 botState_t IR_state, *IR_statep;
 point_t checkpts[8];
 vertex_t ir_point, *IRvertex;
 vertex_t obstacle, *obstaclep;
 float tcomold;
 float tcomcor;
 float tendcor;
 float vavg;
 int irdir;


/**************** FUNCTION DECLERATIONS ****************/
// BL2600 necessary functions
int EncRead(int channel, int reg);
void EncWrite(int channel, int data, int reg);
int init(void);

// Robot Position and Velocity State Functions
void stateInit(botState_t *botState);
void checkInit(point_t *arr);
void stateUpdate(botState_t *botState);
void controlLoop(botState_t *botState);
void reqVel(botState_t *botState);
long getPosition(int j);

// Robot Sensor State Functions
void IRxy(botState_t *botState, vertex_t *IRvertex);
void avoidObs(botState_t *botState, vertex_t *obstaclep);
void obsVect(botState_t *botState, vertex_t *obstaclep, long arr);
void sensorInit(botState_t *botState);
void sensorReverse(botState_t *botState);
float getDistance(void);

// Mapping Functions and Conversions
long initIntArray(long sz, int initVal);
int getIndex(int x, int y);
void insertElement(long arr, int idx, int value);
int getElement(long arr, int idx);
void incElement(long arr, int idx);
float idxToAvgPosX(int node);
float idxToAvgPosY(int node);
int idxToMapPosX(int node);
int idxToMapPosY(int node);
void IRMap(long map, vertex_t *IRvertex);
void printMap(long arr);
void parseMap(botState_t *botState, long arr, point_t *checks);


/* 
 *  Main - Initializes subsystems, sets up map array, switches and runs 
 *      the controller
 */ 
void main(){
  int breakout, i, j;
  long T0, T1, T2;
  long temptime;
  long map;

  /******* initialize subsystems *******/
  brdInit();
  anaOutConfig(1, DAC_SYNC);
  anaInConfig(0, 0);    // config for single ended unipolar
  anaInConfig(3, 0);    // config for single ended unipolar

  //Enable BL2600 power supply to drive D/A channels
  anaOutPwr(1);

  // send 0 volts to DAC to prevent indeterminate bot state
  anaOutVolts(TOP, 0);      
  anaOutStrobe();
  anaOutVolts(LEFT, 0);
  anaOutStrobe();
  anaOutVolts(RIGHT, 0);

  digOutConfig(0xff00);
  digOut(CS1, 1);
  digOut(CS2, 1);
  digOut(RD, 1);
  digOut(WR, 1);
  init();


  while(1){

    printf("INITIALIZING\n");

    /******* initialize global variables *******/
    irdir = 1;
    tcomold = 0;
    tcomcor = 0;
    tendcor = 0;
    obstacle.x = 0;
    obstacle.y = 0;

    breakout = 0;   // non global variable

    /******* initialize map *******/
    //Set up map array with repulsive border
    map = initIntArray(MAPSIZE, MAPFILTER);
    for(j = 1; j < BOXES-1; j++){
      for(i = 1; i < BOXES-1; i++){
        insertElement(map, BOXES * j + i, 0);
      }
    }
    //Make end box non-repulsive
    for(j = 69; j < BOXES; j++){
      for(i = 69; i < BOXES; i++){
        insertElement(map, BOXES * j + i, 0);
      }
    }


    /******* wait switch *******/
    if(anaInVolts(6, 0) < 3){
      while(anaInVolts(6, 0) < 3){
        anaOutVolts(LEFT, 0);
        anaOutVolts(RIGHT,0);
        anaOutVolts(TOP, 0);
        anaOutStrobe();
      }
      printf("STARTING\n");
    }

    curr_statep = &curr_state;
    IR_statep = &IR_state;
    IRvertex = &ir_point;
    obstaclep = &obstacle;
    stateInit(curr_statep);
    sensorInit(curr_statep);     // orients sensor
    checkInit(checkpts);
    stateUpdate(curr_statep);


    while(1){

      /******* costate for robot control and obstacle avoidance *******/
      costate{
        T0 = TICK_TIMER;
        stateUpdate(curr_statep);
        obsVect(curr_statep, obstaclep, map);
        avoidObs(curr_statep, obstaclep);
        reqVel(curr_statep);
        controlLoop(curr_statep);
        IR_state = curr_state;
        waitfor(TICK_TIMER-T0 >= CTRLTICKS);
      }

      /******* costate for IR sensor readings and map update *******/
      costate{
        if(TICK_TIMER-T0 >= CTRLTICKS){
          yield;
        }
        T1 = TICK_TIMER;
        stateUpdate(IR_statep);
        IRxy(IR_statep, IRvertex);
        IRMap(map, IRvertex);
        sensorReverse(IR_statep);
        waitfor(TICK_TIMER-T1 >= IRTICKS);
      }

      /******* costate for parsing map *******/
      costate{
        if(TICK_TIMER-T0 >= CTRLTICKS){
          yield;
        }
        T2 = TICK_TIMER;
        parseMap(curr_statep, map, checkpts);
        waitfor(TICK_TIMER-T2 >= CTRLTICKS);
      }

      /******* costate for switch that pauses and prints a map *******/
      costate{
        if(TICK_TIMER-T0 >= CTRLTICKS){
          yield;
        }
        if (anaInVolts(6, 0)<3){
          printf("PAUSED\n");
          while (anaInVolts(6, 0)<3){
            anaOutVolts(LEFT, 0);
            anaOutVolts(RIGHT,0);
            anaOutVolts(TOP,0);
            anaOutStrobe();
            if(anaInVolts(7, 0) > 3){
              printf("BREAKING\n");
              //PRINT OUT THE MAP FOR PLOTTING IN MATLAB
              printMap(map);
              //END OF MAP PRINTING
              breakout = 1;
              break;
            }
          }
          if(breakout == 0){
            printf("RESTARTING\n");
          }
        }
        if(breakout != 0){
          break;
        }
      }
    } 
  } 
} 


/* 
 *  reqVel - Request a velocity to the left and right wheels, sends a command 
 *      to ramp up or ramp down
 */ 
void reqVel(botState_t *botState){

  // limit velocity cmd to left wheel
  if(botState->lreq > VNE){
    botState->lreq = VNE;
  } else if(botState->lreq < -VNE){
    botState->lreq = -VNE;
  }

  // limit velocity cmd to right wheel
  if(botState->rreq > VNE){
    botState->rreq = VNE;
  } else if(botState->rreq < -VNE){
    botState->rreq = -VNE;
  }

  // changes the command velocity to the left wheel
  if(botState->lreq > botState->lcmd + RAMP){
    botState->lcmd += RAMP;
  } else if(botState->lreq < botState->lcmd - RAMP){
    botState->lcmd -= RAMP;
  } else {
    botState->lcmd = botState->lreq;
  }

  // changes the command velocity to the right wheel
  if (botState->rreq > botState->rcmd + RAMP){
    botState->rcmd += RAMP;
  } else if (botState->rreq < botState->rcmd - RAMP){
    botState->rcmd -= RAMP;
  } else {
    botState->rcmd = botState->rreq;
  }

}


/* 
 *  controlLoop - Sets the velocity in both the right and left wheels based on 
 *      control effort
 */ 
void controlLoop(botState_t *botState){
  float lP, lI, rP, rI, lerr, rerr;
  float luk, ruk;

  // controller gains
  lP = 3;
  lI = 1;
  rP = 3;
  rI = 1;

  // generate left wheel control effort
  lerr = botState->lcmd - (float)botState->lvel;
  botState->lerrtot += lerr;
  luk = lP * botState->lcmd + lI * botState->lerrtot;

  // generate right wheel control effort
  rerr = botState->rcmd - (float)botState->rvel;
  botState->rerrtot += rerr;
  ruk = rP * botState->rcmd + rI * botState->rerrtot;

  // update voltage sent to wheels
  anaOutVolts(LEFT, luk);
  anaOutStrobe();
  anaOutVolts(RIGHT, ruk);
  anaOutStrobe();
}

/* 
 *  getPosition - gets the position of encoder j in ticks
 */ 
long getPosition(int j) {
  int asb, bsb, csb;
  long position;

  EncWrite(j, TRSFRCNTR_OL, CNT);
  EncWrite(j,BP_RESETB,CNT);
  asb = EncRead(j,DAT);
  bsb = EncRead(j,DAT);
  csb = EncRead(j,DAT);

  position  = (long)asb;         // least significant byte
  position += (long)(bsb << 8);
  position += (long)(csb <<16);
  return position;
}

/* 
 *  stateUpdate - Updates the current state of the robot and sensor, updates 
 *      the botState struct
 */ 
void stateUpdate(botState_t *botState){
  long curr_l, curr_r, n_l, n_r;
  float d_theta, dx, dy, avg_theta, n_avg;
  float step;
  float base;
  float r;

  //sensor variables
  long sens_pos, start_pos; 
  float sens_dist; 

  /******* bot position update *******/
  curr_l = getPosition(LEFT);   // get position of left wheel
  curr_r = -1 * getPosition(RIGHT);  // get position of right wheel
  step = STEP;
  base = WHEELBASE;
  n_l = (curr_l - botState->lpos);  // difference in left curr and prev pos
  n_r = (curr_r - botState->rpos);  // difference in right curr and prev pos

  // rollover check
  if(labs(n_l)>5000){
    n_l = (long)(botState->lvel*CTRLTICKS);
  }
  if(labs(n_r)>5000){
    n_r = (long)(botState->rvel*CTRLTICKS);
  }

  n_avg = (n_l + n_r)/2.0;
  d_theta = (float)(n_r - n_l) * step/base;
  avg_theta = botState->theta + (d_theta/2.0);

  // find change in x and y positions
  dx = step * n_avg * cos(avg_theta);
  dy = step * n_avg * sin(avg_theta);

  /******* sensor readings update *******/
  sens_pos = getPosition(TOP) - botState->IRstartPos;
  sens_dist = getDistance();

  /******* bookkeeping into botState struct *******/
  botState->theta += d_theta;
  botState->xpos += dx;
  botState->ypos += dy;

  botState->lpos = curr_l;
  botState->rpos = curr_r;

  botState->lvel = (float)n_l/CTRLTICKS;
  botState->rvel = (float)n_r/CTRLTICKS;

  botState->IRcurrPos = sens_pos;
  botState->IRcurrDist = sens_dist;
}

/* 
 *  sensorReverse - Changes the sign of the voltage sent to the sensor based 
 *      on the desired sensor spin angle
 */ 
void sensorReverse(botState_t *botState){
  long sens_pos, start_pos;
  sens_pos = botState->IRcurrPos;
  start_pos = botState->IRstartPos;

  // change the sign of the voltage sent based on current sensor position
  if(sens_pos >= SENSORSPIN){
    irdir = -1;
  }else if(sens_pos <= -SENSORSPIN){
    irdir = 1;
  }

  anaOutVolts(TOP, IRW*irdir);
  anaOutStrobe();
}

/* 
 *  stateInit - Initializes the state of the robot upon program start
 */ 
void stateInit(botState_t *botState){
  botState->lvel = 0;
  botState->rvel = 0;
  botState->xpos = XSTART;
  botState->ypos = YSTART;
  botState->theta = TSTART;
  botState->lerrtot = 0;
  botState->rerrtot = 0;
  botState->lcmd = 0;
  botState->rcmd = 0;
  botState->lreq = 0;
  botState->rreq = 0;
  botState->lpos = getPosition(LEFT);
  botState->rpos = -1 * getPosition(RIGHT);

  // for sensor
  botState->IRcurrPos = getPosition(TOP);
  botState->IRcurrDist = getDistance(); 
}

/* 
 *  sensorInit - Initializes the state of the robot sensor upon program start
 */ 
void sensorInit(botState_t *botState){
  long start_pos;

  start_pos = getPosition(TOP);
  botState->IRstartPos = start_pos;
}

/* 
 *  avoidObs - Generates a potential navigation field by summing up attractive 
 *      and repulsive forces
 */ 
void avoidObs(botState_t *botState, vertex_t *obstaclep){
  float xbot, ybot, xrep, yrep, kt, lookahead, tline, dv, vr, vl, l;
  float tbot, tcom, vxcom, vycom, ka, km, kr;
  float vxend, vyend, dend, xatt, yatt, xmom, ymom, tend, tdiff, kdiff, drep, dcom;
  
  // avoidance gains
  kt = 1*vavg;    // theta gain
  ka = 5;    // attractive gain
  km = 15;    // momentum gain
  kr = 12000;   // repulsive gain (10000)
  kdiff = 0.14;    // windup gain
  lookahead = 5;    //cm distance at which to stop for endpoint

  // get current bot position
  xbot = botState->xpos;
  ybot = botState->ypos;
  tbot = botState->theta;

  // get obstacle positions to generate repulsive vectors
  xrep = obstaclep->x;
  yrep = obstaclep->y;

  // generate an attractive vector for the endpoint
  vxend = XEND - xbot;
  vyend = YEND - ybot;
  dend = sqrt(pow(vxend, 2.0) + pow(vyend, 2.0));
  xatt = vxend/dend;
  yatt = vyend/dend;

  // theta windup torque
  tend = atan2(vyend, vxend)+tendcor;
  tdiff = tend - tbot;

  if(tdiff > 3 * HALFPIE) {
    tdiff = 3 * HALFPIE;
  } else if (tdiff < -3 * HALFPIE) {
    tdiff = -3 * HALFPIE;
  }

  // avoid wall following by turning to goal after following obstacle to wall
  if(xbot < 30 || ybot > 370) {
      if(tdiff < -HALFPIE){
          tendcor = tendcor + 2 * PIE;
      }
  } else if(xbot > 370 || ybot < 30) {
      if(tdiff > HALFPIE){
          tendcor = tendcor - 2 * PIE;
      }
  }

  // generate an attractive vector based on current robot heading (momentum)
  xmom = cos(tbot);
  ymom = sin(tbot);

  // command vector
  vxcom = ka * xatt + km * xmom + kr * xrep;
  vycom = ka * yatt + km * ymom + kr * yrep;
  tcom = atan2(vycom, vxcom);
  dcom = pow(vxcom, 2.0) + pow(vycom, 2.0);

  // speed control based on total vector magnitude
  if(dcom > 300){
    vavg = VHIGH;
  } else {
    vavg = VLOW;
  }

  // correct command theta to allow for multiple rotations
  if(tcom - tcomold + tcomcor > PIE){
    tcomcor = tcomcor - 2 * PIE;
  } else if (tcom - tcomold + tcomcor < -PIE){
    tcomcor = tcomcor + 2 * PIE;
  }
  tcom = tcom + tcomcor;
  tcomold = tcom;

  // control effort (dv = right velocity increase = left velocity decrease)
  dv = kt * (tcom - tbot) + kdiff * tdiff;
  botState->rreq = vavg + dv; //right velocity request
  botState->lreq = vavg - dv; //left velocity request

  // compute distance to end point
  l = sqrt(pow((XEND - xbot), 2.0) + pow((YEND - ybot), 2.0));

  // ramp down velocity when close to end point 
  if (l<lookahead){
        botState->rreq = 0;   //right velocity request
        botState->lreq = 0;   //left velocity request
      }
}

/* 
 *  obsVect - Outward search from robot to check whether there are obstacles 
 *      around it. It generates a repulsive vetor sum based on the obstacles and 
 *      stores the x and y repulsive vector values
 */ 
void obsVect(botState_t *botState, vertex_t *obstaclep, long arr){
  int xbotcoord, ybotcoord, searchradius, xstart, ystart, xend, yend, i, j;
  int index;
  float xbot, ybot, tbot, dobs, dobs2, dzero, vxobs, vyobs, xrep, yrep;

  searchradius = 7;   //Squares to search in all directions
  dzero = 30;   // distance threashold for no effect

  // get the current state of bot
  tbot = botState->theta;
  xbot = botState->xpos + LOOKAHEAD * cos(tbot);
  ybot = botState->ypos + LOOKAHEAD * sin(tbot);

  xbotcoord = (int)floor(xbot/BOXLENGTH);
  ybotcoord = (int)floor(ybot/BOXLENGTH);

  // map a search radius around the bot of 7 units
  xstart = xbotcoord - searchradius;
  if(xstart < 0){
    xstart = 0;
  }
  ystart = ybotcoord - searchradius;
  if(ystart < 0){
    ystart = 0;
  }
  xend = xbotcoord + searchradius;
  if(xend >= BOXES){
    xend = BOXES - 1;
  }
  yend = ybotcoord + searchradius;
  if(yend >= BOXES){
    yend = BOXES - 1;
  }

  xrep = 0;
  yrep = 0;

  // generate repulsive vectors by doing an outward search from the bot
  for(j = ystart; j <= yend; j++){
    for(i = xstart; i <= xend; i++){
      index = (BOXES * j + i);
      if(getElement(arr, index)>=MAPFILTER){
        vxobs = xbot - idxToAvgPosX(index);
        vyobs = ybot - idxToAvgPosY(index);
        dobs=sqrt(pow(vxobs, 2.0) + pow(vyobs, 2.0));
        if(dobs <= dzero){
        dobs2 = pow(dobs, 2.0);
        xrep = xrep + (((1/dobs) - (1/dzero)) * (vxobs/dobs2));
        yrep = yrep + (((1/dobs) - (1/dzero)) * (vyobs/dobs2));
        }
      }
    }
  }

  // store repulsive vectors
  obstaclep->x = xrep;
  obstaclep->y = yrep;
}

/* 
 *  parseMap - Filters the map around the bot so false IR readings do not 
 *      inhibit the robot's path
 */ 
void parseMap(botState_t *botState, long arr, point_t *checks){
  int xbotcoord, ybotcoord, searchradius, xstart, ystart, xend, yend, i, j, k;
  int index, neighbors;
  float xbot, ybot, tbot;
  searchradius = 10; //Squares to search in all directions

  // get current state of the bot
  tbot = botState->theta;
  xbot = botState->xpos + LOOKAHEAD * cos(tbot);
  ybot = botState->ypos + LOOKAHEAD * sin(tbot);

  xbotcoord = (int)floor(xbot/BOXLENGTH);
  ybotcoord = (int)floor(ybot/BOXLENGTH);

  // let the search radius avoid any squares with walls
  xstart = xbotcoord - searchradius;
  if(xstart < 1){
    xstart = 1;
  }
  ystart = ybotcoord - searchradius;
  if(ystart < 1){
    ystart = 1;
  }
  xend = xbotcoord + searchradius;
  if(xend > BOXES - 2){
    xend = BOXES - 2;
  }
  yend = ybotcoord + searchradius;
  if(yend > BOXES - 2){
    yend = BOXES - 2;
  }

  // iterate through the search area
  for(j = ystart; j <= yend; j++){
    for(i = xstart; i <= xend; i++){

      // get an index value for map coord x and map coord y
      index = (BOXES * j + i);

      // only if the element has at least REQWEIGHT IR points in it and is less 
      // than MAPFILTER thresh then we want to search the neighbors to see if 
      // they have the REQWEIGHT number of points if there are at least 
      // NEIGHBORS # of live neighbors then set the value at the respective node
      // to MAPFILTER

      if((getElement(arr, index) >= REQWEIGHT) && (getElement(arr, index) < MAPFILTER)){
        neighbors = 0;
        for (k = 0; k < 8; k++) {
          if(getElement(arr, BOXES * (j + checks[k].y) + (i + checks[k].x)) >= REQWEIGHT){
            neighbors = neighbors + 1;
            if(neighbors >= NEIGHBORS){
              insertElement(arr, index, MAPFILTER);
              break;
            }
          }
        }
        // we want to filter for false points, so if the point does not have enough 
        // neighbors then make sure it isn't a live point
        if(!neighbors){
          insertElement(arr, index, REQWEIGHT-DEWEIGHT);
        }
      }
    }
  }
}

/* 
 *  checkInit - initialize points to search for neighbor searching
 */ 
void checkInit(point_t *arr){
    arr[0].x = 1;
    arr[0].y = 0;
    arr[1].x = 1;
    arr[1].y = 1;
    arr[2].x = 0;
    arr[2].y = 1;
    arr[3].x = -1;
    arr[3].y = 1;
    arr[4].x = -1;
    arr[4].y = 0;
    arr[5].x = -1;
    arr[5].y = -1;
    arr[6].x = 0;
    arr[6].y = -1;
    arr[7].x = 1;
    arr[7].y = -1;
}

/* 
 *  IRxy - Turns an IR reading into an x, y coordinate in cm
 */ 
void IRxy(botState_t *botState, vertex_t *IRvertex){
  float x, y, irtheta, sumtheta;
  if(botState->IRcurrDist < MAXSIGHT){
    irtheta = (float) 2 * PIE * (botState->IRcurrPos/1440.0);
    sumtheta = irtheta + botState->theta;
    IRvertex->x = botState->IRcurrDist * cos(sumtheta) + botState->xpos;
    IRvertex->y = botState->IRcurrDist * sin(sumtheta) + botState->ypos;
  }
}

/* 
 *  getDistance - IR sensor reading model to return where an obstacle is in cm
 */ 
float getDistance(void) {
  float dist, v_in;
  float p1, p2, p3, p4, p5;   // poly fit model

  // get sensor voltage
  v_in = anaInVolts(0, 1);
  v_in = (float)1/v_in;

  // model: using actual dist, inv volts w poly fit (deg 4, bisquare)
  p1 = 152.4;
  p2 = -381.3;
  p3 = 399.8;
  p4 = -134.4;
  p5 = 26.98;

  dist = p1 * pow(v_in, (float)4.0) + p2 * pow(v_in, (float)3.0) + p3 * pow(v_in, (float)2.0) + p4 * v_in + p5;

  return dist;
}


/**************** MAPPING FUNCTIONS AND CONVERSIONS ****************/

/* 
 *  initIntArray - Create an integer array in extended memory
 *    sz - size of requested array
 *    initVal - value for the array to be initialized to
 *    returns the address of the first element of the array in extened memory
 */ 
long initIntArray(long sz, int initVal) {
  long arr;
  int i;
  int element;
  sz = sz * sizeof(int);
  arr = xalloc(sz);
  for(i = 0; i < (sz/sizeof(int)); i++){
    xsetint((arr + i * sizeof(int)), initVal);
  }
  return arr;
}

/* 
 *  getIndex - gets node value of x, y grid point on map
 */ 
int getIndex(int x, int y){
  int index;
  if (x < 0 || y < 0 ){
    index = -8;
  }
  else if (x >= BOXES || y >= BOXES){
    index = -8;
  }
  else {
    index = BOXES * y + x;
  }
  return index;
}

/* 
 *  getElement - gets an element from an array stored in extended memory
 *    arr - address of array to access
 *    idx - index value of array
 *    returns an integer element value stored at the index
 */ 
int getElement(long arr, int idx){
  int element;
  element = xgetint(arr + idx * sizeof(int));
  return element;
}

/* 
 *  insertElement - set the value of an element in extended memeory array
 *    arr - address of array to access
 *    idx - index value to set
 *    value - integer value to store
 */ 
void insertElement(long arr, int idx, int value){
  xsetint(arr + idx * sizeof(int), value);
}

/* 
 *  incElement - increment value at a particular index in an extended array by +1
 *    arr - address of array to access
 *    idx - index value to increment
 */ 
void incElement(long arr, int idx){
  int value;
    value = getElement(arr, idx);
    value = value + 1;
    insertElement(arr, idx, value);
  }

/* 
 *  IRMap - maps ir sensor (x, y) struct to increment a square on the map
 *    map - address of map array to access
 *    IRvertex - struct with x, y location of IR point
 */ 
void IRMap(long map, vertex_t *IRvertex){
  int xcoord, ycoord, index;
  xcoord = (int)floor(IRvertex->x/BOXLENGTH);
  ycoord = (int)floor(IRvertex->y/BOXLENGTH);
  index = getIndex(xcoord, ycoord);
  if (index != -8){
    incElement(map, index);
  }
}

/* 
 *  idxToAvgPosX - get the center position x cm value of a certain node
 *    node - unique index value associated with a point on the grid
 *    returns center from 0,0 of the node in cm (x coord)
 */ 
float idxToAvgPosX(int node) {
  int map_pos_x;
  float avg_x;
  map_pos_x = idxToMapPosX(node);
  avg_x = BOXLENGTH * (map_pos_x + 1.0) - BOXLENGTH/2.0;
  return avg_x;
}

/* 
 *  idxToAvgPosY - get the center position y cm value of a certain node
 *    node - unique index value associated with a point on the grid
 *    returns center from 0,0 of the node in cm (y coord)
 */ 
float idxToAvgPosY(int node) {
  int map_pos_y;
  float avg_y;
  map_pos_y = idxToMapPosY(node);
  avg_y = BOXLENGTH * (map_pos_y + 1.0) - BOXLENGTH/2.0;
  return avg_y;
}

/* 
 *  idxToMapPosX - given unique node value, get the x map coord
 *    node - unique index value associated with a point on the grid
 *    returns map x coord (int)
 */ 
int idxToMapPosX(int node) {
  int map_pos_x;
  map_pos_x = (int) (float)node % BOXES;
  return map_pos_x;
}

/* 
 *  idxToMapPosY - given unique node value, get the y map coord
 *    node - unique index value associated with a point on the grid
 *    returns map y coord (int)
 */ 
int idxToMapPosY(int node) {
  int map_pos_y;
  map_pos_y = (int)floor(node/BOXES);
  return map_pos_y;

}

/* 
 *  printMap - prints all the obstacles that were mapped
 */ 
void printMap(long arr) {
  int i, j, index;
  float x, y;
  for(j = 1; j < BOXES-1; j++){
    for(i = 1; i < BOXES-1; i++){
      index = BOXES * j + i;
      if(getElement(arr, index) >= MAPFILTER){
        x = idxToAvgPosX(index);
        y = idxToAvgPosY(index);
        printf("%f, %f\n", x, y);
      }
    }
  }
}


/**************** NECESSARY BL2600 FUNCTIONS ****************/

int init(void)
{
  int fail;
  int i,j,k,delayvar;
  fail = 0;

  for (i = 0; i<4; i++)
  {
     EncWrite(i, XYIDR_SETUP,CNT);    // Disable Index
     EncWrite(i,EFLAG_RESET,CNT);

     EncWrite(i,BP_RESETB,CNT);

     EncWrite(i,CLOCK_DATA,DAT);
     EncWrite(i,CLOCK_SETUP,CNT);
     EncWrite(i,INPUT_SETUP,CNT);
     EncWrite(i,QUAD_X4,CNT);

     EncWrite(i,BP_RESETB,CNT);
     EncWrite(i,0x12,DAT);
     EncWrite(i,0x34,DAT);
     EncWrite(i,0x56,DAT);

     EncWrite(i,TRSFRPR_CNTR,CNT);
     EncWrite(i,TRSFRCNTR_OL,CNT);

     EncWrite(i,BP_RESETB,CNT);

  }
  return fail;
}

// channel is an int from 0 to 3 indicating which encoder
// reg is an int which is 1 or 0 indicating whether control or data is desired
int EncRead(int channel, int reg)
{
  int EncData;
  int i, delayvar;
  EncData = 0;

  digOutConfig(0xff00); // set data lines as inputs and everything else as outputs

  // select which chip
  if (channel <= 1)
     digOut(CS1,0);
  else
     digOut(CS2,0);

   // Select control or data register
   digOut(CD,reg);

  // select which channel, X or Y
  if ((channel == 0) | (channel == 3) )
     digOut(YX,0);
 else
     digOut(YX,1);
    // assert Read low
  digOut(RD,0);

  EncData = digInBank(0);     // read the data from the data lines

   //deassert read reads the data.  Deassert, delay to allow rise
  // then deselect chips
  digOut(RD,1);

  digOut(CS1,1);
  digOut(CS2,1);

  return EncData;
}

void EncWrite(int channel, int data, int reg)
{
  int i, delayvar;

  // select which chip - channel 0 & 1 are chip 1 and channel 2 & 3 are chip 2
  if (channel <= 1)
     digOut(CS1,0);
  else
     digOut(CS2,0);
  // select which channel, X or Y  X = 0 and 2, Y = 1 and 3

  digOut(CD,reg);

  if ((channel == 0) | (channel == 3) )
     digOut(YX,0);
 else
   digOut(YX,1);
   // assert write
   digOut(WR,0);    //First assert WR before driving outputs to avoid bus
                    //contention with encoder board  TWP 4/2/12

   digOutConfig(0xffff);// set all digI/O lines as outputs
   digOutBank((char)0,(char)data);



  // deassert write
  digOut(WR,1);

  // deselect chip
  digOut(CS1,1);
  digOut(CS2,1);
  //Set all outputs to 1 so that open collector transistor is off
  digOutBank((char)0,(char)HI_Z_STATE);
  digOutConfig(0xff00);
}
