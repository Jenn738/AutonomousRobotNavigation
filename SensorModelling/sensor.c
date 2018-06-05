 //Force functions to be compiled to extended memory.  Helps when the
// program gets large
#memmap xmem

//Import Library for BL2600
//#use STDIO.LIB
#use BL26XX.lib

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
#define HI_Z_STATE         0xFF
#define num_samples 200
#define M_PI 3.14159265358979323846

/**** FUNCTION DECLERATIONS ****/
void DispStr(int x, int y, char *s);

int EncRead(int channel, int reg);
void EncWrite(int channel, int data, int reg);
float getDistance1(float v_in);
float getDistance2(float v_in);
float getDistance3(float v_in);
float getDistance4(float v_in);
float getDistance5(float v_in);
float getDistance6(float v_in);
float getDistance7(float v_in);


void DispStr(int x, int y, char *s)
{
   x += 0x20;
   y += 0x20;
   printf ("\x1B=%c%c%s", x, y, s);
}


void main(void){

  /**** VARIABLES ****/
  int m, s;
  int done;
  float calcDist;
  float dist1, dist2, dist3, dist4, dist5, dist6, dist7;
  float curr_sens_v;
  float v_samples[num_samples];
  float actDist_samples[num_samples];
  float dist_samples[num_samples];

  auto float actDist;
  auto char tmpbuf[128];



  /**** INITIALIZE SUBSYSTEMS ****/
  brdInit();

  digOutConfig(0xff00);
  digOut(CS1,1);
  digOut(CS2,1);
  digOut(RD,1);
  digOut(WR,1);

  // Configure channel 0 for IR sensor voltage input
  anaInConfig(0, 0);    // config for single ended unipolar

  //Enable BL2600 power supply to drive D/A channels
  anaOutPwr(1);


  /**** INITIALIZE VARIABLES ****/
  curr_sens_v = 0;
  done = 0;


  // clear sample arrays
  for(m = 0; m < num_samples; m++) 
{    v_samples[m] = 0.0;
    actDist_samples[m] = 0.0;
    dist_samples[m] = 0.0;
  }


  // continue prompting user for measurements
  //printf("ENTER the actual distance, enter 0 to exit: ");

 // FILE *fp = fopen ("results.txt", "w+");
  while(!done) {

    // prompt user for actual distance
    printf("ENTER the actual distance, enter 0 to exit: ");
    actDist = atof(gets(tmpbuf));

    // exit loop is '0' is entered
    if (actDist == 0) {
      done = 1;
    }
    // store actual distance
    //printf("The value you entered is: %f\n", actDist);

    // get the sensor voltage and store
    curr_sens_v = anaInVolts(0, 1);
    curr_sens_v = curr_sens_v + anaInVolts(0, 1);
    curr_sens_v = curr_sens_v + anaInVolts(0, 1);
    curr_sens_v = curr_sens_v/(float)3;

    //printf("The current sensor reading is: %f\n", curr_sens_v);

    // dist1 = getDistance1(curr_sens_v);
    dist2 = getDistance2(curr_sens_v);
    // dist3 = getDistance3(curr_sens_v);
    // dist4 = getDistance4(curr_sens_v);
    // dist5 = getDistance5(curr_sens_v);
    // dist6 = getDistance6(curr_sens_v);
    // dist7 = getDistance7(curr_sens_v);

//    fprintf(fp, "%f, %f, %f, %f, %f, %f, %f, %f, %f\n", curr_sens_v, actDist, dist1, dist2, dist3, dist4, dist5, dist6, dist7 );
    //printf("%f, %f, %f, %f, %f, %f, %f, %f, %f\n", curr_sens_v, actDist, dist1, dist2, dist3, dist4, dist5, dist6, dist7 );

  }
  //fclose(fp);

}



// need to put model in this functions
float getDistance1(float v_in) {
  float dist;
  float p1, p2, p3, q1;   // model 1: rational fit


  // model 1: using actual dist, volts w rational fit (num: 2 deg, den: 1 deg)
  p1 = -4.039;
  p2 = 17.9;
  p3 = 12.15;
  q1 = -0.5868;

  dist = (p1*pow(v_in, (float)2.0) + p2*v_in + p3) / (v_in + q1);

  return dist;
}

float getDistance2(float v_in) {
  float dist;
  float p1, p2, p3, p4, p5;   // model 2: poly fit

  // get sensor voltage
  v_in = (float)1/v_in;

  // model 2: using actual dist,  inv volts w poly fit (deg 4, bisquare)
  p1 = 152.4;
  p2 = -381.3;
  p3 = 399.8;
  p4 = -134.4;
  p5 = 26.98; 

  dist = p1*pow(v_in, (float)4.0) + p2*pow(v_in, (float)3.0) + p3*pow(v_in, (float)2.0) + p4*v_in + p5;

  return dist;
}


float getDistance3(float v_in) {
  float dist;
  float a, b, c, d;   // model 3: exponential fit

  // model 3: using inv dist, actual volts w exponential fit (2 terms)
  a = 0.1355;
  b = 0.04664;
  c = -0.1553;
  d = -0.2055;

  dist = a*exp(b*v_in) + c*exp(d*v_in);
  dist = (float)1/dist;

  return dist;
}

float getDistance4(float v_in) {
  float dist;
  float a, b, c;   // model 4: linear fit

  // model 4: using inv dist, actual volts w linear fit 
  a = -0.0008729;
  b = -0.001781;
  c = 0.1591;
  dist = a*(sin(v_in-M_PI)) + b*(pow(v_in-(float)10, (float)2.0)) + c;
  dist = (float)1/dist;
  
  return dist;
}

float getDistance5(float v_in) {
  float dist;
  float p1, p2, p3, p4;   // model 5: poly fit

  // model 5: using inv dist, actual volts w poly fit (deg 3)
  p1 = 0.0006422;
  p2 = -0.005483;
  p3 = 0.04199;
  p4 = -0.02149;

  dist = p1*pow(v_in, (float)3.0) + p2*pow(v_in, (float)2.0) + p3*v_in + p4 ;
  dist = (float)1/dist;
  
  return dist;
}


float getDistance6(float v_in) {
  float dist;
  float a, b, c;   // model 6: power fit

  // model 6: using inv dist, actual volts w powwer fit (2 terms)
  a = 0.04293;
  b = 0.7657;
  c = -0.02727;

  dist = a*pow(v_in, b)+c;
  dist = (float)1/dist;
  
  return dist;
}


float getDistance7(float v_in) {
  float p1, p2, p3, q1;   // model 7: rational fit
  float dist;

  // model 7: using inv dist, actual volts w rational fit (num:2, den: 1)
  p1 = 0.02327;
  p2 = 0.04265;
  p3 = -0.03095;
  q1 = 1.229;

  dist = (p1*pow(v_in, (float)2.0) + p2*v_in + p3) / (v_in + q1);
  dist = (float)1/dist;
  
  return dist;
}



/************ Necessary Functions ************/
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
      printf("written = %d, read = %d\n",0x12,EncRead(i,DAT));
      printf("written = %d, read = %d\n",0x34,EncRead(i,DAT));
      printf("written = %d, read = %d\n",0x56,EncRead(i,DAT));

    // Reset the counter now so that starting position is 0  TWP 4/2/12
    EncWrite(i,CNTR_RESET,CNT);
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

//float movePosition(float r, float *v_vect)
//float movePosition(float r)
void movePosition(float r)
{
  /**** INITIALIZE VARIABLES ****/

  int i,j, m, num_samplesmove;
  int delayvar;
  int asb, bsb, csb;
  int N;
  int q;

  unsigned long int T0, thold, thold2;
  long position;
  float radpos, prev_radpos, rpos, output;
  float u_temp, e_temp, u_prev_temp, e_prev_temp, e_prev_temp2, u_prev_temp2;
  float e_k, u_k, r_k, x_k, r_star, time, cycle_time, dt;
  //float r, K, Ke;
  float K, Ke;
  float time_samples[200];
  float u_samples[200];
  float e_samples[200];
  float position_samples[200];
  float coef1,coef2,coef3,coef4,coef5;

  /**** ASSIGN VARIABLES ****/
  num_samplesmove = 8;
  Ke = 1600;
  //r = 9; //degrees
  r = r*2*3.1415; //Convert to radians
  r = r/360; //degrees desired in radians
  K = 18;

  // poles and zeros of system
  coef1 = 1;
  coef2 = 1.94;
  coef3 = 0.9405;
  coef4 = 1.594;
  coef5 = 0.594;
  N = 10;   // sample time

  dt = (float) N/1024;   // change in time
  r_k = 0;    // initial voltage
  j = 1;  // encoder slot used

  i = 0;
  q = 0;
  thold = TICK_TIMER;


  // INITIAL VALUES
  e_samples[0] = 0;
  u_samples[0] = 0;
  time_samples[0] = 0;
  e_prev_temp = 0;
  u_prev_temp = 0;
  e_prev_temp2 = 0;
  u_prev_temp2 = 0;


  EncWrite(j, TRSFRCNTR_OL, CNT);
  EncWrite(j,BP_RESETB,CNT);
  asb = EncRead(j,DAT);
  bsb = EncRead(j,DAT);
  csb = EncRead(j,DAT);

  position  = (long)asb;      // least significant byte
  position += (long)(bsb << 8);
  position += (long)(csb <<16);

  position_samples[0] = (position*2*3.1415)/1600;
  rpos = r + position_samples[0];

  m = 1;
  T0 = TICK_TIMER;
  while (m < num_samplesmove){
  // collect data
  // reset T0 for use in sample time count

    EncWrite(j, TRSFRCNTR_OL, CNT);
    EncWrite(j,BP_RESETB,CNT);
    asb = EncRead(j,DAT);
    bsb = EncRead(j,DAT);
    csb = EncRead(j,DAT);

    position  = (long)asb;      // least significant byte
    position += (long)(bsb << 8);
    position += (long)(csb <<16);

    // store samples
    radpos = (((float)position)*2*3.1415)/1600;
    e_temp = rpos - radpos;
    u_temp = K*(coef1 *e_temp - coef2*e_prev_temp + coef3*e_prev_temp2) + coef4*u_prev_temp - coef5*u_prev_temp2;

    anaOutVolts(1,u_temp);
    anaOutStrobe();

    // book-keeping
    e_samples[m] = e_temp;
    e_prev_temp2 = e_prev_temp;
    e_prev_temp = e_temp;
    u_samples[m] = u_temp;
    u_prev_temp2 = u_prev_temp;
    u_prev_temp = u_temp;
    time_samples[m] = m*dt;
    position_samples[m] = radpos;

    m++;

    while ((TICK_TIMER - T0) < N)  {
    }
    T0 = TICK_TIMER;
    }

    // r_k = 0;
    // anaOutVolts(1,r_k);
    // anaOutStrobe();

    //output = position_samples[num_samplesmove-1];
    //return output;
}







