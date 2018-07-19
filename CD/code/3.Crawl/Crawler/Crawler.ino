/* -----------------------------------------------------------------------------
  - Project: Remote control Crawling robot
  - Author:	panerqiang@sunfounder.com
  - Date:	2015/1/27
   -----------------------------------------------------------------------------
  - Overview
  - This project was written for the Crawling robot desigened by Sunfounder.
    This version of the robot has 4 legs, and each leg is driven by 3 servos.
	This robot is driven by a Ardunio Nano Board with an expansion Board.
	We recommend that you view the product documentation before using.
  - Request
  - This project requires some library files, which you can find in the head of
    this file. Make sure you have installed these files.
  - How to
  - Before use,you must to adjust the robot,in order to make it more accurate.
    - Adjustment operation
	  1.uncomment ADJUST, make and run
	  2.comment ADJUST, uncomment VERIFY
	  3.measure real sites and set to real_site[4][3], make and run
	  4.comment VERIFY, make and run
	The document describes in detail how to operate.
   ---------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include <Servo.h>		//to define and control servos
#include <EEPROM.h>		//to save errors of all servos

/* Installation and Adjustment -----------------------------------------------*/
//#define INSTALL	//uncomment only this to install the robot
//#define ADJUST	//uncomment only this to adjust the servos
//#define VERIFY	//uncomment only this to verify the adjustment
const int pos_x = 0;
const int pos_y = 1;
const int pos_z = 2;
const float adjust_site[3] = {
  75, // x
  72, // y
  27  // z
};
const float real_site[4][3] = {
  { 75, 72, 27 }, // front right
  { 75, 72, 27 }, // back right
  { 75, 72, 27 }, // front left
  { 75, 72, 27 }  // back left
};


/* Serv
  /* Servos --------------------------------------------------------------------*/
//define 12 servos for 4 legs
Servo servo[4][3];
//define servos' ports
const int servo_pin[4][3] = { 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13 };

/* Size of the robot ---------------------------------------------------------*/
const int femur_servo_index = 0;
const int tibia_servo_index = 1;
const int coxa_servo_index = 2;
const float length_femur = 51;
const float length_tibia = 80;
const float length_coxa = 30;
const float length_side = 90;
const float z_absolute = -27;
/* Constants for movement ----------------------------------------------------*/
const float z_default = -70, z_up = -10, z_boot = z_absolute;
const float x_default = 75, x_offset = 0;
const float y_start = 0, y_step = 50;
/* variables for movement ----------------------------------------------------*/
volatile float site_now[4][3];		//real-time coordinates of the end of each leg
volatile float site_expect[4][3];	//expected coordinates of the end of each leg
float temp_speed[4][3];		//each axis' speed, needs to be recalculated before each movement
float move_speed;			//movement speed
float speed_multiple = 1;	//movement speed multiple
const float spot_turn_speed = 4;
const float leg_move_speed = 8;
const float body_move_speed = 3;
const float stand_seat_speed = 1;
volatile int rest_counter;			//+1/0.02s, for automatic rest
const int wait_rest_time = 3 * 50;	//3s*50Hz, the time wait for automatic rest
//functions' parameter
const float KEEP = 255;
//define PI for calculation
const float pi = 3.1415926;
/* Constants for turn --------------------------------------------------------*/
//temp length
const float temp_a = sqrt(pow(2 * x_default + length_side, 2) + pow(y_step, 2));
const float temp_b = 2 * (y_start + y_step) + length_side;
const float temp_c = sqrt(pow(2 * x_default + length_side, 2) + pow(2 * y_start + y_step + length_side, 2));
const float temp_alpha = acos((pow(temp_a, 2) + pow(temp_b, 2) - pow(temp_c, 2)) / 2 / temp_a / temp_b);
//site for turn
const float turn_x1 = (temp_a - length_side) / 2;
const float turn_y1 = y_start + y_step / 2;
const float turn_x0 = turn_x1 - temp_b * cos(temp_alpha);
const float turn_y0 = temp_b * sin(temp_alpha) - turn_y1 - length_side;
/* ---------------------------------------------------------------------------*/
#define TIME_INTERVAL 5000

#define FORWARD 'f'
#define LEFT 'l'
#define STAND 's'
#define RIGHT 'r'
#define BACKWARD 'b'

unsigned long cur_time;
/*
  - setup function
   ---------------------------------------------------------------------------*/
void setup()
{
#ifdef INSTALL
  //initialize all servos
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      servo[i][j].attach(servo_pin[i][j]);
      delay(100);
    }
  }
  while (1);
#endif
#ifdef ADJUST
  adjust();
  while (1);
#endif
#ifdef VERIFY
  verify();
  while (1);
#endif

  //start serial for debug
  Serial.begin(115200);
  Serial.println("Robot starts initialization");

  //initialize default parameter
  set_site(0, x_default - x_offset, y_start + y_step, z_boot);
  set_site(1, x_default - x_offset, y_start + y_step, z_boot);
  set_site(2, x_default + x_offset, y_start, z_boot);
  set_site(3, x_default + x_offset, y_start, z_boot);
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      site_now[i][j] = site_expect[i][j];
    }
  }


  Serial.println("Servo service started");
  //initialize servos
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      servo[i][j].attach(servo_pin[i][j]);
      //delay(100);
    }
  }
  //start servo service
  cli();
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
  sei();
  stand();
  cur_time = millis();
  Serial.println("Servos initialized");
  Serial.println("Robot initialization Complete");

}
/*
  - loop function
   ---------------------------------------------------------------------------*/
void loop()
{
#ifdef INSTALL
  while (1);
#endif
#ifdef ADJUST
  while (1);
#endif
#ifdef VERIFY
  while (1);
#endif

  while (1) {
    char movements[] = {
      FORWARD, LEFT, FORWARD, RIGHT, BACKWARD
    };
    static unsigned long old_time = cur_time;
    static int c = 0;
    char cmd;
    if (cur_time - old_time >= TIME_INTERVAL ) {
      old_time = cur_time;
__auto:
      if (1) {
        c = c % (sizeof(movements) / sizeof(char));
        cmd = movements[c++];
      } else {
        c = (int)random(0, sizeof(movements) / sizeof(char));
        cmd = movements[c];
      }
    }

    switch (cmd) {
      case FORWARD:
        step_forward(1);
        break;
      case BACKWARD:
        step_back(1);
        break;
      case RIGHT:
        turn_right(1);
        break;
      case LEFT:
        turn_left(1);
        break;
      case STAND:
        stand();
        break;
    }
    cur_time = millis();    
  }

  if (1 && rest_counter > wait_rest_time)
  {
    if (is_stand())
    {
      Serial.println("Auto sit");
      sit();
      rest_counter = 0;
    }
  }
}

/*
  - adjustment function
  - move each leg to adjustment site, so that you can measure the real sites.
   ---------------------------------------------------------------------------*/
void adjust(void)
{
  //initializes eeprom's errors to 0
  //number -100 - +100 is map to 0 - +200 in eeprom
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      EEPROM.write(i * 6 + j * 2, 100);
      EEPROM.write(i * 6 + j * 2 + 1, 100);
    }
  }

  //initializes the relevant variables to adjustment position
  for (int leg = 0; leg < 4; leg++)
  {
    set_site(leg, adjust_site[pos_x], adjust_site[pos_y], adjust_site[pos_z] + z_absolute);
    for (int j = 0; j < 3; j++)
    {
      site_now[leg][j] = site_expect[leg][j];
    }
  }

  //initialize servos
  for (int leg = 0; leg < 4; leg++)
  {
    for (int j = 0; j < 3; j++)
    {
      servo[leg][j].attach(servo_pin[leg][j]);
      //delay(100);
    }
  }
  //start servo service
  cli();
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
  sei();
}

/*
  - verify function
  - verify the adjustment results, it will calculate errors and save to eeprom.
   ---------------------------------------------------------------------------*/
void verify(void)
{
  //calculate correct degree
  float alpha0, beta0, gamma0;
  cartesian_to_polar(alpha0, beta0, gamma0, adjust_site[pos_x], adjust_site[pos_y], adjust_site[pos_z] + z_absolute);
  //calculate real degree and errors
  float alpha, beta, gamma;
  float degree_error[4][3];
  for (int leg = 0; leg < 4; leg++)
  {
    cartesian_to_polar(alpha, beta, gamma, real_site[leg][pos_x], real_site[leg][pos_y], real_site[leg][pos_z] + z_absolute);
    degree_error[leg][femur_servo_index] = alpha0 - alpha;
    degree_error[leg][tibia_servo_index] = beta0 - beta;
    degree_error[leg][coxa_servo_index] = gamma0 - gamma;
  }
  //save errors to eeprom
  for (int leg = 0; leg < 4; leg++)
  {
    for (int j = 0; j < 3; j++)
    {
      EEPROM.write(leg * 6 + j * 2, (int)degree_error[leg][j] + 100);
      EEPROM.write(leg * 6 + j * 2 + 1, (int)(degree_error[leg][j] * 100) % 100 + 100);
    }
  }

  //initializes the relevant variables to adjustment position
  for (int leg = 0; leg < 4; leg++)
  {
    set_site(leg, adjust_site[pos_x], adjust_site[pos_y], adjust_site[pos_z] + z_absolute);
    for (int j = 0; j < 3; j++)
    {
      site_now[leg][j] = site_expect[leg][j];
    }
  }

  //initialize servos
  for (int leg = 0; leg < 4; leg++)
  {
    for (int j = 0; j < 3; j++)
    {
      servo[leg][j].attach(servo_pin[leg][j]);
      //delay(100);
    }
  }
  //start servo service
  //  FlexiTimer2::set(20, servo_service);
  //  FlexiTimer2::start();
  cli();
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
  sei();
}

/*
  - sit
  - blocking function
   ---------------------------------------------------------------------------*/
void sit(void)
{
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++)
  {
    set_site(leg, KEEP, KEEP, z_boot);
  }
  wait_all_reach();
}

/*
  - stand
  - blocking function
   ---------------------------------------------------------------------------*/
void stand(void)
{
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++)
  {
    set_site(leg, KEEP, KEEP, z_default);
  }
  wait_all_reach();
}

/*
  - is_stand
   ---------------------------------------------------------------------------*/
bool is_stand(void)
{
  if (site_now[0][2] == z_default)
    return true;
  else
    return false;
}

/*
  - spot turn to left
  - blocking function
  - parameter step steps wanted to turn
   ---------------------------------------------------------------------------*/
void turn_left(unsigned int step)
{
  move_speed = spot_turn_speed;
  while (step-- > 0)
  {
    if (site_now[3][1] == y_start)
    {
      //leg 3&1 move
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start, z_up);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 0&2 move
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_up);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

/*
  - spot turn to right
  - blocking function
  - parameter step steps wanted to turn
   ---------------------------------------------------------------------------*/
void turn_right(unsigned int step)
{
  move_speed = spot_turn_speed;
  while (step-- > 0)
  {
    if (site_now[2][1] == y_start)
    {
      //leg 2&0 move
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_up);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 1&3 move
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

/*
  - go forward
  - blocking function
  - parameter step steps wanted to go
   ---------------------------------------------------------------------------*/
void step_forward(unsigned int step)
{
  move_speed = leg_move_speed;
  while (step-- > 0)
  {
    if (site_now[2][1] == y_start)
    {
      //leg 2&1 move
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 0&3 move
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

/*
  - go back
  - blocking function
  - parameter step steps wanted to go
   ---------------------------------------------------------------------------*/
void step_back(unsigned int step)
{
  move_speed = leg_move_speed;
  while (step-- > 0)
  {
    if (site_now[3][1] == y_start)
    {
      //leg 3&0 move
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 1&2 move
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}



/*
  - set one of end points' expect site
  - this founction will set temp_speed[4][3] at same time
  - non - blocking function
   ---------------------------------------------------------------------------*/
void set_site(int leg, float x, float y, float z)
{
  float length_x = 0, length_y = 0, length_z = 0;

  if (x != KEEP)
    length_x = x - site_now[leg][pos_x];
  if (y != KEEP)
    length_y = y - site_now[leg][pos_y];
  if (z != KEEP)
    length_z = z - site_now[leg][pos_z];

  float length = sqrt(pow(length_x, 2) + pow(length_y, 2) + pow(length_z, 2));

  temp_speed[leg][pos_x] = length_x / length * move_speed * speed_multiple;
  temp_speed[leg][pos_y] = length_y / length * move_speed * speed_multiple;
  temp_speed[leg][pos_z] = length_z / length * move_speed * speed_multiple;

  if (x != KEEP)
    site_expect[leg][pos_x] = x;
  if (y != KEEP)
    site_expect[leg][pos_y] = y;
  if (z != KEEP)
    site_expect[leg][pos_z] = z;
}

/*
  - wait one of end points move to expect site
  - blocking function
   ---------------------------------------------------------------------------*/
void wait_reach(int leg)
{
  while (1)
    if (site_now[leg][pos_x] == site_expect[leg][pos_x])
      if (site_now[leg][pos_y] == site_expect[leg][pos_y])
        if (site_now[leg][pos_z] == site_expect[leg][pos_z])
          break;
}

/*
  - wait one of end points move to one site
  - blocking function
   ---------------------------------------------------------------------------*/
void wait_reach(int leg, float x, float y, float z)
{
  while (1)
    if (site_now[leg][pos_x] == x)
      if (site_now[leg][pos_y] == y)
        if (site_now[leg][pos_z] == z)
          break;
}

/*
  - wait all of end points move to expect site
  - blocking function
   ---------------------------------------------------------------------------*/
void wait_all_reach(void)
{
  for (int i = 0; i < 4; i++)
    wait_reach(i);
}

/*
  - microservos service /timer interrupt function/50Hz
  - when set site expected,this function move the end point to it in a straight line
  - temp_speed[4][3] should be set before set expect site,it make sure the end point
   move in a straight line,and decide move speed.
   ---------------------------------------------------------------------------*/
SIGNAL(TIMER0_COMPA_vect) {
  static long old_time = cur_time;
  if (cur_time - old_time >= 20) {
    old_time = cur_time;
    servo_service();
  }
  cur_time = millis();
}

inline void servo_service(void)
{
  sei();
  static float alpha, beta, gamma;

  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      if (abs(site_now[i][j] - site_expect[i][j]) >= abs(temp_speed[i][j]))
        site_now[i][j] += temp_speed[i][j];
      else
        site_now[i][j] = site_expect[i][j];
    }

    cartesian_to_polar(alpha, beta, gamma, site_now[i][0], site_now[i][1], site_now[i][2]);
    polar_to_servo(i, alpha, beta, gamma);
  }

  rest_counter++;
}
/*
  - trans site from polar to cartesian
  - mathematical model 1/2
   ---------------------------------------------------------------------------*/
void polar_to_cartesian(volatile float alpha, volatile float beta, volatile float gamma, volatile float &x, volatile float &y, volatile float &z)
{
  //trans degree 180->pi
  alpha = alpha / 180 * pi;
  beta = beta / 180 * pi;
  gamma = gamma / 180 * pi;
  //calculate w-z position
  float v, w;
  v = length_femur * cos(alpha) - length_tibia * cos(alpha + beta);
  z = length_femur * sin(alpha) - length_tibia * sin(alpha + beta);
  w = v + length_coxa;
  //calculate x-y-z position
  x = w * cos(gamma);
  y = w * sin(gamma);
}

/*
  - trans site from cartesian to polar
  - mathematical model 2/2
   ---------------------------------------------------------------------------*/
void cartesian_to_polar(volatile float &alpha, volatile float &beta, volatile float &gamma, volatile float x, volatile float y, volatile float z)
{
  //calculate w-z degree
  float v, w;
  w = (x >= 0 ? 1 : -1) * (sqrt(pow(x, 2) + pow(y, 2)));
  v = w - length_coxa;
  alpha = atan2(z, v) + acos((pow(length_femur, 2) - pow(length_tibia, 2) + pow(v, 2) + pow(z, 2)) / 2 / length_femur / sqrt(pow(v, 2) + pow(z, 2)));
  beta = acos((pow(length_femur, 2) + pow(length_tibia, 2) - pow(v, 2) - pow(z, 2)) / 2 / length_femur / length_tibia);
  //calculate x-y-z degree
  gamma = (w >= 0) ? atan2(y, x) : atan2(-y, -x);
  //trans degree pi->180
  alpha = alpha / pi * 180;
  beta = beta / pi * 180;
  gamma = gamma / pi * 180;
}

/*
  - trans site from polar to microservos
  - mathematical model map to fact
  - the errors saved in eeprom will be add
   ---------------------------------------------------------------------------*/
void polar_to_servo(int leg, float alpha, float beta, float gamma)
{

  float alpha_error = EEPROM.read(leg * 6 + 0) - 100 + ((float)EEPROM.read(leg * 6 + 1) - 100) / 100;
  float beta_error  = EEPROM.read(leg * 6 + 2) - 100 + ((float)EEPROM.read(leg * 6 + 3) - 100) / 100;
  float gamma_error = EEPROM.read(leg * 6 + 4) - 100 + ((float)EEPROM.read(leg * 6 + 5) - 100) / 100;

  //  float alpha_error = 0;
  //  float beta_error  = 0;
  //  float gamma_error = 0;

  alpha += alpha_error;
  beta += beta_error;
  gamma += gamma_error;

  if (leg == 0)
  {
    alpha = 90 - alpha;
    beta = beta;
    gamma = 90 + gamma;
  }
  else if (leg == 1)
  {
    alpha += 90;
    beta = 180 - beta;
    gamma = 90 - gamma;
  }
  else if (leg == 2)
  {
    alpha += 90;
    beta = 180 - beta;
    gamma = 90 - gamma;
  }
  else if (leg == 3)
  {
    alpha = 90 - alpha;
    beta = beta;
    gamma = 90 + gamma;
  }

  servo[leg][femur_servo_index].write(alpha);
  servo[leg][tibia_servo_index].write(beta);
  servo[leg][coxa_servo_index].write(gamma);
}

