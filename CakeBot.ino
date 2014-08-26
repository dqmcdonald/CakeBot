/*

 "Cakebot" robotics platform main control code. 
 This is designed to run an Arduino "Mega".
 
 Version 1.0 - Initial version. Simply basic distance measuring and motor control.
 
 */
#include <Servo.h>
#include <IRremote.h>


#include <stdlib.h> // for malloc and free
//void* operator new(size_t size) { 
//  return malloc(size); 
//}
//void operator delete(void* ptr) { 
//  free(ptr); 
//} 

// Apple IR Remote codes:
#define IR_PLAY_CODE             0x77E12087
#define IR_RIGHT_ARROW_CODE       0x77E1E087
#define IR_LEFT_ARROW_CODE      0x77E11087
#define IR_INC_SPEED_CODE          0x77E1D087
#define IR_DEC_SPEED_CODE          0x77E1B087
#define IR_MENU_CODE             0x77E14087

// Pin Assignments follow:

#define DS_PIN 4 
#define LEFT_SPEED_PIN 11
#define LEFT_DIR_PIN 13
#define RIGHT_SPEED_PIN  3
#define RIGHT_DIR_PIN  12
#define EYES_LED_PIN 5
#define HEAD_SERVO_PIN 2
#define IR_RECV_PIN A0

#define VOICE_BOX_SPEAKING 7
#define VOICE_BOX_BUFFER_FULL 8
#define VOICE_BOX_READY 9


#define LEFT_TURN 1
#define RIGHT_TURN 2

#define NINETY_DEG_TIME 200
#define REMOTE_TURN_TIME 50

// Set up a memorable token for "Word Pause"
#define WP 6 // 6 is 90ms pause

// Voice box messages:
uint8_t intro[] = {
  183, 7, 159, 146, 164,  WP, WP,    // Hello
  140, 155, WP,                      // my
  141, 154, 140,  WP,                // name
  8, 129, 167,   WP,                 // is
  194, 154, 15, 196, WP,             // Cake
  171, 8, 13,5, 191, 191 , WP         // Bot
};

uint8_t robot_noise[] = {
  200, WP, 201, WP, 202, WP, 203, WP, 204, WP, 205
};

uint8_t ping[] =  {
  252, WP                          // Sonar ping
};

uint8_t cool[] = {
  194, 154, 15, 196, WP,              // Cake
  171, 8, 13,5, 191, 191 , WP,        // Bot
  8, 129, 167,   WP,                 // is
  195,8,139, 139, 139, 145, WP        // Cool
};

uint8_t tooclose[] = { 
  8, 191, 162, WP,                  // Too
  8, 195, 7, 145, 164, 7, 167, WP   // close
};

uint8_t left[] = { 
  145, 131, 186, 191, WP, WP        // Left

};

uint8_t right[] = { 
  148, 7, 155, 7, 128, 191, WP, WP   // Right

};

uint8_t say_go[] = { 
  8, 179, 8, 164, WP, WP   // Go

};

uint8_t say_stop[] = { 
  187, 191, 8, 135, 199, WP, WP // Stop 
};

uint8_t say_turn[] = { 
  8, 191, 151, 141,  WP,WP //Turn
};

uint8_t say_stuck[] = {
  194, 154, 15, 196, WP,             // Cake
  171, 8, 13,5, 191, 191 , WP,       // Bot
  8, 129, 167,   WP,                 // is
  187, 191, 134, 195, 6 , WP          // stuck
};

uint8_t say_free[] = {
  194, 154, 15, 196, WP,             // Cake
  171, 8, 13,5, 191, 191 , WP,       // Bot
  8, 129, 167,   WP,                 // is
  8, 186, 148, 8, 128 , WP            // free
};

uint8_t say_slower[] = { 
 187, 7, 146, 8, 164,  151, 6 , WP, WP // Slower
};

uint8_t say_faster[] = { 
  186, 132, 187, 191, 148 , WP, WP // Faster
};

IRrecv irrecv(IR_RECV_PIN);
decode_results results;


// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

// A distance sensor class, used to encapsulate the Matbotix ultrasonic distance sensor.
class DistanceSensor
{
public:
  DistanceSensor();  

  void setPin( int pin );

  float getDistance();  /// Returns the distance in mm


private:
  int  _ds_pin;
};

DistanceSensor::DistanceSensor( )
{


}

void DistanceSensor::setPin( int pin )
{
  pinMode( pin, INPUT );
  _ds_pin = pin; 

  Serial.print( "Distance measuring sensor initialized with pin: ");
  Serial.println( _ds_pin, DEC);

}

float DistanceSensor::getDistance( ) 
{

  int pw_val;
  float distance;

  pw_val = pulseIn( _ds_pin, HIGH );
  distance = pw_val/147*2.54;

  Serial.print( "Distance sensor pulse distance = " );
  Serial.println(distance, DEC );

  return distance;

}

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

// An LED class. Turns LEDs on and off, can flash them as well

class LED {

public:
  LED( );  
  void setPin( int led_pin );

  void on();
  void off();
  void flash( int num_times=3, int delay_time=400);

private:
  int _led_pin;

};

LED::LED( ) {

  _led_pin = -1; 
}

void LED::setPin( int led_pin ) {
  pinMode( led_pin, OUTPUT);
  _led_pin = led_pin; 

}

void LED::on( ) {
  digitalWrite( _led_pin, HIGH ); 
}

void LED::off() {
  digitalWrite( _led_pin, LOW );
}

void LED::flash( int num_times, int delay_time )  {

  int i;
  for( i=0; i< num_times; i++ ) {
    digitalWrite( _led_pin, HIGH );
    delay( delay_time/2 );
    digitalWrite( _led_pin, LOW );
    delay( delay_time/2);
  }
}


// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

// A motor class. Has the ability to set the speed an direction for a motor:

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

class Motor {

public:
  Motor( );
  void setPins( int speed, int direction );

  void forward();
  void reverse();
  void speed( int speed );

private:
  int _speed_pin;
  int _direction_pin;
  int _speed;
};

Motor::Motor() {
  _speed_pin = -1;
  _direction_pin = -1;


}

void Motor::setPins( int speed, int direction ) {
  _speed_pin  = speed;
  _direction_pin = direction; 
  pinMode( _speed_pin, OUTPUT );
  pinMode( _direction_pin, OUTPUT );
  return;
}

// Set the motor to run forward
void Motor::forward( ) {
  if( _direction_pin > 0 ) {
    digitalWrite( _direction_pin, LOW  );
  }

}
// Set the motor to run backwards
void Motor::reverse( ) {
  if( _direction_pin > 0 ) {
    digitalWrite( _direction_pin, HIGH  );
  }
}

// Set the speed - value in the range 1-100
void Motor::speed( int speed ) {
  if( _speed_pin > 0 && speed >= 0 && speed <= 100) {
    analogWrite( _speed_pin, int(float(speed)/100 * 255) );   
  }
  if( _speed_pin > 0 && speed == 0 ) {
    analogWrite( _speed_pin, 0 ); 
  }

} 

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

// A SpeakJet based speech synthesier class

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

class VoiceBox {
public:
  VoiceBox();
  void speak( uint8_t message[], int len );

private:
  int d_rx_pin;
  int d_tx_pin;
};



VoiceBox::VoiceBox( ) {


  Serial1.begin(9600); // The SpeakJet defaults to 9600bps


  // Send the SpeakJet some initialisation values
  Serial1.write(20); // Enter volume set mode
  Serial1.write(96); // Set volume to 96 (out of 127)
  Serial1.write(21); // Enter speed set mode
  Serial1.write(114); // Set speed to 114 (out of 127)


}

void VoiceBox::speak( uint8_t message[], int len ) {
  int i;
  for (i=0; i<len; i++)
  {
    Serial1.write(message[i]);
  }
}

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

// The main robot class:

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


class CakeBot {

public:
  CakeBot();
  void setup(int distance_sensor_pin, int left_dir_pin, int left_speed_pin, int right_dir_pin, 
  int right_speed_pin, int eyes_pin, int head_servo_pin );
  void loop ();
  int lookBothWays( );
  void turn180();
  void turn( int direction, int turn_time);
  void backup( void );

protected:
  DistanceSensor d_distance_sensor;
  Motor d_left_motor;
  Motor d_right_motor;
  LED d_eyes;
  Servo d_head;
  VoiceBox* d_voicebox_p;

  int loop_cnt;

  boolean d_running;  // Whether we are currently running
  int d_speed;          // The current running speed.
  int d_menu_count;      // The number of times the menu button has been pressed
};

CakeBot::CakeBot()
{

}

void CakeBot::setup(  int distance_sensor_pin, int left_dir_pin, int left_speed_pin, int right_dir_pin, 
int right_speed_pin, int eyes_pin, int head_servo_pin )
{
  d_distance_sensor.setPin( distance_sensor_pin );
  d_left_motor.setPins( left_speed_pin, left_dir_pin);
  d_right_motor.setPins( right_speed_pin, right_dir_pin);
  d_eyes.setPin( eyes_pin );
  d_head.attach( head_servo_pin);
  d_voicebox_p = new VoiceBox( );

  d_left_motor.speed( 0 );
  d_right_motor.speed( 0 );

  d_voicebox_p->speak( intro, sizeof(intro) );
  d_voicebox_p->speak( robot_noise, sizeof(robot_noise) ); 
  // flash d_eyes, turn d_head to each side with flashing d_eyes:
  lookBothWays();


  d_running = false; // Start with not running

  loop_cnt = 0;
  d_speed = 90;
  d_menu_count = 0;
  return;
}


// All updating for the cakebot is done here. Called everytime through the loop function:
void CakeBot::loop( )
{
  float distance;
  int turn_dir;

  if (irrecv.decode(&results)) {

     d_eyes.flash(1); // Flashyes to acknowldge IR
      
    
    // Play/pause is used to stop or start the robot running:
    if( results.value == IR_PLAY_CODE ) {

      
      if( d_running ) {

        // Stop motors:
        d_left_motor.forward();
        d_right_motor.forward();
        d_left_motor.speed( 0 );
        d_right_motor.speed( 0 );

        d_voicebox_p->speak( say_stop, sizeof(say_stop));
        d_running = false;
      } 
      else {

        // Start rolling forward:
        d_left_motor.forward();
        d_right_motor.forward();
        d_left_motor.speed( d_speed );
        d_right_motor.speed( d_speed );
        d_voicebox_p->speak( say_go, sizeof(say_go));
        d_running = true;  
      }
    } 
    else if( results.value == IR_INC_SPEED_CODE ) {
       d_voicebox_p->speak( say_faster, sizeof(say_faster));
      d_speed += 5;
      if( d_speed > 100 ) {
        d_speed = 100; 
      }
      if( d_running ) {
        d_left_motor.speed( d_speed );
        d_right_motor.speed( d_speed );

      }

    } 
    else if( results.value == IR_DEC_SPEED_CODE ) {
      d_voicebox_p->speak( say_slower, sizeof(say_slower));
      d_speed -= 5;
      if( d_speed < 0 ) {
        d_speed = 0;      
      } 
      if( d_running ) {
        d_left_motor.speed( d_speed );
        d_right_motor.speed( d_speed );

      }
    }
    else if( results.value == IR_LEFT_ARROW_CODE ) {
      d_left_motor.speed( 0 );
      d_right_motor.speed(0 );
      d_voicebox_p->speak( say_turn, sizeof(say_turn));
      d_voicebox_p->speak( left, sizeof(left));
      turn(LEFT_TURN, REMOTE_TURN_TIME);
      if( d_running) {
        d_left_motor.speed( d_speed );
        d_right_motor.speed( d_speed );
      }

    }
    else if( results.value == IR_RIGHT_ARROW_CODE ) {
      d_left_motor.speed( 0 );
      d_right_motor.speed(0 );
      d_voicebox_p->speak( say_turn, sizeof(say_turn));
      d_voicebox_p->speak( right, sizeof(right));
      turn(RIGHT_TURN, REMOTE_TURN_TIME );
      if( d_running) {
        d_left_motor.speed( d_speed );
        d_right_motor.speed( d_speed );
      }
    }
    else if( results.value == IR_MENU_CODE ) {

      if( d_menu_count % 2 == 0 ) {
        d_voicebox_p->speak( say_stuck, sizeof(say_stuck));
      } 
      else {
        d_voicebox_p->speak( say_free, sizeof(say_free));
      }
      d_menu_count += 1;
    }


    Serial.println(results.value, HEX);
    irrecv.resume(); // Receive the next value
  }


  if( d_running ) {
    distance = d_distance_sensor.getDistance();

    if( distance < 30 ) {
      // Too close to object, stop and reverse:

      backup();


      d_voicebox_p->speak( tooclose, sizeof(tooclose));

      // Figure out which way we want to turn:
      turn_dir = lookBothWays();
      turn( turn_dir, NINETY_DEG_TIME );

      if( turn_dir == LEFT_TURN ) {
        d_voicebox_p->speak( left, sizeof( left ) );
      } 
      else {
        d_voicebox_p->speak( right, sizeof( right ) );
      }
      delay(100);
      distance = d_distance_sensor.getDistance();
      d_left_motor.speed(d_speed);
      d_right_motor.speed(d_speed);

    } 
    else {
      d_left_motor.speed(d_speed);
      d_right_motor.speed(d_speed);

    }


    loop_cnt ++;
    if( loop_cnt > 5000 ) {
      loop_cnt = 0;
    }

    if( loop_cnt % 30 == 0 ) {
      int r = random(100);
      if( r < 30 ) {
        d_voicebox_p->speak( robot_noise, sizeof(robot_noise) ); 
      } 
      else if( r < 60 ) {
        d_voicebox_p->speak( cool, sizeof( cool ));
      }
    }

  }

}

// Stop and backup a short distance:
void CakeBot::backup() {

  d_left_motor.speed(0);
  d_right_motor.speed(0);

  d_left_motor.reverse();
  d_right_motor.reverse();
  d_right_motor.speed(100);
  d_left_motor.speed(100);
  delay(400);
  d_left_motor.speed(0);
  d_right_motor.speed(0);
  d_left_motor.forward();
  d_right_motor.forward();

}


// Turn d_head left, right and then center, flashing d_eyes at each point. Measure the distance and
// return the direction with the longest distance to an obstacle.
int CakeBot::lookBothWays() {
  float dist_right, dist_left;

  d_eyes.flash(5,400 );
  d_head.write(175);
  delay(500); // Delay to let head turn
  dist_right = d_distance_sensor.getDistance();
  d_voicebox_p->speak( ping, sizeof(ping));
  d_eyes.flash();
  d_head.write( 5 );
  delay(500); // Delay to let head turn
  dist_left = d_distance_sensor.getDistance();
  d_voicebox_p->speak( ping, sizeof(ping));
  d_eyes.flash();
  d_head.write(90);
  delay(250);
  if( dist_right > dist_left )
    return RIGHT_TURN;
  else
    return LEFT_TURN;


}

// Turn 180 deg:
void CakeBot::turn180( void ) {

  d_left_motor.forward();
  d_right_motor.reverse();
  d_left_motor.speed( 100 );
  d_right_motor.speed( 100 );
  delay(400);
  d_left_motor.speed(0);
  d_right_motor.speed(0);
  d_left_motor.forward();
  d_right_motor.forward();
  return;
}

// Turn left or right
void CakeBot::turn( int dir, int delay_time ) {

  if( dir == LEFT_TURN ) {
    d_left_motor.forward();
    d_right_motor.reverse();
  } 
  else {
    d_left_motor.reverse();
    d_right_motor.forward();
  }
  d_left_motor.speed( 100 );
  d_right_motor.speed( 100 );
  delay(delay_time);
  d_left_motor.speed(0);
  d_right_motor.speed(0);
  d_left_motor.forward();
  d_right_motor.forward();
  return;
}

// variables:
CakeBot cakebot;


// Setup and loop follow:

void setup() {

  Serial.begin(9600);
  cakebot.setup( DS_PIN, LEFT_DIR_PIN, LEFT_SPEED_PIN, RIGHT_DIR_PIN, RIGHT_SPEED_PIN, EYES_LED_PIN, 
  HEAD_SERVO_PIN);

  irrecv.enableIRIn(); // Start the receiver
}


void loop() {

  cakebot.loop();

}






















































