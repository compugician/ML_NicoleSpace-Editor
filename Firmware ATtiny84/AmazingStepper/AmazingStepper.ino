#include <avr/io.h>
#include <avr/interrupt.h>

//** THIS IS USING: http://members.shaw.ca/climber/avr_usi_i2c.html **//

uint8_t userdata;                        // data to receive and echo from master.

int smDirectionPin = 3; //Direction pin
int smStepPin = 2; //Stepper pin

// Pin 3  & 4 connected to Limit switch out
#define LMT_PIN_UP 0
#define LMT_PIN_DOWN 1
#define PINA_LMT_UP (PINA & 0b00000001) 
#define PINA_LMT_DOWN (PINA & 0b00000010) 

//TODO: check direction
#define DIRECTION_UP 1
#define DIRECTION_DOWN 0

#define HEARTBEAT_LED_PIN 8
#define ERR_LED_PIN 7

int lastDirection;

volatile bool swUp, swDown,pinChanged;

bool homed = false;

long motorPosition; //current position of the motor, only usable if homed. homing sets this to 0 if successful.
float moveSpeed = 0.5;


//Define a reference to the I2C slave register bank pointer array
extern char* USI_Slave_register_buffer[];
unsigned int targetPosition = 0;

void emergencyKill() {
  //turn on LED
  digitalWrite(ERR_LED_PIN,HIGH);
 
  //wait forever!
  while (1) { }
}

void setup() {
  
  //setup pins
  pinMode(HEARTBEAT_LED_PIN,OUTPUT);
  digitalWrite(HEARTBEAT_LED_PIN,LOW);
 
  pinMode(ERR_LED_PIN,OUTPUT);
  digitalWrite(ERR_LED_PIN,LOW);
  
  pinMode(smDirectionPin, OUTPUT);
  pinMode(smStepPin, OUTPUT);

  pinMode(LMT_PIN_UP, INPUT); //TODO: test with internal pullups, to simplify board
  pinMode(LMT_PIN_DOWN, INPUT);

  swUp = false;
  swDown = false;


  //Any logical change on INT0 generates an interrupt request.
  MCUCR = MCUCR & (0b11111100);
  MCUCR += 0b00000001;

  GIMSK |= (1 << PCIE0);
  PCMSK0 |= 0b00000011; //enable pin change interrupts on PA0 and PA1 (pins 13 and 12)

//  SREG |= 0b10000000; //Set SREG bit I to 1  
  sei();     // enable interrupts

  digitalWrite(smDirectionPin, DIRECTION_DOWN);
  lastDirection = DIRECTION_DOWN;

  setupUSI_I2C(0x42); 
   
  seekHome();
}


//Interrupt Service Routine
ISR(PCINT0_vect)
{
  pinChanged=true;
  swUp = (PINA_LMT_UP)!=0;
  swDown = (PINA_LMT_DOWN)!=0;
}

/**
 * This function will step the motor in the given direction, 
 * up to the given number of steps. Interrupted if any of the limit switches CHANGE state.
 * 
 * returns: number of steps actually taken.
 */

int stepMotor(int dir, int steps, float spd) {
  int stepsTaken = 0;
  
  if (lastDirection!=dir) {
    digitalWrite(smDirectionPin, dir); //todo : switch to fast (direct port manipulation)
    lastDirection=dir;
  }
  
  spd = 1 / spd * 70; //Calculating speed
  steps = abs(steps); //Stores the absolute value of the content in 'steps' back into the 'steps' variable

  /*Steppin'*/
  pinChanged = false;
  for (int i = 0; (i < steps) && !pinChanged ; i++) {
    digitalWrite(smStepPin, HIGH);
    delayMicroseconds(spd);
    digitalWrite(smStepPin, LOW);
    delayMicroseconds(spd);
    stepsTaken++;
  }

  return stepsTaken;
}

#define MAX_HOMING_STEPS 10000 
#define MAX_UP_HOMING_BACKOFF 100000
#define HOMING_SPEED 0.2
#define MAX_BACKOFF_STEPS 10000
#define BACKOFF_SPEED 0.05
#define EXTRA_BACKOFF_STEPS 80

void seekHome() {
  int stepsTaken;

/*** VVVV DEBUG ***/
  //check if we are hitting the up button...
  while (swUp && stepsTaken<MAX_UP_HOMING_BACKOFF ) {
    stepsTaken += stepMotor(DIRECTION_DOWN,MAX_UP_HOMING_BACKOFF-stepsTaken,BACKOFF_SPEED);
    delay(5);
  }
  delay(15);
  if (stepsTaken>=MAX_UP_HOMING_BACKOFF || swUp) {
    //we used more steps than expected, or the switch is not set as expected
     emergencyKill();
  }
/**** ^^^^ DEBUG ****/

  if (!swDown) {
    stepsTaken = stepMotor(DIRECTION_DOWN,MAX_HOMING_STEPS,HOMING_SPEED);
  }
  delay(15);
  if (stepsTaken>=MAX_HOMING_STEPS || !swDown) {
    //we used more steps than expected, or the switch is not set as expected
     emergencyKill();
  }

  stepsTaken = stepMotor(DIRECTION_UP,MAX_BACKOFF_STEPS,BACKOFF_SPEED);
  delay(15);

  if (stepsTaken>=MAX_BACKOFF_STEPS || swDown) {
     emergencyKill();
  }

  stepsTaken = stepMotor(DIRECTION_UP,EXTRA_BACKOFF_STEPS,BACKOFF_SPEED);
 
  motorPosition = 0;
  homed = true;
}

void runToPosition() {
  long steps = targetPosition - motorPosition;
  if (steps!=0) {
    int dir = (steps<0) ? DIRECTION_DOWN : DIRECTION_UP;
    int stepsTaken = stepMotor(dir, abs(steps), moveSpeed);
    motorPosition = motorPosition + stepsTaken * ((dir==DIRECTION_DOWN)?-1:1);  
  }
}

int heartrate_delay = 100;
void loop() {  
  //TODO: Notice, right now, run to position is blocking. Meaning, you cannot change direction mid motion... and only the 'latest' target will be used each time... no queue
  runToPosition();
  targetPosition = userdata * 62;

  digitalWrite(HEARTBEAT_LED_PIN,HIGH);
  delay(heartrate_delay);
  digitalWrite(HEARTBEAT_LED_PIN,LOW);
  delay(heartrate_delay);
  
}


