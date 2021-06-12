#include <Arduino.h>
#include <Encoder.h>
#include <ClickEncoder.h>
#include <TimerOne.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

const float version = 1.2;

//Optimized encoder Switch read, removed function from main and included in function to change the feedrate
//Added Aceleration profiles 
//Added High Speed mode, keep the encoder pressed and the speed will go to high speed
//Added configuration menu, to select settings and save them in EEPROM

//Define functions for bit manipulation
#define BIT_SET(a,b) ((a) |= (1ULL<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1ULL<<(b)))
#define BIT_FLIP(a,b) ((a) ^= (1ULL<<(b)))
#define BIT_CHECK(a,b) (!!((a) & (1ULL<<(b))))  

/**-------------------------------Definition Fixed Values-----------------------------**/
#define RIGHT 0
#define LEFT 1

#define ENABLE 1
#define DISABLE 0
/**-----------------------------------------------------------------------------------**/


/**---------------------------------Setting definition--------------------------------**/
const int ENCODER_STEPS_PER_NOTCH = 4;  // Change this depending on which encoder is used

long Arduino_clk = 16000000;  //Oscillator frequency of arduino board (Nano 16MHz)

//Structure for settings values
struct __attribute__ ((packed)) settings {
  float Motor_stepangle;    //This value is defined in the specs of the motor
  int Driver_stepsrev;      //This value define the microstteping which you have selected in your driver
  float Machine_lead;       //mm/rev this is the mm which your axis moves each turn of your leadscrew (In my case 2mm each rev)  
  int Motor_aceleration;    //Value for aceleration in mm/min^2 (Limited to 200mm/min^2)
  int high_speed_feedrate;  //Value for speed in High speed mode (Encoder SW held)
};
settings value = { 1.8 , 6400 , 2 , 200, 400};  //Assign default values (Will be saved on EEPROM)

struct settings *ptr_settings;  //Pointer to modify the structure

bool DIR_Inv  = 1;    //Use this value to Invert or not the DIRection Pin function. (In case the direction of the power feed is inverted)
bool EN_Inv  = 0;     //Use this value to Invert or not the ENable Pin function.   
/**-----------------------------------------------------------------------------------**/


/**--------------------------------Function Declaration-------------------------------**/

void disable_pulse_output(void);
void set_speed_mmmin(int feedrate);
void stepper_enable(bool enable);
void direction_selection(bool direction);
int check_switch_status(void);
void print_main_page(int feedrate);
int read_encoder_feedrate(int feed_rate);
void lcd_clear(int row, int start_pos, int end_pos);
int read_encoderswitch(void);
void draw_arrow(int direction);
float aceleration_feedrate(float current_feedrate, int target_feedrate);
int read_encoder(void);
void eeprom_save(struct settings value);
void eeprom_read(struct settings *ptr_struct);

/**-----------------------------------------------------------------------------------**/


/**---------------------------------Pinout definition---------------------------------**/
//Modify this values according your connectionts, just dont move encoder pins and PULSE pin they are fixed
const int pinA = 2;         //Encoder input pin A (If selected 2/3 works with interrupt) (Do not change)
const int pinB = 3;         //Encoder input pin B (If selected 2/3 works with interrupt) (Do not change)
const int PULSE =11;        //Pulse signal for Stepper driver (Do not change)    
const int EN = 6;           //Enable signal for Stepper driver
const int DIR = 7;          //Direction signal for stepper driver
const int EN_sw = 10;       //Encoder switch
const int SW_left = 9;      //Input switch signal for left movement
const int SW_right = 12;    //Input switch signal for right movement
/**-----------------------------------------------------------------------------------**/


/**-----------------------------Peripheral Initialization-----------------------------**/
LiquidCrystal_I2C lcd(0x20,16,2);  // Set the LCD address to 0x20 for a 16 chars and 2 line display

ClickEncoder encoder(pinA, pinB, EN_sw, ENCODER_STEPS_PER_NOTCH); // Enable encoder in pinA pinB and SW Steps selected to 4 

// Interruption call for Encoder
void timerIsr() {
  encoder.service();
}
/**-----------------------------------------------------------------------------------**/


/**-------------------------------Variable Initialization-----------------------------**/
//Declaration of states in main state machine
const int INIT = 1;           
const int STANDBY = 2; 
const int MOVING_RIGHT = 3;
const int MOVING_LEFT = 4;
const int LOCK = 5;
const int CONFIG = 6;
//Initalization state variables for main state machine
int state = INIT;           
int previous_state = INIT;  

//Initialization main variables 
int feedrate = 0;           //Feedrate currently selected
int new_feedrate = 5;       //Variable for new feedrate selection
int saved_feedrate = 5;     //Variable to save the current feedrate in high speed mode
int encoder_sw_status = 0;  //Variable for the status of the encoder switch
bool change_mode = 0;       //Variable to select the mode between x1 and x10 in the encoder changing the feedrate
int oldEncPos, encPos;      //Variables for encoder position
int switch_status = 1;      //Default value for Switch position
bool speed_mode = 0;        //Variable to select if we are in high speed mode or not
int submenu = 0;            //Variable for states in submenu
int selected_value = 0;     //VAriable for Config Menu, selecting values
bool keep_loop = 0;         //Variable to keep the loop in config menu
int encoder_turns = 0;      //VAriable to count the encoder turns
int temp_encoder_turns = 0;
bool save_confirmation = 0; //Variable for save menu

float test_feedrate = 0;
/**-----------------------------------------------------------------------------------**/


/**----------------------------------------SETUP--------------------------------------**/
void setup() {

  //Serial Interface enable for debugging
  Serial.begin(9600);                       //Selected speed 9600bps                 
  Serial.println("Basic Encoder Test:");    //Check COM is working

  //PinMode for selected pins 
  pinMode(SW_left, INPUT);      
  pinMode(SW_right, INPUT);
  pinMode(DIR, OUTPUT);
  pinMode(EN, OUTPUT);
  pinMode(PULSE, OUTPUT);

  //LCD Initialization
  lcd.init();         //Initialize the lcd 
  lcd.backlight();    //LCD Backlight Enable

  //Encoder Initialization
  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr);         //Interrupt assigned to Timer1
  encoder.setAccelerationEnabled(false);    //Aceleration mode not used for this application

  //Configuration of TIMER2, used to generate the pulse for Driver on PULSE pin.
  //Configure OC0B Toggle on Compare Match
  TCCR2A &= ~_BV(COM2A1);   //Clear COM0B1
  TCCR2A |= _BV(COM2A0);    //Set   COM0B0
  // Configure as CTC mode  
  TCCR2B &= ~_BV(WGM22);    //Clear WGM22
  TCCR2A |= _BV(WGM21);     //Set WGM21
  TCCR2A &= ~_BV(WGM20);    //Clear WGM20  
  //Disable output of TIMER2 (Will be enabled later, just for initialization)
  BIT_CLEAR(TCCR2B, CS20);
  BIT_CLEAR(TCCR2B, CS21);
  BIT_CLEAR(TCCR2B, CS22);
  }
/**-----------------------------------------------------------------------------------**/


/**--------------------------------------MAIN LOOP------------------------------------**/
void loop() {

switch (state) {
  case INIT:      //First state INIT 
    ptr_settings = &value;        //Asign address from struc value to pointer
    eeprom_read(ptr_settings);    //Read the values from the EEPROM and substitute the default values
    stepper_enable(DISABLE);      //Keep the motor disabled
    lcd.clear();                  //Clear LCD content
    lcd.setCursor(2,0);           
    lcd.print("Ar_PowerFeed");    //Name Project Printing
    lcd.setCursor(4,1);
    lcd.print("Ver:");            //Version Printing
    lcd.print(version);
    delay(2000);
    lcd.clear();                  //Clear LCD content
    previous_state = INIT;        //Update previous state to INIT the current one
    switch_status = check_switch_status();  //Checking the status of the switch, should be not enabled
    switch (switch_status){
      case 0:             //Unknow status on the switch 
        state = LOCK;     //Going to LOCK
        break;
      case 1:             //Switch status OPEN Correct
        if (encoder.getButton() == 3){     //Held the encoder enter in CONFIG
          state = CONFIG;  //Going to CONFIG
          oldEncPos = 0;
          encPos = 0;
        }else{
          state = STANDBY;  //Going to STANDBY
        }
        break;
      case 2:             //Switch status RIGHT POSITION 
        state = LOCK;     //Going to LOCK
        break; 
      case 3:             //Switch status LEFT POSITION 
        state = LOCK;     //Going to LOCK
        break;
    }
    break;
  case STANDBY:     //STANDBY Main status when there is no movement.
    if (speed_mode == 1){             //If we were in speed mode, remove and clean the variables
      speed_mode = 0;
      new_feedrate = saved_feedrate;  //Recover the previous feedrate
      print_main_page(new_feedrate);
    }
    test_feedrate = aceleration_feedrate(test_feedrate, 0); //Set the feedrate according to the aceleration
    if (test_feedrate >= 1){
      set_speed_mmmin((int)test_feedrate);    //Enable the PULSE Output and addapt the value
    }else{
      disable_pulse_output();       //Disable the PULSE output
      stepper_enable(DISABLE);      //Disable Stepper Driver
      test_feedrate = 0;            //Feedrate set to Zero, deceleration completed
    }
    if (feedrate != new_feedrate){  //If new change in the feedrate update the feedrate variable and print the new value
    feedrate = new_feedrate;    
    print_main_page(feedrate);      //Printing feedrate value
    }
    new_feedrate = read_encoder_feedrate(feedrate); //Check if there is change in the feedrate
    if (state != previous_state){   //Check if we came from other state
      lcd.clear();                
      print_main_page(feedrate);
      lcd.setCursor(0,1);
      lcd.print("STOP");
    }
    switch_status = check_switch_status(); //Checking the status of the switch
    switch (switch_status){
      case 0:
        //ERROR on SWITCH
        break;
      case 1:                       //Switch status OPEN Correct
        break;
      case 2:                       //Switch status RIGHT POSITION 
        previous_state = STANDBY;   
        state = MOVING_RIGHT;       //Going to MOVING RIGHT
        break;
      case 3:                       //Switch status LEFT POSITION 
        previous_state = STANDBY;
        state = MOVING_LEFT;        //Going to MOVING LEFT
        break;
    }
    previous_state = STANDBY;       //Move the previous state to STANDBY
    break;
  case MOVING_RIGHT:      //Status to move the axis to the RIGHT  
    if (encoder.getButton() == 3 && speed_mode == 0){     //Held the encoder sw and we were not in speed mode, enter in speed mode
      saved_feedrate = new_feedrate;        //Storage the new feedrate
      new_feedrate = value.high_speed_feedrate;   //Set the high speed feedrate
      speed_mode = 1;                       //Enable the speed mode
    }else if (encoder.getButton() == 0 && speed_mode == 1){   //Release encoder sw
      speed_mode = 0;                       //Disable the speed mode
      new_feedrate = saved_feedrate;        //Recover feedrate value
      print_main_page(new_feedrate);
    }
    test_feedrate = aceleration_feedrate(test_feedrate, new_feedrate); //Set the feedrate according to the aceleration
    if (test_feedrate != new_feedrate){
      set_speed_mmmin((int)test_feedrate);    //Enable the PULSE Output and addapt the value
    }
    if (feedrate != new_feedrate){  //If new change in the feedrate update the feedrate variable and print the new value
      feedrate = new_feedrate;
      print_main_page(new_feedrate);
    }
    new_feedrate = read_encoder_feedrate(feedrate);  //Check if there is change in the feedrate
    if (state != previous_state){   //Check if we came from other state
      lcd.clear();
      print_main_page(feedrate);
      lcd.setCursor(0,1);
      lcd.print("RIGHT");
      direction_selection(RIGHT);   //Select direction pin to RIGHT
      stepper_enable(ENABLE);       //Enable EN pin
    }
    draw_arrow(RIGHT);              //Draw arrow in the menu to the right
    switch_status = check_switch_status();  //Checking the status of the switch
    switch (switch_status){
      case 0:
        //ERROR on SWITCH
        break;
      case 1:                           //Switch status OPEN go to STANDBY
        previous_state = MOVING_RIGHT;
        state = STANDBY;                //Move to STANDBY status
        break;
      case 2:                           //Keep status RIGHT POSITION 
        previous_state = MOVING_RIGHT;
        state = MOVING_RIGHT;
        break;
      case 3:                           //Switch status LEFT POSITION 
        previous_state = MOVING_RIGHT;
        state = MOVING_LEFT;            //Move to MOVING LEFT status
        break;
    }
    previous_state = MOVING_RIGHT;
    break;
  case MOVING_LEFT:     //Status to move the axis to the LEFT
    if (encoder.getButton() == 3 && speed_mode == 0){   //Held the encoder sw and we were not in speed mode, enter in speed mode
      saved_feedrate = new_feedrate;        //Storage the new feedrate
      new_feedrate = value.high_speed_feedrate;   //Set the high speed feedrate
      speed_mode = 1;                       //Enable the speed mode
    }else if (encoder.getButton() == 0 && speed_mode == 1){ //Release encoder sw
      speed_mode = 0;                       //Disable the speed mode
      new_feedrate = saved_feedrate;        //Recover feedrate value
      print_main_page(new_feedrate);
    }
    test_feedrate = aceleration_feedrate(test_feedrate, new_feedrate);  //Set the feedrate according to the aceleration
    if (test_feedrate != new_feedrate){
      set_speed_mmmin((int)test_feedrate);    //Enable the PULSE Output and addapt the value
    }
    if (feedrate != new_feedrate){  //If new change in the feedrate update the feedrate variable and print the new value
      feedrate = new_feedrate;
      print_main_page(new_feedrate);
      //set_speed_mmmin(feedrate);    //Enable the PULSE Output and addapt the value
    }
    new_feedrate = read_encoder_feedrate(feedrate);  //Check if there is change in the feedrate
    if (state != previous_state){   //Check if we came from other state
      lcd.clear();
      print_main_page(feedrate);
      lcd.setCursor(0,1);
      lcd.print("LEFT");
      //set_speed_mmmin(feedrate);    //Enable the PULSE Output and addapt the value after come from other state
      direction_selection(LEFT);    //Select direction pin to LEFT
      stepper_enable(ENABLE);       //Enable EN pin
    }
    draw_arrow(LEFT);              //Draw arrow in the menu to the left
    switch_status = check_switch_status();  //Checking the status of the switch
    switch (switch_status){
      case 0:
        //ERROR on SWITCH
        break;
      case 1:                           //Switch status OPEN go to STANDBY
        previous_state = MOVING_LEFT;
        state = STANDBY;                //Move to STANDBY status
        break;
      case 2:                           //Move status to RIGHT POSITION 
        previous_state = MOVING_LEFT;
        state = MOVING_RIGHT;           //Move to MOVING RIGHT status
        break;
      case 3:                           //Keep status LEFT POSITION 
        previous_state = MOVING_LEFT;
        state = MOVING_LEFT;
        break;
    }
    previous_state = MOVING_LEFT;
    break;
  case LOCK:      //Status reached when the switch is not in middle position (STOP POSITION)
    if (state != previous_state){    
      lcd.clear();                  //Clear LCD content
      lcd.setCursor(0,0);           
      lcd.print("ERR: MOVE SWITCH");    
      lcd.setCursor(0,1);           
      lcd.print("TO STOP POSITION:"); 
    }
    previous_state = LOCK;
    state = LOCK;
    switch_status = check_switch_status();  //Checking the status of the switch, should be not enabled
    switch (switch_status){
      case 0:             //Unknow status on the switch 
        break;
      case 1:             //Switch status OPEN Correct
        state = STANDBY;  //Going to STANDBY
        break;
      case 2:             //Switch status RIGHT POSITION 
        break; 
      case 3:             //Switch status LEFT POSITION 
        break;
      }
    break;
  case CONFIG:    //Status to enter in CONFIG menu to modify the parameters
    switch (submenu){   //Submenu for different values
      case 0:     //Setup for Driver STEP/REV
        if (!keep_loop){
          lcd.clear();                    //Clean LCD
          lcd.setCursor(0,0);             //Set cursor to init
          lcd.print("Driver STEP/REV");   //Print required text
          lcd.setCursor(0,1);             //Move cursor second line to print the value
          lcd.print(value.Driver_stepsrev); //Print current value
          keep_loop = 1;                  //Move keep loop to 1 to avoid repeat this printing
        }else{
          encoder_turns = read_encoder(); //Read the value of the encoder (Increment)
          if (encoder_turns != 0){        //If the encoder value changed
            value.Driver_stepsrev = (encoder_turns * 100) + value.Driver_stepsrev;  //Function to increase the value (x100)
            value.Driver_stepsrev = constrain(value.Driver_stepsrev,100,60000);     //Limit the value between 100 to 60000
            if (value.Driver_stepsrev < 1000){  //Clear the LCD cells when number size change
              lcd_clear(1,3,5);
            }else if (value.Driver_stepsrev < 10000){
              lcd_clear(1,4,6);
            }
            lcd.setCursor(0,1);
            lcd.print(value.Driver_stepsrev);   //Print again the updated value
            encoder_turns = 0;            //Reset the encoder turns value after encoder movement
          }
          if (encoder.getButton() == 5){    //Check if we have push the buttom to save and continue
            submenu++;        //Increase the submenu, change to next setting
            keep_loop = 0;    //Remove the loop lock
          }
        }
        break;
      case 1:     //Setup for LeadScrew mm/REV
        if (!keep_loop){
          lcd.clear();                    //Clean the LCD
          lcd.setCursor(0,0);             //Set cursor to INIT
          lcd.print("LeadScrew mm/REV");  //Print required text
          lcd.setCursor(0,1);             //Move cursor second line to print the value
          lcd.print(value.Machine_lead,2);  //Print current value
          keep_loop = 1;                  //Move keep loop to 1 to avoid repeat this printing
        }else{
          encoder_turns = read_encoder(); //Read the value of the encoder (Increment)
          if (encoder_turns != 0){        //If the encoder value changed
            value.Machine_lead = ((float) encoder_turns * 0.1) + value.Machine_lead; //Function to increase the value (x0.1)
            value.Machine_lead = constrain(value.Machine_lead,0.1,20);      //Limit the value between 0.1 to 20
            if (value.Machine_lead < 10){ //Clear the LCD cells when number size change
              lcd_clear(1,3,5);
            }
            lcd.setCursor(0,1);
            lcd.print(value.Machine_lead,2);  //Print again the updated value
            encoder_turns = 0;            //Reset the encoder turns value after encoder movement
          }
          if (encoder.getButton() == 5){  //Check if we have push the buttom to save and continue
            submenu++;        //Increase the submenu, change to next setting
            keep_loop = 0;    //Remove the loop lock
          }
        }
        break;
      case 2:     //Setup for SpeedMode mm/min
        if (!keep_loop){
          lcd.clear();                    //Clean the LCD
          lcd.setCursor(0,0);             //Set cursor to INIT
          lcd.print("SpeedMode mm/min");  //Print required text
          lcd.setCursor(0,1);             //Move cursor second line to print the value
          lcd.print(value.high_speed_feedrate); //Print current value
          keep_loop = 1;                  //Move keep loop to 1 to avoid repeat this printing
        }else{
          encoder_turns = read_encoder(); //Read the value of the encoder (Increment)
          if (encoder_turns != 0){        //If the encoder value changed
            value.high_speed_feedrate = ((float) encoder_turns * 10) + value.high_speed_feedrate; //Function to increase the value (x10)
            value.high_speed_feedrate = constrain(value.high_speed_feedrate,10,500);  //Limit the value between 10 to 50
            if (value.Machine_lead < 10){ //Clear the LCD cells when number size change
              lcd_clear(1,2,5);
            }
            lcd.setCursor(0,1);
            lcd.print(value.high_speed_feedrate); //Print again the updated value
            encoder_turns = 0;            //Reset the encoder turns value after encoder movement
          }
          if (encoder.getButton() == 5){  //Check if we have push the buttom to save and continue
            submenu++;        //Increase the submenu, change to next setting
            keep_loop = 0;    //Remove the loop lock
          }
        }
        break;
      case 3:     //Setup for SpeedMode Acel mm/min^2
        if (!keep_loop){
          lcd.clear();                    //Clean the LCD
          lcd.setCursor(0,0);             //Set cursor to INIT
          lcd.print("Acel mm/min^2");     //Print required text
          lcd.setCursor(0,1);             //Move cursor second line to print the value
          lcd.print(value.Motor_aceleration); //Print current value
          keep_loop = 1;                  //Move keep loop to 1 to avoid repeat this printing
        }else{
          encoder_turns = read_encoder(); //Read the value of the encoder (Increment)
          if (encoder_turns != 0){        //If the encoder value changed
            value.Motor_aceleration = ((float) encoder_turns * 10) + value.Motor_aceleration; //Function to increase the value (x10)
            value.Motor_aceleration = constrain(value.Motor_aceleration,10,200);  //Limit the value between 10 to 200
            if (value.Machine_lead < 10){  //Clear the LCD cells when number size change
              lcd_clear(1,2,5);
            }else if (value.Machine_lead < 100){
              lcd_clear(1,3,5);
            }
            lcd.setCursor(0,1);
            lcd.print(value.Motor_aceleration); //Print again the updated value
            encoder_turns = 0;              //Reset the encoder turns value after encoder movement
          }
          if (encoder.getButton() == 5){  //Check if we have push the buttom to save and continue
            submenu++;        //Increase the submenu, change to next setting
            keep_loop = 0;    //Remove the loop lock
          }
        }
        break;
      case 4:     //Setup for Step Angle Motor
        if (!keep_loop){
          lcd.clear();                    //Clean the LCD
          lcd.setCursor(0,0);             //Set cursor to INIT  
          lcd.print("Step Angle Motor");  //Print required text
          lcd.setCursor(0,1);             //Move cursor second line to print the value
          lcd.print(value.Motor_stepangle,2); //Print current value
          keep_loop = 1;                  //Move keep loop to 1 to avoid repeat this printing
        }else{
          encoder_turns = read_encoder(); //Read the value of the encoder (Increment)
          if (encoder_turns != 0){        //If the encoder value changed
            value.Motor_stepangle = ((float) encoder_turns * 0.1) + value.Motor_stepangle;  //Function to increase the value (x0.1)
            value.Motor_stepangle = constrain(value.Motor_stepangle,0.2,2);   //Limit the value between 0.2 to 2
            lcd.setCursor(0,1);
            lcd.print(value.Motor_stepangle); //Print again the updated value
            encoder_turns = 0;            //Reset the encoder turns value after encoder movement
          }
          if (encoder.getButton() == 5){  //Check if we have push the buttom to save and continue
            submenu++;        //Increase the submenu, change to next setting
            keep_loop = 0;    //Remove the loop lock
          }
        }
        break;
      case 5:     //Final menu to select save or not the values introduced.
        if (!keep_loop){
          lcd.clear();                    //Clean the LCD
          lcd.setCursor(0,0);             //Set cursor to INIT 
          lcd.print("Save Changes?");     //Print required text
          lcd.setCursor(0,1);             //Move cursor second line to print the value
          lcd.print("NO");                //Print current value
          keep_loop = 1;                  //Move keep loop to 1 to avoid repeat this printing
          encoder_turns = 2;              //Change to even number to start with NO option
        }else{
          encoder_turns = read_encoder(); //Read the value of the encoder 
          temp_encoder_turns = temp_encoder_turns + encoder_turns*1;  //Function to increase the value (x1)
          if (temp_encoder_turns % 2 != 0){   //If the number is EVEN
              lcd.setCursor(0,1);         //Set cursor to INIT 
              lcd.print("YES");           //Print required text
              if (encoder.getButton() == 5){ //If buttom is pressed, values will be saved
                submenu = 6;    //Switch to submenu to show SAVED Message
                keep_loop = 0;  //Release the loop
                eeprom_save(value); //Save on eeprom
              }
            }else{  //If the number is ODD
              lcd_clear(1,2,5);     //Clear LCD cells
              lcd.setCursor(0,1);   //Set cursor to INIT 
              lcd.print("NO");      //Print required text
              if (encoder.getButton() == 5){  //If buttom is pressed, values will not be saved
                submenu = 7;        //Switch to submenu to show NOT SAVED Message
                keep_loop = 0;      //Release the loop
              }
            }
          }
        break;
      case 6:     //Case to show that the vales has been saved
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Values Saved");  //Print required text
          delay(2000);
          state = INIT;   //Return to INIT
        break;
      case 7:     //Case to show that the vales has NOT been saved
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Values Discarded");  //Print required text
          delay(2000);
          state = INIT;   //Return to INIT
        break;
      }
    break;
  } 
}
/**-----------------------------------------------------------------------------------**/


/**--------------------------------------Functions------------------------------------**/

//Function to Disable PULSE output on Timer2 peripheral
void disable_pulse_output(void)
{
  BIT_CLEAR(TCCR2B, CS20);
  BIT_CLEAR(TCCR2B, CS21);
  BIT_CLEAR(TCCR2B, CS22);
}

//Function to calculate the value for Timer2 to stablish the correct feedrate
//Input: rate in mm/min
void set_speed_mmmin(int rate)
{
  float freq = 0; //Variable to calculate the frequency required
  int Prescale[7] = {1, 8, 32, 64, 128, 256, 1024}; //Array with the Prescale values
  long ORC_value = 256; //Variable for the register value
  int Preescalefinal = 0; 

  freq = ( (float)rate * (float)value.Driver_stepsrev)/(60 * value.Machine_lead);  //Formula to calculate the frequency

  for (int i=0 ; ORC_value>255 ; i++){    // For to calculate which Prescale is required. OCR max value 255, so if it is higher, we need to use bigger prescaler
  ORC_value = (( Arduino_clk/(2* Prescale[i]* freq))-1) ; //Formula to calculate the ORC register value
  Preescalefinal = i;
  }
  switch (Preescalefinal){
    case 0:   //Prescale 1
      BIT_SET(TCCR2B, CS20);
      BIT_CLEAR(TCCR2B, CS21);
      BIT_CLEAR(TCCR2B, CS22);
      break;  
    case 1:   //Prescale 8
      BIT_CLEAR(TCCR2B, CS20);
      BIT_SET(TCCR2B, CS21);
      BIT_CLEAR(TCCR2B, CS22);
      break;
    case 2:   //Prescale 32
      BIT_SET(TCCR2B, CS20);
      BIT_SET(TCCR2B, CS21);
      BIT_CLEAR(TCCR2B, CS22);
      break;
    case 3:   //Prescale 64
      BIT_CLEAR(TCCR2B, CS20);
      BIT_CLEAR(TCCR2B, CS21);
      BIT_SET(TCCR2B, CS22);
      break;
    case 4:   //Prescale 128
      BIT_SET(TCCR2B, CS20);
      BIT_CLEAR(TCCR2B, CS21);
      BIT_SET(TCCR2B, CS22);
      break;
    case 5:   //Prescale 256
      BIT_CLEAR(TCCR2B, CS20);
      BIT_SET(TCCR2B, CS21);
      BIT_SET(TCCR2B, CS22);
      break;
    case 6:   //Prescale 1024
      BIT_SET(TCCR2B, CS20);
      BIT_SET(TCCR2B, CS21);
      BIT_SET(TCCR2B, CS22);
      break;
  }
  OCR2A = ORC_value;  //Load correct value in the Timer2 Compare Register
}

//Function to modify the DIR pin
//Input: Direction required
void direction_selection(bool direction)
{
  switch (direction){
    case RIGHT:
      digitalWrite(DIR, DIR_Inv);
      break;
    case LEFT:
      digitalWrite(DIR, !DIR_Inv);
  }
}

//Function to modify the EN pin
//Input: Enable status required
void stepper_enable(bool enable)
{
  switch (enable){
    case ENABLE:
      digitalWrite(EN, EN_Inv);
      break;
    case DISABLE:
      digitalWrite(EN, !EN_Inv);
  }
}

//Function to check the status of the direction switch 
//Output: 0 - Unknow value
//        1 - Default value, centered
//        2 - Switch move to right position
//        3 - Switch move to left position
int check_switch_status(void)
{
  if (digitalRead(SW_left) && !digitalRead(SW_right)){
    return 2;
  }else if (!digitalRead(SW_left) && digitalRead(SW_right)){
    return 3;
  }else if (digitalRead(SW_left) && digitalRead(SW_right)){
    return 1;
  }else{
    return 0;
  }
}

//Function to print the mm/min current value in the main screen
//Input: Current feedrate
void print_main_page(int feed_rate)
{
  if(feed_rate <= 10){        //Clear two positions of the screen if value lower than 10
    lcd_clear(0,14,15);
  }else if(feed_rate <= 100){ //Clear one position of the screen if value lower than 100
    lcd_clear(0,15,15);
  }
  lcd.setCursor(0,0);
  lcd.print("Feedr: ");
  lcd.print(feed_rate);
  lcd.print("mm/min");
}

//Function to read the encoder 
//Input:  Feedrate - current feedrate value
//        Mode - Mode for x1 or x10 multiplier for encoder increments/decrements
int read_encoder_feedrate(int feed_rate)
{
  static bool mode;
  int encoder_increment;      
  encPos += encoder.getValue(); //Get value for the encoder
  if (encPos != oldEncPos) {    //If the value of the encoder change
    encoder_increment = encPos - oldEncPos; //Calculate the increment of the 
    if (mode == 1){             //If we are in mode x10 multiply the increment by 10
      encoder_increment = encoder_increment*10;
    }
    oldEncPos = encPos;         //Update value of old position
    feed_rate = feed_rate + encoder_increment;  //Increment the feedrate according to the encoder increment
    feed_rate = constrain(feed_rate,5,500);     //Limit the value between 
  }
  if (encoder.getButton() == 5){ //Check if we have Push the Encoder Switch to change the mode x1 or x10
      mode = !mode;   //Change the mode
      //switch_status = 0;            //Reset status variable       REMOVE
  }
  return feed_rate;   //Return the feedrate updated
}

//Function to clear specific lines in the LCD
//Input:  Row - Row selection
//        Start_pos - Start position where we want to start clearing
//        End_pos - End position where we want to finish clearing
void lcd_clear(int row, int start_pos, int end_pos)
{
  for(int i = start_pos; i<=end_pos ; i++){
  lcd.setCursor(i,row);
  lcd.print(" ");
  }
}

//Function to read the status of the encoder switch
//Output: Status of the encoder switch
int read_encoderswitch(void)
{
  int buttonState;
  buttonState = encoder.getButton();
  switch (buttonState) {                  //Switch not in use for the moment
    case ClickEncoder::Open:          //0
      break;
    case ClickEncoder::Closed:        //1
      break;

    case ClickEncoder::Pressed:       //2
      break;

    case ClickEncoder::Held:          //3
      break;

    case ClickEncoder::Released:      //4
      break;

    case ClickEncoder::Clicked:       //5
      break;

    case ClickEncoder::DoubleClicked: //6
      break;
    }
  return buttonState;
}

//Function to draw the "live" arrow 
//Input:  Direction - Select the desired direction 
void draw_arrow(int direction)
{
  static unsigned long current_millis;    //Static variable for current millis
  static unsigned long old_millis = 0;    //Static variable for previous millis
  static int arrow_sequence = 0;          //Static variable for sequence status
  static bool previous_direction;         //Static variable to know previous status
  int column = 0;
  int time_delay = 400;                   //Time in miliseconds between changes in the arrow

  if (previous_direction != direction){ //Check if there is a change in the direction between calls
  previous_direction = direction;
  arrow_sequence = 0;                   //Reset arrow sequence when direction change
  }

  current_millis = millis();  //Update the value from millis

  if (current_millis >= old_millis+time_delay){  //If reached the desired timing target
    old_millis = current_millis;    //Update the old millis with the current value
    switch(direction){
    case RIGHT:         //If direction is right
      column = arrow_sequence + 8;  //Start the arrow in LCD position 8
      if (arrow_sequence < 4){      //Draw the lines of the arrow
        lcd.setCursor(column,1);
        lcd.print("-");
        arrow_sequence++;           //Increment the sequence
      }else if (arrow_sequence == 4){ //Draw the pointer of the arrow
        lcd.setCursor(column,1);
        lcd.print(">");
        arrow_sequence++;           //Increment the sequence
      }else{                        //After draw all the arrow, clean the display
        lcd_clear(1,8,16);    
        arrow_sequence = 0;
      }
      break; 
      case LEFT:        //If direction is left
      column = (arrow_sequence * -1) + 12;  //Start the arrow in LCD position 12 (Minus to change the direction)
      if (arrow_sequence < 4){      //Draw the lines of the arrow
        lcd.setCursor(column,1);
        lcd.print("-");
        arrow_sequence++;           //Increment the sequence
      }else if (arrow_sequence == 4){ //Draw the pointer of the arrow
        lcd.setCursor(column,1);
        lcd.print("<");
        arrow_sequence++;           //Increment the sequence
      }else{
        lcd_clear(1,8,16);          //After draw all the arrow, clean the display
        arrow_sequence = 0;
      }
    }   
  }

}

//Function to aceleration profiles
//Input:  Current_Feedrate - Current Feedrate of Motor
//        Target_feedback - Target for feedrate 
//Output: Calculated feedrate according to the aceleration
float aceleration_feedrate(float current_feedrate, int target_feedrate){

  static unsigned long current_time;    //Static variable for current millis
  static unsigned long old_time = 0;    //Static variable for previous millis
  static float time = 0.05;   //Static variable for previous millis
  static bool reset = 0;      //Boolean to reset the time
  float aceleration = 0;      //Variable to storage the aceleration

  current_time = millis();

  if ((current_time > old_time + 20)  && !reset){     //Change speed every 20ms
    old_time = current_time;    
    reset = 1;
    if( (int)current_feedrate < target_feedrate ){    //Aceleration Positive Target > Current
      if (target_feedrate-current_feedrate <= 50){    //Reduce aceleration when the difference between is below 50
        aceleration  = value.Motor_aceleration * ((target_feedrate - current_feedrate) * 0.01); //Reduce the aceleration 
      }else{       //Difference between target and current bigger
        aceleration = value.Motor_aceleration;      
      }
      current_feedrate = (float) (current_feedrate + (aceleration) * time); //Formula to increate the aceleration V = Vo + a * t
      reset = 0;
      time = time + 0.02;     //Increase the time
    }else if ( (int)current_feedrate > target_feedrate ){ //Aceleration Negative Current > Target
      if (current_feedrate - target_feedrate <= 50){   //Reduce aceleration when the difference between is below 50
        aceleration  = value.Motor_aceleration * ((current_feedrate - target_feedrate) * 0.01); //Reduce the aceleration 
      }else{      //Difference between target and current bigger
        aceleration = value.Motor_aceleration; 
      }
      current_feedrate = (float) (current_feedrate - (aceleration) * time); //Formula to increate the aceleration V = Vo + a * t
      reset = 0;
      time = time + 0.02;     //Increase the time
    }
    }else if((current_time > old_time + 20)  && reset) {  //Reached the target, reset variables
      time = 0;
      old_time = current_time;
      reset = 0;
  }
  return current_feedrate;
}


//Function to read the encoder 
//Input:  Feedrate - current feedrate value
int read_encoder(void)
{
  int encoder_movement = 0;      
  encPos += encoder.getValue(); //Get value for the encoder
  if (encPos != oldEncPos) {    //If the value of the encoder change
    encoder_movement = encPos - oldEncPos; //Calculate the increment of the 
    oldEncPos = encPos;         //Update value of old position
  }
  return encoder_movement;   //Return the feedrate updated
}


//Function to store data on EEPROM  
//Input:  Structure with the current data
void eeprom_save(struct settings value){
  union save_union   //Union to copy the data from Structure and separate values on bytes
  {
    struct settings temp;
    uint8_t bytes[sizeof(settings)];
  };

  save_union save_union;  

  save_union.temp = value; //Copy values from value struct to uniun

  for (uint8_t i=0; i<sizeof(settings); i++)  //Transfer values to EEPROM
  {
    EEPROM.update(i,save_union.bytes[i]); //Update values, if the values didnt change, this function wont write the EEPROM
  }
}

//Function to recover data from EEPROM 
//Input: Pointer to the structure to modify 
void eeprom_read(struct settings *ptr_struct){
  union save_union   //Union to copy the data from Structure and separate values on bytes
  {
    struct settings temp;
    uint8_t bytes[sizeof(settings)];
  };
  save_union save_union;
  uint8_t data;             //Temporal variable to copy each data from EEPROM to Union
  for (uint8_t i=0; i<sizeof(settings); i++)  //Transfers values to Union
  {
    EEPROM.get(i, data);          //Get each byte from EEPROM address
    save_union.bytes[i] = data;   //Save data on union
  }

  *ptr_struct = save_union.temp;  //Copy values from union to destiny structure
}
/**-----------------------------------------------------------------------------------**/

