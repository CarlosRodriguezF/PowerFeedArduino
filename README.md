# PowerFeed
PowerFeed for Workshop Machines based on Arduino

**REVISION**

V1.0 
- First version with the basic functionallity LEFT and RIGHT movement, mm/min selection and initial lock in case the switch is in LEFT or RIGHT to avoid un-desired movements

V1.1
- Optimized encoder Switch read, removed function from main and included in function to change the feedrate
- Added Aceleration profiles 
- Added High Speed mode, keep the encoder pressed and the speed will go to high speed

V1.2
- Added possibility to configure the CONFIGURATION with the LCD screen (Just press encoder switch meanwhile we are powering up the module)

**DESCRIPTION**

Project based on Arduino to be able to control PAP motor driver (EN,DIR,PULSE) and set the speed in mm/min of the axis of your machines, like lathe, milling machine...

This project is being developed in VisualStudioCode with PlatformIO libraries. You can find the CPP file and migrate it to Arduino IDE if do you prefer

Firware.hex and .helf you can use these files to upload the code directly to your target with a USBasp programer for example. 

Basically this project consist in an Arduino Nano 328P, Rotary Encoder, 3Pos (Latched) Switch and I2C LCD2*16. You can control easily PAP motor driver like for example DM545 and select which direction do you want to feed and with which speed in mm/min do you want to go. It is possible as well to modify the speed when the motor is in movement. 


**CONFIGURATION**

You can adapt the control to your Driver/Motor changing this parameters in the main code. For the moment I have not checked all of the combinations, please if you find a BUG let me know. 

const int ENCODER_STEPS_PER_NOTCH = 4;  // Change this depending on which encoder is used

long Arduino_clk = 16000000;  //Oscillator frequency of arduino board (Nano 16MHz)

float Motor_stepangle = 1.8;  //This value is defined in the specs of the motor
int Driver_stepsrev = 6400;   //This value define the microstteping which you have selected in your driver
int Machine_lead  = 2;        //mm/rev this is the mm which your axis moves each turn of your leadscrew (In my case 2mm each rev)

bool DIR_Inv  = 1;    //Use this value to Invert or not the DIRection Pin function. (In case the direction of the power feed is inverted)
bool EN_Inv  = 0;     //Use this value to Invert or not the ENable Pin function.   

You can change the pin out assignation in the main as well
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

**PICTURES**

![alt tag](https://github.com/CarlosRodriguezF/PowerFeed/blob/main/BlockDiagramU.JPG?raw=true)
![alt tag](https://github.com/CarlosRodriguezF/PowerFeed/blob/main/P1070544.JPG?raw=true)
![alt tag](https://github.com/CarlosRodriguezF/PowerFeed/blob/main/P1070557.JPG?raw=true)


NOTES:
- Dont forget to use debouncing circuitry on the ENCODER in case it doesnt have it, is is recomended
- Dont supply the 5V for the LCD directly from the ARDUINO use external power source
- Block Diagram just as reference
- You can see more pictures in the git
