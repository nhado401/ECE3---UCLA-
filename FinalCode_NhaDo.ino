/*ECE 3 - UCLA - Fall 2020 - Dr Briggs - Code Written by Nha Do.
Line Following Program for MSP 432, TI-RSLK robotic car (TI Launchpad).

#include <ECE3.h>

uint16_t sensorValues[8]; // right -> left, 0 -> 7

const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int left_dir_pin=29;
const int left_pwm_pin=40;

const int right_nslp_pin=11; // nslp ==> awake & ready for PWM
const int right_dir_pin=30;
const int right_pwm_pin=39;

const int LED_RF = 41;
const int evenLed = 45;
const int oddLed = 61;

int preError = 0;
int lineCount = 0;

int leftSpeed = 80; 
int rightSpeed = 80;

///////////////////////////////////
void setup() {
//Set up code to run once:
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);
  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  
  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);
  
  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,HIGH);
  

  pinMode(LED_RF, OUTPUT);
  pinMode(evenLed,OUTPUT);
  pinMode(oddLed,OUTPUT);

  digitalWrite(oddLed,HIGH);
  digitalWrite(evenLed,HIGH);
  
  ECE3_Init();
  //delay(1000); // delay 1 second before running
 }

void loop() {
  // Code inside this loop will run repeatedly
    
  ECE3_read_IR(sensorValues);

  //Calculate the values of each sensor based on Sensor Fusion
  int s1=(sensorValues[0]-597)*0.525486075;
  int s2=(sensorValues[1]-620)*0.570450656;
  int s3=(sensorValues[2]-666)*0.545256270;
  int s4=(sensorValues[3]-620)*0.754261578;
  int s5=(sensorValues[4]-667)*0.703729768;
  int s6=(sensorValues[5]-681.4)*0.5670220;
  int s7=(sensorValues[6]-738)*0.567536890;
  int s8=(sensorValues[7]-736)*0.566893424;
  
  int sensorCount = 0; // used to count the number of sensors are being covered. (see the black line)

  //If a sensor sees the black line, the value increases by 1
  for(int i = 0; i < 8; i++){
    if(sensorValues[i] > 1000)
      sensorCount++;
  };

  //Calculate error function used to control motor's speed.
  int error = s1*(-8) + s2*(-4) + s3*(-2) + s4*(-1) + s5*1 + s6*2 + s7*4 + s8*8;

  float kP = 50.0/2200;
  float kD = 0.0002*0.6;
  
  float currentLeftSpeed = leftSpeed - kP*error + kD*(preError - error);
  float currentRightSpeed = rightSpeed + kP*error - kD*(preError - error);
  
  analogWrite(left_pwm_pin,currentLeftSpeed);
  analogWrite(right_pwm_pin,currentRightSpeed);

  //Turning the car at the end of the track and stopping at the beginning
  if(sensorCount >= 6){   // At least 6 sensors see black line (the horizontal line at the end and the beginning of the track)     
      
    //Seeing the line at the end of the track and turning around
    if(lineCount == 0){    
      digitalWrite(left_dir_pin, HIGH);
      analogWrite(right_pwm_pin, 200);
      analogWrite(left_pwm_pin, 200);
      delay(300);
      digitalWrite(left_dir_pin, LOW);
      lineCount = 1; // Saw the first horizontal line
      }
    
    //Seeing the line at the beginning of the track and stopping
    else if (lineCount == 1){
      analogWrite(right_pwm_pin, 0);
      analogWrite(left_pwm_pin, 0);
      currentLeftSpeed = 0;
      currentRightSpeed = 0;
      delay(5000);
      }
    }     

  preError = error;
}
