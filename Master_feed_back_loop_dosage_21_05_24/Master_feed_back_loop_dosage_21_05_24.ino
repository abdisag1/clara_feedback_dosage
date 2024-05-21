#include <TimerOne.h>

/*
The user should in put the following parameters to calibrate the system as per required.
target_frc =1.5//target FRC value needed in mg/l
target_wata_concentration = 4.5;  // target concentration of NaClO needed
flowratio // variable that stores the ration between the normal pipe and the bypass pipe

*/

const int pwmPin = 11;  // PWM output pin
const int rpmPin = 3;
const int pin2 = 2;
const int pwmFrequency = 10000;  // 10 kHz PWM frequency
int pwm = 32;
unsigned long previousMillis1 = 0;  // Variable to store the last time function1 was called
unsigned long previousMillis2 = 0;  // Variable to store the last time function1 was called

// variables to hold the calculated flow_freq and flowrate
float callibration = 4.8;
float flow_freq = 0;
float flowrate = 0;
volatile long average_pulse = 0;

// variables to hold the pulse interupts for the flow
unsigned long pulse_flow = 0;
unsigned long currTime_flow = 0;
unsigned long currTime_flow2 = 0;
unsigned long prevTime_flow = 0;
unsigned int pulse_count_flow = 0;
volatile long commulative_pulse_flow = 0;

// interval for the first timer
const long interval1 = 3000;
// interval for the second timer
const long interval2 = 60000;

// variables for the controller
float set_point = 5;
int kp = 1.5;
float ki = 1;
float error_accumulator = 0;
int flowratio=25;
float k_ml = 1.2;

// flow rate variables
float Q = 100; // variable to stores the multiplied flowrate with the flow ratio
//float dosage = 0.33; // dosage in ml/l
float r_dose;
int correction = 1;  ///variable for correction
float target_frc =1.5; //target FRC value needed in mg/l
float target_wata_concentration = 4.5;  // target concentration of NaClO needed
float dosage = (target_frc)/target_wata_concentration; // dosage in ml/l
float intercept = 0.116;
float slope = 1.013;


/// isr function variables

unsigned long pulse1 = 0;
unsigned long currTime1 = 0;
unsigned long currTime2 = 0;
unsigned long prevTime1 = 0;
unsigned int pulse_count_1 = 0;
volatile long commulative_pulse_1 = 0;
// Declare variables to store the frequency and flow rate values
volatile int freq1 = 0;
volatile int freq2 = 0;

// variables to hold pulse interrupt for the flowmeter


// function to be called when there is trigger on the feedback pin
void isr1() {
  // Get the current time
  currTime1 = micros();
  // Calculate the pulse duration
  pulse1 = currTime1 - prevTime1;
  // read pulse
  // Serial.print("pulse1: ");
  // Serial.print(pulse1);
  // Update the previous time
  prevTime1 = currTime1;
  // Calculate the frequency
  freq1 = 1000000 / pulse1;
  // increments every time there is an interupt
  pulse_count_1++;
  // whenever there is an interupt the pulse will be commulated.
  commulative_pulse_1 += pulse1;
    // Serial.print("\tpulse :  ");
    // Serial.println(pulse1);
  // Serial.print("\tcommulative:  ");
  // Serial.println(commulative_pulse_1);

  TCNT1 = 0;  // Reset the timer1 counter
}

//interrupt subroutine call when there is rising edge on the flowmeter's pin.
void isr2() {
  // Get the current time
  currTime_flow = micros();
  // Calculate the pulse duration
  pulse_flow = currTime_flow - prevTime_flow;
  // read pulse
  // Serial.print("pulse_flow: ");
  // Serial.println(pulse_flow);
  // Update the previous time
  prevTime_flow = currTime_flow;
  // Calculate the flow_frequency
  // flow_freq1 = 1000000 / pulse_flow;
  // increments every time there is an interupt
  pulse_count_flow++;
  // whenever there is an interupt the pulse will be commulated.
  commulative_pulse_flow += pulse_flow;
  //  Serial.print("\tflow_frequency:  ");
  // Serial.println(flow_freq1);
  // Serial.print("\tcommulative:  ");
  // Serial.println(commulative_pulse_flow);

  TCNT1 = 0;  // Reset the timer1 counter
}



// sets the target point by reciving flowrate and returns the 
float set_point_f(float q) {
  set_point = q * dosage * correction;
  // Serial.print("set point: ");
  // Serial.println(set_point);

  return set_point;
}

void proportional_controller(int pulse, float set_point_l) {
  float pulse_rpm = pulse *k_ml* 2.5; // 8 pulse = 12 section in a min (5sec)-- 12/8 ...
  float error = set_point_l - pulse_rpm;
  

     Serial.print("\tpulse count:  ");
   Serial.println(pulse_count_1);
  Serial.print("Set point inside pc:  ");
  Serial.println(set_point_l);

  float ut = error * kp + error_accumulator * ki;

error_accumulator = error_accumulator+ error;
  Serial.print("Commulated Error:  ");
  Serial.println(error_accumulator);

  // ut saturation
  if (ut < 32) {
    ut = 20;
  }
  if (ut >= 255) {
    ut = 255;
  }
    int pwm_value = map(ut,0,255,0,1023);
    Serial.print("PWM: ");
    Serial.println(pwm_value);
  Timer1.pwm(11, pwm_value);

  Serial.print("ut : ");
    Serial.println(ut);

    Serial.print("error: ");
    Serial.println(error);
}


void setup() {
  pinMode(11, OUTPUT);  // Set pwmPin as an output
  pinMode(pin2, INPUT);
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(rpmPin), isr1, RISING);
  attachInterrupt(digitalPinToInterrupt(pin2), isr2, RISING);
  sei();
  Timer1.initialize(100);  // Set the timer period in microseconds for desired frequency
  Timer1.pwm(11, 128);     // Initialize PWM on pwmPin with 50% duty cycle (adjust as needed)
  delay(1000);
}

// calculates the flow rate by taking commulative_pulse_flow and pulse_count_flow as a parameter
void read_flow (){
  // display flow rate
    // Serial.print("pulse_flow");
    // Serial.println(pulse_flow);
      // Serial.print("commulative pulse: ");
      // Serial.println(commulative_pulse_flow);
    average_pulse = commulative_pulse_flow / pulse_count_flow;
    //  Serial.print("Average pulse");
    //  Serial.println(average_pulse);
    flow_freq = 1000000 / average_pulse;
    flowrate = flow_freq / callibration;
    Q = (flowrate + intercept);
    Q = Q*slope*flowratio;

    Serial.print("Flow Rate: ");
    Serial.print(Q);
    Serial.println(" L/min");
    pulse_count_flow = 0;
    commulative_pulse_flow = 0;
    // end variables to read flow

}



void loop() {

  // int pwm_value = map(pwm,0,255,0,1023);
  // Timer1.pwm(11, pwm_value);
  //analogWrite(9,126);


  unsigned long currentMillis = millis();  // Get the current time

  // Check if it's time to call function1 (every second)
  if (currentMillis - previousMillis1 >= interval1) {
    // Save the last time function1 was called
    previousMillis1 = currentMillis;
    // Serial.print("set point function: ");
    // Serial.println(set_point_f(Q));
    

    read_flow ();
    set_point_f(Q);
    proportional_controller(pulse_count_1, set_point_f(Q));
    pulse_count_1 = 0;

    // Call function1
    //Serial.println("second tick") ;
  }
  // Nothing to do here as PWM is generated automatically by Timer1


  // 1 min timer

  if (currentMillis - previousMillis2 >= interval2) {
    previousMillis2 = currentMillis;

    //error_accumulator = error_accumulator*0;
    // Serial.print("Error Accumulator: ");
    // Serial.println(error_accumulator);
    // Save the last time function1 was called
  }
}