
/********* INPUT PROTOCOL ***********
* CREATE <ledPort> <ldrPort> <G> <m> <b> <tau_a_up> <tau_b_up> <tau_c_up> <tau_a_down> <tau_b_down> <tau_c_down> <dead_time>  //Called by controller::init() 
* READ //Reads ldr value [outputs in LUX]
* SET <pwm_val> //Sets led to pwm value [0, 255]
* OCCUPIED <ON/OFF> //Sets desk as occupied(20lux/215pwm) or unoccupied(5lux)
* FEEDFORWARD <ON/OFF> //Enable/Disable feedforward
* FEEDBACK <ON/OFF> //Enable/Disable feedback
* COMPUTE G //Compute linear gain and automatically saves it to controller properties
* SIMULATE <final_lux> <initial_lux> <initial_time> <time_we_want_to_compute_the_lux> 
* 
* ******************************
* 1) call CREATE to init controller
* 2) call COMPUTE G to compute gain for each test
* 3) do whatever we want
* 
* CREATE 3 A0 0.093 -0.77 50000 29047.1 -0.224531 9560.57 15904.85 -0.16087 7552.6 369.1
*/

#include "utils.h"
#include "local_controller.h"
#include "pid.h"

const byte mask= B11111000; // mask bits that are not prescale
const int prescale = 1; //fastest possible (1 for fastest)

volatile int counter = 0;
volatile float last_5values[5] = {0}; //In LUX, For avereging and checking if achieved the constant pretended value 
volatile float store_values[80] = {0};
volatile long val_time[50] = {0};
volatile bool interruptOn = false;
volatile bool flag = false;

bool menu = true;
String inBytes;

char *strings[13];
char *ptr = NULL;

bool isOccupied = false;


//Interrupt for analog reading 
ISR(TIMER1_COMPA_vect){
  if(interruptOn) {
    flag = true;    
  }
}


float readLdr(Local_controller _controller) {
    int sensorValue = analogRead(_controller.get_ldr_port());
    return(voltageToLux(analogToVoltage(sensorValue), _controller.getM(), _controller.getB()));
}

float achieved_constant_lux(float _last_5values[5]) {
  unsigned int sum = 0;
  for(int i=0; i<5; i++) {
    sum += _last_5values[i];
    //Serial.println(_last_5values[i]);
  }
  //Serial.println();
  return (sum / 5.0); //Returns true if achieved the pretended lux
}

/* For achieving a certain lux we can't just analogWrite(ledPort
 * we have to start writing to led every 100Hz(0.01s = 10000us) 
 * with value given by the error(ldrRead-simulatedLux)
*/
float start_impulse(Pid _pid, Local_controller _controller, float _pretended_lux) {
  long startingTime = micros(); //We have the <t_i> for simulator
  long currentTime = 0;
  float starting_volt = analogToVoltage( analogRead(_controller.get_led_port()) );
  int u_ff = (int)( (_pretended_lux - _controller.getOffset()) / _controller.getG());  //Conversion to int because the led can only receive a PWM
  int u = u_ff;
  int u_prev = u_ff; //For making an average
  float y = 0.0;
  float y_ref = 0.0;
  counter = 0;
  
  for(int i=0; i<5; i++) {  //Reset the average's vector
    last_5values[i] = 0;
  }

  sei();
  interruptOn = true;
  while(true) {
    if(Serial.available() > 0) { //If we want a differente step, we need to restart the variables
      interruptOn = false;
      _pretended_lux = Serial.readString().toFloat();
      u_ff = (int)( (_pretended_lux - _controller.getOffset()) / _controller.getG());
      startingTime = micros();
      currentTime = 0;
      starting_volt = analogToVoltage( analogRead(_controller.get_led_port()) );
      u = u_ff;
      u_prev = u_ff;
      y = 0.0;
      y_ref = 0.0;
      counter = 0;
      interruptOn = true;
    }
    if(flag) {
      counter = (counter==5) ? 0 : counter; //In case the last values vector overflows, it starts sotring from the beginning, overwriting old values(what's really needed is the last 10)
      last_5values[counter] = analogRead(_controller.get_ldr_port());
      currentTime = micros();
      y = analogToVoltage( achieved_constant_lux(last_5values) );
      y_ref =_controller.simulate(_pretended_lux, starting_volt, startingTime, currentTime  ); //y_ref in Volts
      u = bounds( u_ff + (int)( _pid.compute_pid( y, y_ref, ((float)u_ff)/51.0 ) * 51 ));
      Serial.print(voltageToLux(y, _controller.getM(), _controller.getB()));
      Serial.print(" ");     
      Serial.println(_pretended_lux);
      analogWrite(_controller.get_led_port(), (int) ((u + u_prev)/2.0));
      u_prev = u;
      counter++;
      flag = false;
    }
  }
  cli();
  interruptOn = false;
  TCNT1 = 0; //reset counter
  
  //Return the last value of the storing values array
  return achieved_constant_lux(last_5values);
}

bool checkIfNumber(String str) {
  for(byte i=0;i<str.length();i++) {
    if((isDigit(str.charAt(i)) == 0) and ( str.charAt(i) != '.' ) and ( str.charAt(i) != '-' )) //If in any of the string digits there's a character, that it's not a dot or minus sign (-0.23 i.e) then it's not a number
      return false;
  }
  return true;
}

void setup() {
  Serial.begin(2000000);
  
  TCCR2B = (TCCR2B & mask) | prescale;  //Raise Timer2 (pwm of port 3) for ldr not sensing led flicker

  //Fo preparing the Sampling Period Timer Interrupt
  //100Hz samples == 0.01s == 10000us sample time
  cli(); //disable interrupts
  TCCR1A = 0; // clear register
  TCCR1B = 0; // clear register
  TCNT1 = 0; //reset counter
  // OCR1A = desired_period/clock_period – 1 //This -1 is needed for Arduino specifics
  // = clock_freq/desired_freq - 1
  // = (16*10^6 / 6400) - 1 / (100*1Hz) – 1  //Because prescale is 64, each second or 1Hz is 64, so 100 is 100*64
  // = 2499
  OCR1A = 2499; //must be <65536
  TCCR1B |= (1 << WGM12); //CTC On
  // Set prescaler for 64 -
  TCCR1B |= (1 << CS11) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei(); //enable interrupts 
}

void loop() {  
  Local_controller controller1;
  Pid pid(1.0, 0.8, 0.2, 0.4); //PID with default values of T=1, p=1, i=0, d=0

  while(menu == true) {
    if(Serial.available() > 0) {
      byte index = 0;
      inBytes = Serial.readString();

      // Split string
      char *cstr = inBytes.c_str();
      ptr = strtok(cstr, " ");  // takes a list of delimiters, our case only ' '
      while(ptr != NULL)
      {
          strings[index] = ptr;
          index++;
          ptr = strtok(NULL, " ");
      }

      //MENU INPUT STRING TREATMENT
      if(strcmp(strings[0], "SET") == 0) {
        if(checkIfNumber(strings[1]))
          start_impulse( pid, controller1, (atoi(strings[1])*controller1.getG() + controller1.getOffset()) );
      } else if(strcmp(strings[0], "CREATE") == 0) {
        //Create a new local controller (new ldr)
        if( checkIfNumber(strings[1]) and checkIfNumber(strings[3]) and checkIfNumber(strings[4]) and checkIfNumber(strings[5]) and checkIfNumber(strings[6]) and checkIfNumber(strings[7]) and checkIfNumber(strings[8]) and checkIfNumber(strings[9]) and checkIfNumber(strings[10]) and checkIfNumber(strings[11]) and checkIfNumber(strings[12]) )
          controller1.init(atoi(strings[1]), atoi(strings[2]), atof(strings[3]), atof(strings[4]), atof(strings[5]), atof(strings[6]), atof(strings[7]),atof(strings[8]), atof(strings[9]), atof(strings[10]), atof(strings[11]), atof(strings[12]));
      } else if(strcmp(strings[0], "READ") == 0) {
        //Read the value obtained by the ldr [in LUX]
        Serial.print("LUX VALUE READ ON LDR: ");
        Serial.println(readLdr(controller1));
      } else if(strcmp(strings[0], "OCCUPIED") == 0) {
        if(strcmp(strings[1], "ON") == 0) {
          //Set desk as occupied ( 20 lux )
          start_impulse(pid, controller1, 20.0); //20 LUX is mynoccupied
          isOccupied = true;
        } else if(strcmp(strings[1], "OFF") == 0) {
          //Set desk as unoccupied ( 8 lux )
          start_impulse(pid, controller1, 8.0); //5 LUX is my unoccupied
          isOccupied = false;
        }
      } else if(strcmp(strings[0], "FEEDFORWARD") == 0) {
        //Enable/disable feedforward term
        //TODO:
      } else if(strcmp(strings[0], "FEEDBACK") == 0) {
        //Enable/disable feedback term
        //TODO:
      } else if((strcmp(strings[0], "COMPUTE") == 0) and (strcmp(strings[1], "G") == 0)) {
        //Computes the linear gain G
        controller1.setG(controller1.compute_linear_gain());
      } else if(strcmp(strings[0], "SIMULATE") == 0) {
        Serial.println(controller1.simulate(atof(strings[1]), atof(strings[2]), atol(strings[3]), atol(strings[4])));
      } else {
        //Serial.println("** PLEASE READ INPUT INSTRUCTIONS **");
      }
    }
  }

  while(true) {
    
  }
  
  //controller1.init(3, A0, 0.05353, -0.77, 50000, 29047.1, -0.224531, 9560.57, 15904.85, -0.16087, 7552.6, 369.1); //The 0.05353 G will be changed

  //Serial.println(controller1.simulate_Vt(4.0, 3.0, 300000, 320000, 204*(controller1.getG()))); //204*0.093 = 18.972
}
