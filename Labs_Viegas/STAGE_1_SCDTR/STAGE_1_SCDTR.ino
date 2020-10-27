#include "utils.h"
#include "local_controller.h"

bool menu = true;
String inBytes;

char *strings[13];
char *ptr = NULL;

void print_starting_menu() {
  Serial.print("************************************************************************************************\n");
  Serial.print("*                                   SERIAL INPUT OPTIONS                                       *\n");
  Serial.print("* \"CREATE <led> <ldr> <G> <m> <b> <a_up> <b_up> <c_up> <a_down> <b_down> <c_down> <dead_time>\" *\n");
  Serial.print("* \"READ\"  - To read LDR values in LUX                                                          *\n");
  Serial.print("* \"SET <pwm value [0, 255]>\" - Light led with PWM                                              *\n");
  Serial.print("* \"OCCUPIED\" - Set desk as occupied                                                            *\n");
  Serial.print("* \"UNOCCUPIED\" - Set desk as unoccupied                                                        *\n");
  Serial.print("* \"FEEDFORWARD <ON/OFF>\" - Switch on/off feedforward                                           *\n");
  Serial.print("* \"FEEDBACK <ON/OFF>\" - Switch on/off feedback                                                 *\n");
  Serial.print("*                                         DEBUGGING                                            *\n");
  Serial.print("* \"COMPUTE G\" - Compute linear gain G (automathically saves G to controller)                   *\n");
  Serial.print("* \"SIMULATE <vf> <vi> <ti> <t> <lux>\" - simulate v(t)                                          *\n");
  Serial.print("************************************************************************************************\n\n");
}

float readLdr(Local_controller _controller) {
    int sensorValue = analogRead(_controller.get_ldr_port());
    return(voltageToLux(analogToVoltage(sensorValue), _controller.getM(), _controller.getB()));
}

void setup() {
  Serial.begin(19200);
  print_starting_menu();
  
}

void loop() {
  Local_controller controller1;

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
         //Setting the led to a pwm value
        analogWrite(controller1.get_led_port(), atoi(strings[1]));
        Serial.print("LED SET TO ");
        Serial.print(strings[1]);
        Serial.println(" PWM");
      } else if(strcmp(strings[0], "CREATE") == 0) {
        //Create a new local controller (new ldr)
        controller1.init(atoi(strings[1]), atoi(strings[2]), atof(strings[3]), atof(strings[4]), atof(strings[5]), atof(strings[6]), atof(strings[7]),atof(strings[8]), atof(strings[9]), atof(strings[10]), atof(strings[11]), atof(strings[12]));
      } else if(strcmp(strings[0], "READ") == 0) {
        //Read the value obtained by the ldr [in LUX]
        Serial.print("LUX VALUE READ ON LDR: ");
        Serial.println(readLdr(controller1));
      } else if(strcmp(strings[0], "OCCUPIED") == 0) {
        //Set desk as occupied, which means we want the lux to be on certain high value, maybe 20LUX? My max lux is around 24LUX
        //TODO:
      } else if(strcmp(strings[0], "UNOCCUPIED") == 0) {
        //Set desk as unoccupied, which means we want the lux to be on certain low value, maybe 5LUX? My max lux is around 24LUX
        //TODO:
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
        //Simulate a v(t) based on some inputs vf, vi, ti, t, lux pretended to obtain
        Serial.println(controller1.simulate_Vt(atof(strings[1]), atof(strings[2]), atol(strings[3]), atol(strings[4]), atof(strings[5])));
      } else {
        Serial.println("** PLEASE READ INPUT INSTRUCTIONS **");
      }

      //free(strings);
      //free(ptr);
      //free(cstr);
      controller1.print_params();
    }
  }

  

  //controller1.init(3, A0, 0.05353, -0.77, 50000, 29047.1, -0.224531, 9560.57, 15904.85, -0.16087, 7552.6, 369.1); //The 0.05353 G will be changed


  //Serial.println(controller1.simulate_Vt(4.0, 3.0, 300000, 320000, 204*(controller1.getG()))); //204*0.093 = 18.972
  
  delay(10);
}
