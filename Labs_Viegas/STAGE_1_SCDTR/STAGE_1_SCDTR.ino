/* We define:
 *  Occupied == 20 lux 
 *  Unoccupied == 5 lux
*/

#include "utils.h"
#include "local_controller.h"

bool menu = true;
String inBytes;

char *strings[13];
char *ptr = NULL;

bool occupied = true;

float readLdr(Local_controller _controller) {
    int sensorValue = analogRead(_controller.get_ldr_port());
    return(voltageToLux(analogToVoltage(sensorValue), _controller.getM(), _controller.getB()));
}

void setup() {
  Serial.begin(9600);
  Serial.println("Enter Input: ");  
}

void loop() {
  Local_controller controller1;

  while(menu == true) {
    if(Serial.available() > 0) {
      byte index = 0;
      inBytes = Serial.readString();
      Serial.println(inBytes);

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
         //TODO: this analog write i think is like the u_ff (u of feedforward)
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
        occupied = true;
        analogWrite(controller1.get_led_port(), atoi(strings[1]));
      } else if(strcmp(strings[0], "UNOCCUPIED") == 0) {
        //Set desk as unoccupied, which means we want the lux to be on certain low value, maybe 5LUX? My max lux is around 24LUX
        //TODO:
        occupied = false;
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
        Serial.println("** PLEASE READ INPUT INSTRUCTIONS **");
      }
    }
  }

  

  //controller1.init(3, A0, 0.05353, -0.77, 50000, 29047.1, -0.224531, 9560.57, 15904.85, -0.16087, 7552.6, 369.1); //The 0.05353 G will be changed


  //Serial.println(controller1.simulate_Vt(4.0, 3.0, 300000, 320000, 204*(controller1.getG()))); //204*0.093 = 18.972
  
  delay(10);
}
