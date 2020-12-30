#include "controller.h"
#include "comms.h"
#include "hub.hpp"

// INIT PID
ControllerPid pid(3, A0);
MCP2515 mcp2515(10);
/**********************************************
 * GLOBAL VARIABLES
***********************************************/ 

/*-------------------------------------------
 * ARDUINO PARAMETERS                         |
---------------------------------------------*/
float m;
float b;
byte my_address;

boolean LOOP = false;
boolean SIMULATOR = false;
boolean DEBUG = false;

long ack_time = 0;
long starting_time = 0;
/*-------------------------------------|
 * STATE MACHINE OF THE PROGRAM        |
---------------------------------------|*/
enum state_machine {booting=0, standard=1, w8ing_msg = 2, ready_for_calib=3, offset_calc = 4 ,calibration=5, w8ing_ack=6, consensus = 7, w8ing_ldr_read = 9};
state_machine my_state = booting;
state_machine prev_state = booting;

//number of nodes in ready_for_calib state
int ready_number=0;
int num_of_ollehs = 0;
/*------------------------|
 * TYPE OF MESSAGES       |
--------------------------|*/
msg_types msg_to_send;
int ack_number=0;
/*-------------------------------------------
 * VARIABLES FOR THE CALIBRATION            |
*-------------------------------------------*/
bool master = false;

//L_i = [ki1, ki2, ..., kiN]^T*[d_avg, ..., di, ..., d_avg] + o_i
float my_offset = -1;
int my_pwm = -1;
int avg_pwm = -1;
int *pwm_vect;
float *my_gains_vect;
float my_luminance = -1;

/*--------------------------------------------|
 * CAN BUS COMMUNICATION USEFUL VARIABLES     |
----------------------------------------------|*/
//array of variable size depending on number of nodes
byte *nodes_addresses;  
int number_of_addresses = 0;

can_frame new_msg;

/********************************
 * RPI COMMUNICATION
********************************/
byte *serial_buffer;
int serial_buff_size = 0;
bool im_hub = false;
bool transmiting = false;
double counter = 0;

/********************************
 * GENERIC VARIABLES
********************************/
float my_cost = 0;
float lower_bound_lux = 0;

/*-----------------------------------------------------|
 * FUNCTIONS HEADERS                                     |
-------------------------------------------------------|*/
float getValueMsg(can_frame frame);

void setup() {
  Serial.begin(57600);

  int eeprom_addr = 0;
  EEPROM.get(eeprom_addr, my_address);
  eeprom_addr += sizeof(byte);
  EEPROM.get(eeprom_addr, m);
  eeprom_addr += sizeof(float);
  EEPROM.get(eeprom_addr, b);

  if(DEBUG) {
    Serial.print(my_address, 4);
    Serial.print(" - ");
    Serial.print(m, 4);
    Serial.print(" - ");
    Serial.println(b, 4);
  }

  //CAN-BUS setup
  SPI.begin();
  attachInterrupt(0, irqHandler, FALLING);
  //Must tell SPI lib that ISR for interrupt vector zero will be using SPI
  SPI.usingInterrupt(0);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_16MHZ);
  mcp2515.setNormalMode();
  
  delay(500);
  
  pid.ldr.setGain( pid.getLedPin(), m, b );
  pid.ldr.t_tau_up.setParametersABC( 29.207246, -0.024485, 11.085226); // values computed in the python file
  pid.ldr.t_tau_down.setParametersABC( 15.402250,  -0.015674, 8.313158); // values computed in the python file
  
  delay(500);
  if(DEBUG)
    Serial.println("Set up completed");
  
  //pid.setReferenceLux( 30 ); // sets the minimum value in the led ( zero instant )
  //if(pid.has_feedback()){initInterrupt1();}
  initInterrupt1();
  nodes_addresses = (byte*)malloc(2*sizeof(byte)); //Because we will be address '1', cause '0' is for broadcast
  nodes_addresses[0] = 0;
  nodes_addresses[1] = my_address;
  number_of_addresses = 2;  
}

//*************** STATE MACHINE FUNCTIONS ************************
/*-----------------------------------------------|
  Function when the node is in booting state     |
-------------------------------------------------|*/
void booting_function(MCP2515 mcp2515 ){
  msg_to_send = hello;
  writeMsg(mcp2515, 0, msg_to_send, my_address);
  prev_state = my_state;
  my_state = w8ing_msg;
}

/*-----------------------------------------------|
  Function when tht node is waiting for ollehs   |
-------------------------------------------------|*/
void w8ing_function(MCP2515 mcp2515){
  if(micros() - starting_time >= 4000000){
    //Time has passed, I'm the only node on grid and I'm ready for calib
    prev_state = my_state;
    my_state = ready_for_calib;
  
    if(num_of_ollehs > 0) {
      number_of_addresses = 2 + num_of_ollehs;
      bubbleSort(nodes_addresses, number_of_addresses);
      //send broadcast READY_CALIB
      msg_to_send = Ready;
      writeMsg( mcp2515, 0, msg_to_send, my_address);
      //num_of_ollehs = 0;
    }
  }
}

/*-----------------------------------------------------------------------------|
  Function when the node is waiting all the nodes to be ready for calibration  |
-------------------------------------------------------------------------------|*/
void ready_function(){
  //Reset calibration variables
  my_offset = -1;
  my_pwm = -1;
  avg_pwm = -1;
  //free(pwm_vect);
  //free(my_gains_vect);
  my_gains_vect = (float*)calloc((number_of_addresses - 1), sizeof(float));
  my_luminance = -1;

  if(ready_number==number_of_addresses-2){
    if(nodes_addresses[1] == my_address) { //If i'm the 1st node on addr vector then I run offset computation once and only once
      master = true;
      prev_state = my_state;
      my_state = offset_calc;
    }
    else {
      master = false;
      prev_state = my_state;
      my_state = calibration; //All the other nodes enter calibration state
    }
    ready_number=0;
  }
}


/*-----------------------------------------------|
  Function when the node is computing offset     |
-------------------------------------------------|*/
void offset_function(MCP2515 mcp2515){
  //I'm the master and the first node so I enter first here for computing the offsets for all, only once
  pid.led.setBrightness(0); //Turn off myLed
  if(prev_state == ready_for_calib) {
    if(number_of_addresses > 2) {
      msg_to_send = turnOff_led;
      writeMsg(mcp2515, 0, msg_to_send, my_address);
    }
    prev_state = my_state;
    my_state = w8ing_ack;
    ack_time = micros();
    ack_number = 0;
  } else if( (prev_state == w8ing_ack) && (my_offset < 0) ) { 
    //Now I know all the nodes turned off their led
    my_offset = pid.ldr.luxToOutputVoltage(pid.ldr.getOutputVoltage(), true);
    if(number_of_addresses > 2) {
      msg_to_send = read_offset_value;
      writeMsg(mcp2515, 0, msg_to_send, my_address);
    }
    prev_state = my_state;
    my_state = w8ing_ack;
    ack_time = micros();
    ack_number = 0;
  } else if( (prev_state == w8ing_ack) && (my_offset >= 0) ) { 
    //Now I know every node computed their offset so I'm for entering calibration state
    prev_state = my_state;
    my_state = calibration; 
  }
}

/*------------------------------------------------------|
  Function when the node is waiting for all the acks    |
--------------------------------------------------------|*/

void w8ing_ack_function(MCP2515 mcp2515){
  if( ( ack_number == number_of_addresses-2 ) && ( prev_state == offset_calc ) ){
    ack_number=0; //reset
    prev_state = my_state;
    my_state = offset_calc;
  } else if( (prev_state == offset_calc) && ( micros() - ack_time > 3000000 ) ) {
    ack_time = micros();
    ack_number = 0;
    if(number_of_addresses > 2) {
      msg_to_send = turnOff_led;
      writeMsg( mcp2515, 0, msg_to_send, my_address);
    }
  } else if(( ack_number == number_of_addresses-2 ) && ( prev_state == calibration )) {
    ack_number=0; //reset
    prev_state = my_state;
    my_state = calibration;
  } else if( (prev_state == calibration) && ( micros() - ack_time > 3000000 ) ) {
    ack_time = micros();
    ack_number = 0;
    if(number_of_addresses > 2) {
      msg_to_send = read_lux_value;
      writeMsg( mcp2515, 0, msg_to_send, my_address);
    }
  }
}

/*------------------------------------------------------|
  Function to calibrate K                               |
--------------------------------------------------------|*/

void calib_function(MCP2515 mcp2515){
  //I'm not the first addr so when I'm the master I only do the calibration part of entire Calibration
  if(master) { 
    //If this is my round of being the master
    if( prev_state == offset_calc ) {
      pid.led.setBrightness(255); //I know that everyone's led is turned off so I can turn on mine
      prev_state = my_state;
      my_state = w8ing_ldr_read;
      starting_time = micros();
    } else if(prev_state == w8ing_ldr_read) {
      //Compute my K
      my_gains_vect[retrieve_index(nodes_addresses, number_of_addresses, my_address)-1] = ( pid.ldr.luxToOutputVoltage( pid.ldr.getOutputVoltage(), true) - my_offset ) / 255.0;
      if(number_of_addresses > 2) {
        msg_to_send = read_lux_value;
        writeMsg( mcp2515, 0, msg_to_send, my_address);
      }
      prev_state = my_state;
      my_state = w8ing_ack;
      ack_time = micros();
      ack_number = 0;
    } else if(prev_state == w8ing_ack) {
      pid.led.setBrightness(0); //Turn off led again for not disturbing next reads
      //Pass the grenade to the next node if exists (check if I'm the last one)
      if(my_address == nodes_addresses[number_of_addresses-1]) {
        prev_state = my_state;
        my_state = consensus;
      } else {
        master = false;
        if(number_of_addresses > 2) {
          msg_to_send = your_time_master;
          writeMsg( mcp2515, nodes_addresses[retrieve_index(nodes_addresses, number_of_addresses, my_address) + 1], msg_to_send, my_address); //Send msg to the next node for him to become master
        }
      }
    }        
  }
}

/*------------------------------------------------------|
  Function check end loop messages                      |
--------------------------------------------------------|*/

void check_messages(MCP2515 mcp2515){
    //Check if received msg is from a new entering node...
  if((new_msg.data[0] == hello) && (my_state != w8ing_msg) && (my_state != booting) ) { 
    bool already_on_addresses = false;
    for(int j=0; j < number_of_addresses; j++) {
      if(new_msg.data[1] == nodes_addresses[j]) {
        already_on_addresses = true;
      }
    }
    //If this address have never been to my addresse's vector
    if(already_on_addresses == false) { 
      number_of_addresses += 1;
      nodes_addresses = (byte*)realloc(nodes_addresses, number_of_addresses*sizeof(byte));
      nodes_addresses[number_of_addresses-1] = new_msg.data[1];
      bubbleSort(nodes_addresses, number_of_addresses);
    }
    msg_to_send = olleh;
    smallMsg( mcp2515, new_msg.data[1], msg_to_send, my_address);
  }
  else if( (new_msg.data[0] == olleh) and ( new_msg.can_id == my_address ) ) {
    num_of_ollehs += 1;
    // (2 + num_responses) because nodes_addresses[0] is for broadcast and nodes_addresses[1] is my addr
    nodes_addresses = (byte*)realloc(nodes_addresses, (2 + num_of_ollehs)*sizeof(byte)); 
    //So 1st received response goes to index 2 cause 0 and 1 is for broadcast and my address
    nodes_addresses[num_of_ollehs + 1] = new_msg.data[1];
  }
  //new node entered and said Ready
  else if((new_msg.data[0] == Ready) && new_msg.can_id == 0 && my_state != w8ing_msg){
    msg_to_send = Ready;
    prev_state = my_state;
    my_state = ready_for_calib;
    ready_number += 1;
    for(int i=1; i<number_of_addresses;i++){
      if(nodes_addresses[i] != my_address){
        //send Ready messages for everybody
        delay(1);
        writeMsg( mcp2515, nodes_addresses[i], msg_to_send, my_address);
      }
    }
  }
  else if((new_msg.data[0] == ACK) && new_msg.can_id == my_address){
    //ack number
    ack_number += 1;
  }
  else if((new_msg.data[0] == Ready) && (new_msg.can_id == my_address) && (my_state == ready_for_calib)){
    ready_number+=1;
  }
  else if(!master && my_state == calibration) { 
  //I'm not master, so in calibration I need to check for receiving msgs from the master node
    if(new_msg.data[0] == turnOff_led) {
        pid.led.setBrightness(0);
        msg_to_send = ACK;
        writeMsg( mcp2515, new_msg.data[1], msg_to_send, my_address);
    } else if(new_msg.data[0] == read_offset_value) {
        my_offset = pid.ldr.luxToOutputVoltage( pid.ldr.getOutputVoltage(), true);
        msg_to_send = ACK;
        writeMsg( mcp2515, new_msg.data[1], msg_to_send, my_address);
    } else if(new_msg.data[0] == read_lux_value) {
        //Compute my K
        my_gains_vect[retrieve_index(nodes_addresses, number_of_addresses, byte(new_msg.data[1])) - 1] = ( pid.ldr.luxToOutputVoltage( pid.ldr.getOutputVoltage(), true) - my_offset ) / 255.0;
        msg_to_send = ACK;
        writeMsg( mcp2515, new_msg.data[1], msg_to_send, my_address);
    } else if( (new_msg.data[0] == your_time_master) && (my_address == new_msg.can_id) ) {
        master = true;
        prev_state = offset_calc; //Simulate I was in offset_calc(but I was not)
    } else if(new_msg.data[0] == start_consensus) {
      prev_state = my_state;
      my_state = consensus; 
    }
  }
}


//************** END STATE MACHINE FUNCTIONS ****************

void loop() {
  new_msg = readMsg();

  //Check messages received from the can-bus
  if(new_msg.can_id != 100000)
    check_messages(mcp2515);
  
  switch(my_state){
    case booting:{
      booting_function(mcp2515);
      starting_time = micros();
    }break;
    case w8ing_msg:{
      w8ing_function(mcp2515);
    }break; 
    case ready_for_calib:{
      ready_function();
    }break;
    case offset_calc:{ 
      offset_function(mcp2515);
    }break;
    case calibration:{ 
      calib_function(mcp2515);
    }break;
    case w8ing_ack:{
      w8ing_ack_function(mcp2515);
    }break;
    case consensus:{
      
    }break;
    case w8ing_ldr_read:{ //This because we can't use delay(50) while reading from ldr
      if( (micros() - starting_time >= 2000000) && (prev_state == calibration) ) {
        prev_state = my_state;
        my_state = calibration;
        starting_time = 0;
      }
    }break;
    default:{
      Serial.println("DEFAULT");
    }break;
  }

  if(DEBUG) {
    Serial.print("S: " + String(my_state));
    Serial.print(" - ");
    Serial.print("M: " + String(master));
    Serial.print(" - ");
    for(int i=0; i<number_of_addresses; i++) {
      Serial.print(nodes_addresses[i]);
      Serial.print(" ");
    }
    Serial.println();
    Serial.println("OFFSET = " + String(my_offset));
    for(int i=0; i<number_of_addresses-1; i++) {
      Serial.print(my_gains_vect[i], 4);
      Serial.print(" ");
    }
    Serial.println();
  }  

  if(Serial.available()){ hub(number_of_addresses - 1); } 

  if (LOOP){
    pid.led.setBrightness( pid.getU() );

    if(SIMULATOR){
      pid.simulator( true );
      pid.output();
    }
  }
}


// interrupt service routine 
ISR(TIMER1_COMPA_vect)        
{ 
  pid.computeFeedbackGain( analogRead( pid.getLdrPin() ) );
  // Serial.println( millis() );
  if(transmiting)
    { 
      //counter++;
      // pwm
      Serial.write("+");
      Serial.write("s");
      Serial.write(1);
      float_2_bytes( pid.ldr.luxToOutputVoltage( 5.0*analogRead( pid.getLdrPin() ) / 1023.0, true) );
      float_2_bytes( 100.0*pid.getU()/255.0  );
      //float_2_bytes(counter);
      //float_2_bytes(counter);
    }
} 
