#include "controller.h"
#include <SPI.h>
#include <mcp2515.h>
#include "can_buffer.cpp"
#include "circular_buffer.h"
#include "comms.h"
#include "hub.hpp"


MCP2515 mcp2515(10);
// INIT PID
ControllerPid pid(3, A0);

/**********************************************
 * GLOBAL VARIABLES
***********************************************/ 

/*-------------------------------------------
 * ARDUINO PARAMETERS                         |
---------------------------------------------*/
float m;
float b;
byte my_address;


/*
 * AINDA NÂO SEI BEM O QUE FAZEM ESTAS
*/
boolean LOOP = false;
boolean SIMULATOR = false;
boolean DEBUG = true;

long ack_time = 0;
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
enum msg_types {hello=0, olleh=1, Ready=2, read_offset_value=3, read_lux_value = 4, turnOn_led=5, turnOff_led=6, your_time_master=7, ACK=8, start_consensus = 9};
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

//array with the upcoming messages
int frames_arr_size = 0;
can_frame *frames;

//circular buffer that saves the last 10 messages
volatile can_frame_stream cf_stream;

//flags
volatile bool interrupt = false;
volatile bool mcp2515_overflow = false;
volatile bool arduino_overflow = false;

//number of messages recieved
int n_messages = 0;

/********************************
 * RPI COMMUNICATION
********************************/
byte *serial_buffer;
int serial_buff_size = 0;
bool im_hub = false;


long starting_time = 0;

/*-----------------------------------------------------|
 * FUNCTIONS HEADERS                                     |
-------------------------------------------------------|*/
MCP2515::ERROR write(uint32_t id, uint32_t val);
void writeMsg(int id, byte msg_type, byte sender_address, float value = 0);
void smallMsg(int id, byte msg_type, byte sender_address);
float getValueMsg(can_frame frame);
can_frame* readMsg(int *frames_arr_size);
void initCalibration();
void booting_function(int msg_to_send);
void initCalibration(float **offset);

/*------------------------------------|
 * PROGRAM BEGINS                     |
--------------------------------------|*/

//THE READ INTERRUPT ROUTINES
void irqHandler() {
  can_frame frm;
  uint8_t irq = mcp2515.getInterrupts();
  //check messages in buffer 0
  if ( irq & MCP2515::CANINTF_RX0IF ) {
    mcp2515.readMessage( MCP2515::RXB0, & frm );
    if ( !cf_stream.put( frm ) ) //no space
      arduino_overflow = true;
  }

  //check messages in buffer 1
  if ( irq & MCP2515::CANINTF_RX1IF ) {
    mcp2515.readMessage( MCP2515::RXB1, & frm);
    if ( !cf_stream.put( frm ) ) //no space
      arduino_overflow = true;
  }
  irq = mcp2515.getErrorFlags(); //read EFLG
  if ( (irq & MCP2515::EFLG_RX0OVR) |
       (irq & MCP2515::EFLG_RX1OVR) ) {
    mcp2515_overflow = true;
    mcp2515.clearRXnOVRFlags();
  }
  mcp2515.clearInterrupts();
  interrupt = true; //notify loop()
}

//THE WRITE ROUTINES
//union to pack/unpack long ints into bytes
union my_can_msg {
  unsigned long value;
  unsigned char bytes[4];
};

MCP2515::ERROR write(uint32_t id, uint32_t val){
  can_frame frame;
  frame.can_id = id;
  frame.can_dlc = 4;
  my_can_msg msg;
  msg.value = val; //pack data
  if(DEBUG) {
    Serial.println("NEW MESSAGE:\n");
    Serial.println("ID: " + String(frame.can_id));
  }
  for ( int i = 0; i < 4; i++ ){ //prepare can message
    frame.data[i] = msg.bytes[i];
    if(DEBUG)
      Serial.println(msg.bytes[i]);
  }
  //send data
  return mcp2515.sendMessage(&frame);
}
/*---------------------------------------------------------|
 * Message with no values only msg_type and sender_address |
-----------------------------------------------------------|*/
void writeMsg(int id, byte msg_type, byte sender_address, float value = 0){
  if(value){
    //write message with values
    byte* inBytes = (byte*)malloc(2*sizeof(byte));
    
    inBytes = convertFloat2Bytes(value);
    unsigned long val = (unsigned long)msg_type;
    val += (unsigned long)my_address<<8;
    val += (unsigned long)inBytes[1]<<16;
    val += (unsigned long)inBytes[0]<<24;
    if ( write( id , val ) != MCP2515::ERROR_OK )
      Serial.println( "\t\t\t\tMCP2515 TX Buf Full" );
    free(inBytes);
  }
  else{
    smallMsg(id, msg_type, sender_address);
  }
}
void smallMsg(int id, byte msg_type, byte sender_address) {
  int val = (sender_address<<8)+msg_type;  //Convert 2 bytes into 1 int, with byte 0 being msg_type and byte 1 being sender addr
  if ( write( id , val ) != MCP2515::ERROR_OK )
      Serial.println( "\t\t\t\tMCP2515 TX Buf Full" );
}

can_frame* readMsg(int *frames_arr_size){
  if ( interrupt ) {
    interrupt = false;
    can_frame *frames;
    int n_messages = 0;
    if ( mcp2515_overflow ) {
      Serial.println( "\t\t\t\tMCP2516 RX Buf Overflow" );
      mcp2515_overflow = false;
    }

    if( arduino_overflow ) {
      Serial.println( "\t\t\t\tArduino Buffers Overflow" );
      arduino_overflow = false;
    }
    
    can_frame frame;
    bool has_data;
    cli(); has_data = cf_stream.get( frame ); sei();
    while( has_data ){
        n_messages++;
        frames = (can_frame*)realloc(frames,sizeof(can_frame)*n_messages);
        frames[n_messages-1] = frame;
        cli(); has_data = cf_stream.get( frame ); sei();
    }
    *frames_arr_size = n_messages;
    return frames;
  }
}

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
  nodes_addresses = (byte*)malloc(2*sizeof(byte)); //Because we will be address '1', cause '0' is for broadcast
  nodes_addresses[0] = 0;
  nodes_addresses[1] = my_address;
  number_of_addresses = 2;  
}

/*
 * FUNCTION to get the float value of CAN message
*/
float getValueMsg(can_frame frame){
  byte value[2];    
  value[0] = frame.data[2];
  value[1] = frame.data[3];
  if(DEBUG)
    Serial.println("3rd byte: " + String(value[0]) + "\t4th byte: " + String(value[1]));
  float x = bytes2float(value);
  if(DEBUG)
    Serial.println("VALOR Recebido: " + String(x));
  return x;
}

//*************** STATE MACHINE FUNCTIONS ************************
/*-----------------------------------------------|
  Function when the node is in booting state     |
-------------------------------------------------|*/
void booting_function(msg_types msg_to_send){
  msg_to_send = hello;
  writeMsg(0, msg_to_send, my_address);
  prev_state = my_state;
  my_state = w8ing_msg;
}

/*-----------------------------------------------|
  Function when tht node is waiting for ollehs   |
-------------------------------------------------|*/
void w8ing_function(){
  if(micros() - starting_time >= 2000000){
    //Time has passed, I'm the only node on grid and I'm ready for calib
    prev_state = my_state;
    my_state = ready_for_calib;
  
    if(num_of_ollehs > 0) {
      number_of_addresses = 2 + num_of_ollehs;
      bubbleSort(nodes_addresses, number_of_addresses);
      //send broadcast READY_CALIB
      msg_to_send = Ready;
      writeMsg(0, msg_to_send, my_address);
      num_of_ollehs = 0;
    }
  }
  //If I received response msg, then there are other nodes on the grid
  for(int i=0; i < frames_arr_size; i++) {
    if( (frames[i].data[0] == olleh) and ( frames[i].can_id == my_address ) ) {
      num_of_ollehs += 1;
      // (2 + num_responses) because nodes_addresses[0] is for broadcast and nodes_addresses[1] is my addr
      nodes_addresses = (byte*)realloc(nodes_addresses, (2 + num_of_ollehs)*sizeof(byte)); 
      //So 1st received response goes to index 2 cause 0 and 1 is for broadcast and my address
      nodes_addresses[num_of_ollehs + 1] = frames[i].data[1];
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
  pwm_vect = (int*)malloc((number_of_addresses - 1)*sizeof(int));
  my_gains_vect = (float*)malloc((number_of_addresses - 1)*sizeof(float));
  my_luminance = -1;
  for(int i=0; i < frames_arr_size; i++){
    if(frames[i].data[0] == Ready && frames[i].can_id == my_address){
      ready_number+=1;
    }
  }
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
void offset_function(){
  //I'm the master and the first node so I enter first here for computing the offsets for all, only once
  pid.led.setBrightness(0); //Turn off myLed
  if(prev_state == ready_for_calib) {
    msg_to_send = turnOff_led;
    writeMsg(0, msg_to_send, my_address);
    prev_state = my_state;
    my_state = w8ing_ack;
    ack_time = micros();
  } else if( (prev_state == w8ing_ack) && (my_offset < 0) ) { 
    //Now I know all the nodes turned off their led
    my_offset = pid.ldr.luxToOutputVoltage(pid.ldr.getOutputVoltage(), true);
    msg_to_send = read_offset_value;
    writeMsg(0, msg_to_send, my_address);
    prev_state = my_state;
    my_state = w8ing_ack;
    ack_time = micros();
  } else if( (prev_state == w8ing_ack) && (my_offset >= 0) ) { 
    //Now I know every node computed their offset so I'm for entering calibration state
    prev_state = my_state;
    my_state = calibration; 
  }
}

/*------------------------------------------------------|
  Function when the node is waiting for all the acks    |
--------------------------------------------------------|*/

void w8ing_ack_function(){
  if( ( ack_number == number_of_addresses-2 ) && ( prev_state == offset_calc ) ){
    ack_number=0; //reset
    prev_state = my_state;
    my_state = offset_calc;
  } else if( (prev_state == offset_calc) && ( micros() - ack_time > 1000000 ) ) {
    ack_time = 0;
    msg_to_send = turnOff_led;
    writeMsg(0, msg_to_send, my_address);
  } else if(( ack_number == number_of_addresses-2 ) && ( prev_state == calibration )) {
    ack_number=0; //reset
    prev_state = my_state;
    my_state = calibration;
  } else if( (prev_state == calibration) && ( micros() - ack_time > 1000000 ) ) {
    ack_time = 0;
    msg_to_send = read_lux_value;
    writeMsg(0, msg_to_send, my_address);
  }
}

/*------------------------------------------------------|
  Function to calibrate K                               |
--------------------------------------------------------|*/

void calib_function(){
  //I'm not the first addr so when I'm the master I only do the calibration part of entire Calibration
  if(!master) { 
    //I'm not master, so in calibration I need to check for receiving msgs from the master node
    for(int i=0; i < frames_arr_size; i++) {
      if(frames[i].data[0] == turnOff_led) {
          pid.led.setBrightness(0);
          msg_to_send = ACK;
          writeMsg(frames[i].data[1], msg_to_send, my_address);
      } else if(frames[i].data[0] == read_offset_value) {
          my_offset = pid.ldr.luxToOutputVoltage( pid.ldr.getOutputVoltage(), true);
          msg_to_send = ACK;
          writeMsg(frames[i].data[1], msg_to_send, my_address);
      } else if(frames[i].data[0] == read_lux_value) {
          //Compute my K
          my_gains_vect[retrieve_index(nodes_addresses, number_of_addresses, byte(frames[i].data[1])) - 1] = ( pid.ldr.luxToOutputVoltage( pid.ldr.getOutputVoltage(), true) - my_offset ) / 255.0;
          msg_to_send = ACK;
          writeMsg(frames[i].data[1], msg_to_send, my_address);
      } else if( (frames[i].data[0] == your_time_master) && (my_address == frames[i].can_id) ) {
          master = true;
          prev_state = offset_calc; //Simulate I was in offset_calc(but I was not)
      } else if(frames[i].data[0] == start_consensus) {
        prev_state = my_state;
        my_state = consensus; 
      }
    }
  } else { 
    //If this is my round of being the master
    if( prev_state == offset_calc ) {
      pid.led.setBrightness(255); //I know that everyone's led is turned off so I can turn on mine
      prev_state = my_state;
      my_state = w8ing_ldr_read;
      starting_time = micros();
    } else if(prev_state == w8ing_ldr_read) {
      //Compute my K
      my_gains_vect[retrieve_index(nodes_addresses, number_of_addresses, my_address)-1] = ( pid.ldr.luxToOutputVoltage( pid.ldr.getOutputVoltage(), true) - my_offset ) / 255.0;
      msg_to_send = read_lux_value;
      writeMsg(0, msg_to_send, my_address);
      prev_state = my_state;
      my_state = w8ing_ack;
      ack_time = micros();
    } else if(prev_state == w8ing_ack) {
      pid.led.setBrightness(0); //Turn off led again for not disturbing next reads
      //Pass the grenade to the next node if exists (check if I'm the last one)
      if(my_address == nodes_addresses[number_of_addresses-1]) {
        prev_state = my_state;
        my_state = consensus;
      } else {
        master = false;
        msg_to_send = your_time_master;
        writeMsg(nodes_addresses[retrieve_index(nodes_addresses, number_of_addresses, my_address) + 1], msg_to_send, my_address); //Send msg to the next node for him to become master
      }
    }        
  }
}

/*------------------------------------------------------|
  Function check end loop messages                      |
--------------------------------------------------------|*/

void check_messages(){
  for(int i=0; i < frames_arr_size; i++) {
    //Check if received msg is from a new entering node...
    if((frames[i].data[0] == hello) && my_state != w8ing_msg) { 
      bool already_on_addresses = false;
      for(int j=0; j < number_of_addresses; j++) {
        if(frames[i].data[1] == nodes_addresses[j]) {
          already_on_addresses = true;
        }
      }
      //If this address have never been to my addresse's vector
      if(already_on_addresses == false) { 
        number_of_addresses += 1;
        nodes_addresses = (byte*)realloc(nodes_addresses, number_of_addresses*sizeof(byte));
        nodes_addresses[number_of_addresses-1] = frames[i].data[1];
        bubbleSort(nodes_addresses, number_of_addresses);
      }
      msg_to_send = olleh;
      smallMsg(frames[i].data[1], msg_to_send, my_address);
      delay(2);
    }
    //new node entered and said Ready
    else if((frames[i].data[0] == Ready) && frames[i].can_id == 0 && my_state != w8ing_msg){
      msg_to_send = Ready;
      prev_state = my_state;
      my_state = ready_for_calib;
      ready_number += 1;
      for(int i=1; i<number_of_addresses;i++){
        if(nodes_addresses[i] != my_address){
          //send Ready messages for everybody
          writeMsg(nodes_addresses[i], msg_to_send, my_address);
        }
      }
    }
    else if((frames[i].data[0] == ACK) && frames[i].can_id == my_address){
      //ack number
      ack_number += 1;
    }
  }
}

//************** END STATE MACHINE FUNCTIONS ****************

void loop() {
  frames_arr_size = 0;
  frames = readMsg(&frames_arr_size);
  
  switch(my_state){
    case booting:{
      booting_function(msg_to_send);
      starting_time = micros();
    }break;
    case w8ing_msg:{
      w8ing_function();
    }break; 
    case ready_for_calib:{
      ready_function();
    }break;
    case offset_calc:{ 
      offset_function();
    }break;
    case calibration:{ 
      calib_function();
    }break;
    case w8ing_ack:{
      w8ing_ack_function();
    }break;
    case consensus:{
      
    }break;
    case w8ing_ldr_read:{ //This because we can't use delay(50) while reading from ldr
      if( (micros() - starting_time >= 4000000) && (prev_state == calibration) ) {
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

  //Check messages received from the can-bus  
  check_messages();

  //if(Serial.available()){ hub(number_of_addresses-1); } 
  
  if(!DEBUG) {
    //Check messages received from the Serial bus
    //Doing it in a while() for now, but surely not the best solution because it stops the code on the while
    //I'm doint it cause the serial messages are really small (4bytes)
      while(Serial.available() > 0) { //If received message from RPI via Serial...
        byte inByte = Serial.read();
        serial_buff_size++;
        serial_buffer = (byte*)realloc(serial_buffer, sizeof(byte)*serial_buff_size);
        serial_buffer[serial_buff_size-1] = inByte;
      }
      if(serial_buff_size > 0) {
        if(serial_buff_size == 3 and char(serial_buffer[0]) == 'R' and char(serial_buffer[1]) == 'P' and char(serial_buffer[2]) == 'i') {
          //Then I received the msg from Rpi saying I'm the hub...so I send an ACK msg
          im_hub = true;
        }
    
        serial_buff_size = 0;
        serial_buffer = (byte*)realloc(serial_buffer, sizeof(byte)*serial_buff_size);
      }
  }


  /*if (LOOP){
    if ( Serial.available() ){
      String work_percentage = Serial.readString();  // read input at PWM pin 'led_pin'
      pid.setReferenceLux( work_percentage.toFloat() ); // read input value and ajust the reference brightness
    }  // anytime there is an input

    pid.led.setBrightness( pid.getU() );
    // simulator
    if(SIMULATOR){
      pid.simulator( true );
      pid.output();
    }
  }*/
}


// interrupt service routine 
ISR(TIMER1_COMPA_vect)        
{ 
  pid.computeFeedbackGain( analogRead( pid.getLdrPin() ) );
  // Serial.println( millis() );
} 
