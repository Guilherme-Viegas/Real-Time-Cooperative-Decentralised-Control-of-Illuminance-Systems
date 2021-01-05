#include "controller.h"
#include <SPI.h>
#include <mcp2515.h>
#include "can_buffer.cpp"
#include "consensus.hpp"
//__attribute__((optimize("O0")))

#define BUFFER_SIZE 5 // number of char to read plus \0 (For hub)

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


boolean LOOP = false;
boolean SIMULATOR = false;
boolean DEBUG = false;

/*-------------------------------------|
 * GENERIC VARIABLES                   |
---------------------------------------|*/
float my_cost = 1;
float lower_L_bound = 20.0;
float lower_L_occupied = 50.0;
float lower_L_unoccupied = 20.0;
bool occupancy = false; // true if desk is occupied


/*-------------------------------------|
 * STATE MACHINE OF THE PROGRAM        |
---------------------------------------|*/
enum state_machine {booting = 0, w8ing_olleh = 1, standard = 2, w8ing_ack = 3 , turn_off = 4, read_offset = 5, calibration = 6, w8ing_ldr_read = 7, ready_consensus=8, w8ing_consensus_msgs=9};
state_machine my_state = booting;
state_machine prev_state = booting;

/*------------------------|
 * TYPE OF MESSAGES       |
--------------------------|*/
enum msg_types {hello=1, olleh=2, ack=3, turn_off_led=4, read_offset_value=5, read_gain=6, your_time_master=7, start_consensus=8, sending_consensus_val=9, turn_max_led=10, hub_set_occupancy=255, hub_sending_ack=254, hub_set_bound_occupied=253, hub_set_bound_unoccupied=252, hub_set_cost=251, hub_request_stream=250, hub_sending_stream_lux=249, hub_sending_stream_dimming=248};
msg_types msg_to_send;
/*-------------------------------------------
 * VARIABLES FOR THE CALIBRATION            |
*-------------------------------------------*/
bool master = false;

//L_i = [ki1, ki2, ..., kiN]^T*[d_avg, ..., di, ..., d_avg] + o_i
float my_offset = -1;
float my_gains_vect[3] = {0};

/*--------------------------------------------|
 * CAN BUS COMMUNICATION USEFUL VARIABLES     |
----------------------------------------------|*/
//array of variable size depending on number of nodes
byte nodes_addresses[4];  
int number_of_addresses = 0;


can_frame new_msgs[20];
int frames_arr_size = 0;

//circular buffer that saves the last 20 messages
volatile can_frame_stream cf_stream;

//flags
volatile bool interrupt = false;
volatile bool mcp2515_overflow = false;
volatile bool arduino_overflow = false;

long waiting_time = 0;

long ack_time = 0;
int ack_number = 0;
/********************************
 * RPI COMMUNICATION
********************************/
byte *serial_buffer;
int serial_buff_size = 0;
bool im_hub = false;
volatile bool transmitting = false;
double counter = 0;
byte address_to_send_stream = 1;

float lux_stream = 0;

/********************************
 * CONSENSUS VARIABLES
********************************/
Consensus consensus;
int num_consensus_msgs = 0;
float tmp_received_dimmings[3][3] = {{0}};
float finalDimming = 0;
byte current_sent_msgs = 0;
byte welcome[BUFFER_SIZE];

/*-----------------------------------------------------|
 * FUNCTIONS HEADERS                                     |
-------------------------------------------------------|*/
MCP2515::ERROR write(uint32_t id, uint32_t val);
void writeMsg(int id, byte msg_type, byte sender_address, float dimming=-1, byte index=-1);
void writeMsgWithFloat(int id, byte msg_type, byte sender_address, float value = 0);
void smallMsg(int id, byte msg_type, byte sender_address);
//float getValueMsg(can_frame frame);
can_frame* readMsg(int * frames_arr_size);
//*****HUB******
byte* float_2_bytes(float fnum, bool flag);
float bytes2float(byte * myBytes);
float bytes_2_float_2decimals(byte * myBytes);
float bytes_2_float_2decimals(byte * myBytes);
void greeting(int numLamps);
bool hub();
void send_time();


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
  if ( mcp2515.getInterrupts() & MCP2515::CANINTF_RX1IF ) {
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
    Serial.println(F("NEW MESSAGE:\n"));
    Serial.println("ID: " + String(frame.can_id));
  }
  for ( byte i = 0; i < 4; i++ ){ //prepare can message
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
void writeMsg(int id, byte msg_type, byte sender_address, float dimming=-1, byte index=-1){
  if( (dimming != -1) and (index != -1) ) {
    byte* inBytes = (byte*)malloc(2*sizeof(byte));
    inBytes = float_2_bytes_2decimals(dimming);
    unsigned long val = (unsigned long)msg_type;
    val += (unsigned long)my_address<<8;
    val += (unsigned long)inBytes[1]<<16;
    val += (unsigned long)inBytes[0]<<24;
    if ( write( index , val ) != MCP2515::ERROR_OK )
      Serial.println( F("\t\t\t\tMCP2515 TX Buf Full") );
    free(inBytes);
  }
  else{
    smallMsg(id, msg_type, sender_address);
  }
}

void writeMsgWithFloat(int id, byte msg_type, byte sender_address, float value = 0){
  //write message with values
  byte* inBytes = (byte*)malloc(2*sizeof(byte));
  
  inBytes = float_2_bytes(value, true);
  unsigned long val = (unsigned long)msg_type;
  val += (unsigned long)my_address<<8;
  val += (unsigned long)inBytes[1]<<16;
  val += (unsigned long)inBytes[0]<<24;
  if ( write( id , val ) != MCP2515::ERROR_OK )
    Serial.println( F("\t\t\t\tMCP2515 TX Buf Full") );
  free(inBytes);
}

void smallMsg(int id, byte msg_type, byte sender_address) {
  int val = (sender_address<<8)+msg_type;  //Convert 2 bytes into 1 int, with byte 0 being msg_type and byte 1 being sender addr
  if ( write( id , val ) != MCP2515::ERROR_OK )
      Serial.println( F("\t\t\t\tMCP2515 TX Buf Full") );
}

void readMsg(int *frames_size, can_frame frames[20]){
  int counter = 0;
  if ( mcp2515_overflow ) {
    Serial.println( F("\t\t\t\tMCP2516 RX Buf Overflow") );
    mcp2515_overflow = false;
  }

  if( arduino_overflow ) {
    Serial.println( F("\t\t\t\tArduino Buffers Overflow") );
    arduino_overflow = false;
  }
  
  bool has_data;
  can_frame frame;
  cli(); has_data = cf_stream.get( frame ); sei();
  while( has_data ) {
      counter++;
      frames[counter-1] = frame;
      if(DEBUG)
        Serial.println("REC: " + String(frames[counter-1].can_id) + " " + String(frames[counter-1].data[0]) + " " + String(frames[counter-1].data[1]) + " " + String(frames[counter-1].data[2]) + " " + frames[counter-1].data[3]);
      cli(); has_data = cf_stream.get( frame ); sei();
    }
  *frames_size = counter;
}

void setup() {
  Serial.begin(230400);

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
  mcp2515.setBitrate(CAN_1000KBPS, MCP_20MHZ);
  mcp2515.setNormalMode();
  //mcp2515.setRegister(MCP2515::RXB0CTRL_BUKT, 1);
  
  pid.ldr.setGain( pid.getLedPin(), m, b );
  pid.ldr.t_tau_up.setParametersABC( 29.207246, -0.024485, 11.085226); // values computed in the python file
  pid.ldr.t_tau_down.setParametersABC( 15.402250,  -0.015674, 8.313158); // values computed in the python file

  if(DEBUG)
    Serial.println(F("Set up completed"));
  
  //if(pid.has_feedback()){initInterrupt1();}
  initInterrupt1();
  nodes_addresses[0] = 0;
  nodes_addresses[1] = my_address;
  number_of_addresses = 2;  
  delay(4000);
}

/*
 * FUNCTION to get the float value of CAN message
*//*
float getValueMsg(can_frame frame){
  byte value[2];    
  value[0] = frame.data[2];
  value[1] = frame.data[3];
  float x = bytes2float(value);
  return x;
}*/

//*************** STATE MACHINE FUNCTIONS ************************
/*-----------------------------------------------|
  Function when the node is in booting state     |
-------------------------------------------------|*/
void booting_function(){
  msg_to_send = hello;
  writeMsg(0, msg_to_send, my_address);
  prev_state = my_state;
  my_state = w8ing_olleh;
}

/*-----------------------------------------------|
  Function when tht node is waiting for ollehs   |
-------------------------------------------------|*/
void w8ing_olleh_function(){
  if(millis() - waiting_time >= 5000){
    bubbleSort(nodes_addresses, number_of_addresses);
    prev_state = my_state;
    if(nodes_addresses[1] == my_address) { //If i'm the 1st node on addr vector then I run offset computation once and only once
      master = true;
      my_state = turn_off;
    }
    else {
      master = false;
      my_state = standard; //All the other nodes enter calibration state
    }
    waiting_time = 0;
    
  }
}

/*-----------------------------------------------|
  Ask for them to turn off the led               |
-------------------------------------------------|*/
void turn_off_led_function() {
  pid.led.setBrightness(0); //Turn off myLed
  if(number_of_addresses > 2) {
      msg_to_send = turn_off_led;
      writeMsg(0, msg_to_send, my_address);
   }
   prev_state = my_state;
   my_state = w8ing_ack;
   ack_time = millis();
   ack_number = 0;
}

/*-----------------------------------------------|
  Ask for them to read offset                    |
-------------------------------------------------|*/
void read_offset_function() {
  my_offset = pid.ldr.luxToOutputVoltage(pid.ldr.getOutputVoltage(), true);
  if(number_of_addresses > 2) {
    msg_to_send = read_offset_value;
    writeMsg(0, msg_to_send, my_address);
  }
  prev_state = my_state;
  my_state = w8ing_ack;
  ack_time = millis();
  ack_number = 0;
}

/*-----------------------------------------------|
  For generating gains matrix                    |
-------------------------------------------------|*/
void calibration_function() {
  if(master) {
    if(prev_state == w8ing_ldr_read) {
      my_gains_vect[retrieve_index(nodes_addresses, number_of_addresses, my_address)-1] = ( pid.ldr.luxToOutputVoltage( pid.ldr.getOutputVoltage(), true) - my_offset ) / 255.0;
      if(number_of_addresses > 2) {
        msg_to_send = read_gain;
        writeMsg(0, msg_to_send, my_address);
      }
      prev_state = my_state;
      my_state = w8ing_ack;
      ack_time = millis();
      ack_number = 0;
    } else if( prev_state == w8ing_ack ) {
        pid.led.setBrightness(0);
        if(my_address == nodes_addresses[number_of_addresses-1]) {
          lower_L_bound = occupancy == true ? lower_L_occupied : lower_L_unoccupied;
          byte flag = checkBoundsOnAskedLux(lower_L_bound);
          if(flag == -1) {
              prev_state = my_state;
              my_state = standard;
              msg_to_send = turn_off_led;
              writeMsg(0, msg_to_send, msg_to_send);
              finalDimming = 0;
              pid.setReferenceLux( (finalDimming*255.0)/100.0 );
              LOOP = true;
              SIMULATOR = true;
          } else if(flag == 1) {
              prev_state = my_state;
              my_state = standard;
              msg_to_send = turn_max_led;
              writeMsg(0, msg_to_send, msg_to_send);
              finalDimming = 100;
              pid.setReferenceLux( (finalDimming*255.0)/100.0 );
              LOOP = true;
              SIMULATOR = true;
          } else {
              prev_state = my_state;
              my_state = ready_consensus;
              consensus.Init(lower_L_bound, my_offset, my_gains_vect, my_cost, my_address, number_of_addresses, nodes_addresses);
              msg_to_send = start_consensus;
              writeMsg(0, msg_to_send, my_address);
              LOOP = false;
              SIMULATOR = false;
          }
        } else {
          master = false;
          if(number_of_addresses > 2) {
            msg_to_send = your_time_master;
            writeMsg(nodes_addresses[retrieve_index(nodes_addresses, number_of_addresses, my_address) + 1], msg_to_send, my_address); //Send msg to the next node for him to become master
          }
          prev_state = my_state;
          my_state = standard;
        }
    }
  }
}


/*-----------------------------------------------|
  Waiting for ACKs                               |
-------------------------------------------------|*/
void w8ing_ack_function() {
  if( (ack_number == number_of_addresses-2) and (prev_state == turn_off) ) {
      ack_number = 0;
      prev_state = my_state;
      my_state = read_offset;
  } else if( (prev_state == turn_off) and (millis() - ack_time > 3000) ) {
      if(number_of_addresses > 2) {
        msg_to_send = turn_off_led;
        writeMsg(0, msg_to_send, my_address);
      }
      ack_number = 0;
      ack_time = millis();  
  } else if( (ack_number == number_of_addresses-2) and (prev_state == read_offset) ) {
      ack_number = 0;
      prev_state = my_state;
      my_state = w8ing_ldr_read;
      pid.led.setBrightness(255);
      waiting_time = millis();
  } else if( (prev_state == read_offset) and (millis() - ack_time > 3000) ) {
      if(number_of_addresses > 2) {
          msg_to_send = read_offset_value;
          writeMsg(0, msg_to_send, my_address);
      }
      ack_number = 0;
      ack_time = millis();
  } else if( (ack_number == number_of_addresses-2) and (prev_state == calibration) ) {
      ack_number = 0;
      prev_state = my_state;
      my_state = calibration;
  } else if( (prev_state == calibration) and (millis() - ack_time > 3000) ) {
      if(number_of_addresses > 2) {
          msg_to_send = read_gain;
          writeMsg(0, msg_to_send, my_address);
      }
      ack_number = 0;
      ack_time = millis();
  }
}

/*-----------------------------------------------|
  Waiting for others consensus proposals         |
-------------------------------------------------|*/
void w8ing_consensus_msgs_function() {
  if( num_consensus_msgs >= (pow(number_of_addresses-1, 2) - (number_of_addresses-1)) ) {
    num_consensus_msgs -= (pow(number_of_addresses-1, 2) - (number_of_addresses-1));
    consensus.updateDimmings( tmp_received_dimmings);
    prev_state = my_state;
    my_state = ready_consensus;
    consensus.incrementIterations();
    consensus.updateAverage();
    consensus.updateLagrandeMultipliers( );
    if(consensus.getCurrentIteration() >= max_iterations) {
      prev_state = my_state;
      my_state = standard;
      finalDimming = consensus.getFinalDimming();
      pid.setReferenceLux( (finalDimming*255.0)/100.0 );
      LOOP = true;
      SIMULATOR = true;
      if(DEBUG) {
        Serial.println("DIM: " + String(finalDimming));
        Serial.println("OFFSET = " + String(my_offset));
        for(byte i=0; i<number_of_addresses-1; i++) {
          Serial.print(my_gains_vect[i], 4);
          Serial.print(" ");
        }
        Serial.println();
      }
    }
  }
}

/*------------------------------------------------------|
  Function check end loop messages                      |
--------------------------------------------------------|*/

void check_messages(can_frame new_msg){
  if( new_msg.data[0] == hello ) { 
    if( !isValueInside(nodes_addresses, number_of_addresses, new_msg.data[1]) ) {
      number_of_addresses++;
      nodes_addresses[number_of_addresses-1] = new_msg.data[1];
    }
    msg_to_send = olleh;
    writeMsg(new_msg.data[1], msg_to_send, my_address);
  } else if( (new_msg.data[0] == olleh) and ( new_msg.can_id == my_address ) ) {
      if( !isValueInside(nodes_addresses, number_of_addresses, new_msg.data[1]) ) {
        number_of_addresses++;
        nodes_addresses[number_of_addresses-1] = new_msg.data[1];
      }
  } else if( (new_msg.data[0] == ack) and ( new_msg.can_id == my_address ) ) {
      ack_number++;
  } else if( new_msg.data[0] == turn_off_led ) {
      pid.led.setBrightness(0);
      finalDimming = 0;
      if(my_offset != -1) {
        LOOP = true;
        SIMULATOR = true;
        pid.setReferenceLux( (finalDimming*255.0)/100.0 );
      }
      msg_to_send = ack;
      writeMsg(new_msg.data[1], msg_to_send, my_address);
  } else if(new_msg.data[0] == turn_max_led ) {
      finalDimming = 100;
      pid.setReferenceLux( (finalDimming*255.0)/100.0 );
      LOOP = true;
      SIMULATOR = true;
  } else if( new_msg.data[0] == read_offset_value  ) {
      my_offset = pid.ldr.luxToOutputVoltage( pid.ldr.getOutputVoltage(), true);
      msg_to_send = ack;
      writeMsg(new_msg.data[1], msg_to_send, my_address);
  } else if( new_msg.data[0] == read_gain  ) {
      my_gains_vect[retrieve_index(nodes_addresses, number_of_addresses, new_msg.data[1]) - 1] = ( pid.ldr.luxToOutputVoltage( pid.ldr.getOutputVoltage(), true) - my_offset ) / 255.0;
      msg_to_send = ack;
      writeMsg(new_msg.data[1], msg_to_send, my_address);
  } else if( (new_msg.data[0] == your_time_master) and (new_msg.can_id == my_address) ) {
      master = true;
      prev_state = my_state;
      my_state = w8ing_ldr_read;
      pid.led.setBrightness(255);
      waiting_time = millis();
  } else if( new_msg.data[0] == start_consensus) {
      prev_state = my_state;
      my_state = ready_consensus;
      lower_L_bound = occupancy == true ? lower_L_occupied : lower_L_unoccupied;
      consensus.Init(lower_L_bound, my_offset, my_gains_vect, my_cost, my_address, number_of_addresses, nodes_addresses);
      LOOP = false;
      SIMULATOR = false;
  } else if( new_msg.data[0] == sending_consensus_val ) {
      byte value_received[2] = {new_msg.data[2], new_msg.data[3]};
      //Index of the node of value received is stored in can_id, because this messages never go to one node specifically
      tmp_received_dimmings[retrieve_index(nodes_addresses, number_of_addresses, new_msg.data[1]) - 1][new_msg.can_id] = bytes_2_float_2decimals(value_received);
      num_consensus_msgs++;
  }

  //*************HUB MESSAGES*********
  if((new_msg.data[0] == hub_set_occupancy) and (new_msg.can_id == my_address)) {
      byte received_val[2] = {new_msg.data[2], new_msg.data[3]};
      bool received_occ = (bool)bytes2float(received_val);
      if(occupancy != received_occ) {
        occupancy = received_occ;
        prev_state = my_state;
        my_state = ready_consensus;
        lower_L_bound = occupancy == true ? lower_L_occupied : lower_L_unoccupied;
        consensus.Init(lower_L_bound, my_offset, my_gains_vect, my_cost, my_address, number_of_addresses, nodes_addresses);
        LOOP = false;
        SIMULATOR = false;
        msg_to_send = start_consensus;
        writeMsg(0, msg_to_send, my_address);
      }
      msg_to_send = hub_sending_ack;
      writeMsgWithFloat(new_msg.data[1], msg_to_send, my_address, bytes2float(received_val));
  } else if((new_msg.data[0] == hub_sending_ack) and (new_msg.can_id == my_address)) {
      Serial.write("+");
      Serial.write(welcome[0]);
      Serial.write(welcome[1]);
      Serial.write(welcome[2]);
      Serial.write(welcome[3]);
  } else if((new_msg.data[0] == hub_set_bound_occupied) and (new_msg.can_id == my_address)) {
      byte received_val[2] = {new_msg.data[2], new_msg.data[3]};
      float new_bound = bytes2float(received_val);
      if((occupancy == true) and (lower_L_bound != new_bound)) {
        prev_state = my_state;
        my_state = ready_consensus;
        lower_L_occupied = new_bound;
        lower_L_bound = occupancy == true ? lower_L_occupied : lower_L_unoccupied;
        consensus.Init(lower_L_bound, my_offset, my_gains_vect, my_cost, my_address, number_of_addresses, nodes_addresses[4]);
        LOOP = false;
        SIMULATOR = false;
        msg_to_send = start_consensus;
        writeMsg(0, msg_to_send, my_address);
      }
      msg_to_send = hub_sending_ack;
      writeMsgWithFloat(new_msg.data[1], msg_to_send, my_address, new_bound);
  } else if((new_msg.data[0] == hub_set_bound_unoccupied) and (new_msg.can_id == my_address)) {
      byte received_val[2] = {new_msg.data[2], new_msg.data[3]};
      float new_bound = bytes2float(received_val);
      if((occupancy == false) and (lower_L_bound != new_bound)) {
        prev_state = my_state;
        my_state = ready_consensus;
        lower_L_unoccupied = new_bound;
        lower_L_bound = occupancy == true ? lower_L_occupied : lower_L_unoccupied;
        consensus.Init(lower_L_bound, my_offset, my_gains_vect, my_cost, my_address, number_of_addresses, nodes_addresses);
        LOOP = false;
        SIMULATOR = false;
        msg_to_send = start_consensus;
        writeMsg(0, msg_to_send, my_address);
      }
      msg_to_send = hub_sending_ack;
      writeMsgWithFloat(new_msg.data[1], msg_to_send, my_address, new_bound);
  } else if((new_msg.data[0] == hub_set_cost) and (new_msg.can_id == my_address)) {
      byte received_val[2] = {new_msg.data[2], new_msg.data[3]};
      float new_cost = bytes2float(received_val);
      prev_state = my_state;
      my_state = ready_consensus;
      my_cost = new_cost;
      lower_L_bound = occupancy == true ? lower_L_occupied : lower_L_unoccupied;
      consensus.Init(lower_L_bound, my_offset, my_gains_vect, my_cost, my_address, number_of_addresses, nodes_addresses);
      LOOP = false;
      SIMULATOR = false;
      msg_to_send = hub_sending_ack;
      writeMsg(new_msg.data[1], msg_to_send, my_address, new_cost);
      msg_to_send = start_consensus;
      writeMsgWithFloat(0, msg_to_send, my_address, new_cost);
  } else if((new_msg.data[0] == hub_request_stream) and (new_msg.can_id == my_address)) {
      transmitting = true;
      address_to_send_stream = new_msg.data[1];
  } else if((new_msg.data[0] == hub_sending_stream_lux) and (new_msg.can_id == my_address)) {
      lux_stream = pid.ldr.luxToOutputVoltage( 5.0*analogRead( pid.getLdrPin() ) / 1023.0, true);
  } else if((new_msg.data[0] == hub_sending_stream_dimming) and (new_msg.can_id == my_address)) {
      Serial.write("+s");
      Serial.write(new_msg.data[1]);
      float_2_bytes( lux_stream, false );
      float_2_bytes( 100.0*pid.getU()/255.0 , false );
  }
}

//************** END STATE MACHINE FUNCTIONS ****************

void loop() {
  if(interrupt) {
    interrupt = false;
    readMsg(&frames_arr_size, new_msgs);
    for(int i=0; i < frames_arr_size; i++) {
      check_messages(new_msgs[i]);
    }
    frames_arr_size = 0;
  }
    
  switch(my_state){
    case booting:{
      booting_function();
      waiting_time = millis();
    }break;
    case standard:{
            
    }break;
    case w8ing_olleh:{
      w8ing_olleh_function();
    }break;
    case w8ing_ack:{
      w8ing_ack_function();
    }break;
    case turn_off:{
      turn_off_led_function();
    }break;
    case read_offset:{
      read_offset_function();
    }break;
    case calibration:{
      calibration_function();
    }break;
    case w8ing_ldr_read:{
      if( (millis() - waiting_time >= 1000) and ( prev_state != ready_consensus ) ) {
        prev_state = my_state;
        my_state = calibration;
        waiting_time = 0;
      }else if( (millis() - waiting_time >= 100) and (prev_state == ready_consensus) ) {
        prev_state = my_state;
        my_state = ready_consensus;
        waiting_time = 0;
      }
    }break;
    case ready_consensus:{
        if(current_sent_msgs == 0)
          consensus.computeValueToSend( );
        msg_to_send = sending_consensus_val;
        float *my_dimmings = consensus.getDimmings();
        if(current_sent_msgs < number_of_addresses-1) {
          writeMsg(0, msg_to_send, my_address, my_dimmings[current_sent_msgs], current_sent_msgs);
          current_sent_msgs++;
          waiting_time = millis();
          prev_state = my_state;
          my_state = w8ing_ldr_read;
        } else {
          current_sent_msgs=0;
          prev_state = my_state;
          my_state = w8ing_consensus_msgs;
        }        
    }break;
    case w8ing_consensus_msgs:{
      w8ing_consensus_msgs_function();
    }break;
    default:{
    }break;
  }

    if(DEBUG) {
    Serial.print("Dim: " + String(finalDimming) + " I " + String(consensus.getCurrentIteration()));
    Serial.print(" - ");
    Serial.print("S: " + String(my_state));
    Serial.print(" - ");
    for(byte i=0; i<number_of_addresses; i++) {
      Serial.print(nodes_addresses[i]);
      Serial.print(" ");
    }
    Serial.println();
    Serial.println("OFFSET = " + String(my_offset));
    for(byte i=0; i<number_of_addresses-1; i++) {
      Serial.print(my_gains_vect[i], 4);
      Serial.print(" ");
    }
    Serial.println();
  }  

  if(Serial.available()){ transmitting = hub(); } 

  if (LOOP){
    pid.led.setBrightness( pid.getU() );

    if(SIMULATOR){
      pid.simulator( false );
      pid.output();
    }
  }
}


// interrupt service routine 
ISR(TIMER1_COMPA_vect)        
{ 
  pid.computeFeedbackGain( analogRead( pid.getLdrPin() ) );
  if(transmitting) { 
    if(address_to_send_stream == my_address) {
      Serial.write("+s");
      Serial.write(my_address);
      float_2_bytes( pid.ldr.luxToOutputVoltage( 5.0*analogRead( pid.getLdrPin() ) / 1023.0, true), false );
      float_2_bytes( 100.0*pid.getU()/255.0 , false );
    } else {
        msg_to_send = hub_sending_stream_lux;
        writeMsgWithFloat(address_to_send_stream, msg_to_send, my_address, (pid.ldr.luxToOutputVoltage( 5.0*analogRead( pid.getLdrPin() ) / 1023.0) , false ));
        msg_to_send = hub_sending_stream_dimming;
        writeMsgWithFloat(address_to_send_stream, msg_to_send, my_address, (100.0*pid.getU()/255.0) );
    }
  }
} 


//******************** HUB FUNCTIONS *********************
bool hub()
{   
  int temp = Serial.read();
  if(temp != '+'){
    return false;
  }
  
  Serial.readBytes(welcome, BUFFER_SIZE);
  byte addr_to_send = (int)welcome[1]-48;
  byte received_val[2] = {welcome[2], welcome[3]};
  float new_bound = bytes2float(received_val);
  bool new_occupancy = (bool)(new_bound);
  if( (char)welcome[0] == 'R' && (char)welcome[1] == 'P' && welcome[2] == (char)'i' && welcome[3] == (char)'G' ) {
        greeting(number_of_addresses-1);
  } else if( (char)welcome[0] == 'R' && (char)welcome[1] == 'P' && (char)welcome[2] == 'i' && (char)welcome[3] == 'E' ) { // last message
      return false;
  } else if( (char)welcome[0] == 'R' && (char)welcome[1] == 'P' && (char)welcome[2] == 'i' && (char)welcome[3] == 'S' ) {   //Stream
    //Preciso que o Almeida me explique o que queria com estes for's a seguir
    //=> AQUI É PRECISO MUDAR: NÃO PODE SER "RPiS", TEM DE SER MESMO O DO ENUNCIADO PQ PRECISO DE SABER QUAL O ARDUINO A TRANSMITIR E FAZER O SAMPLING
    //TMB É PRECISO MUDAR AS SEGUINTES FUNCOES DE SERIAL.WRITE PQ O GAJO SE N FOR ELE TEM DE ENVIAR A PEDIR AS CENAS
      send_time();
      
      bool state[number_of_addresses-1] = {true};
      for(int a=0; a<number_of_addresses-1; a++)
      {
        Serial.write("+o");
        Serial.write(a+1);
        Serial.write(state[a]);
        Serial.write('*');
      }
  
      float lower_bound_occupied[number_of_addresses-1] = {50.0};
      for(int a=0; a<number_of_addresses-1; a++)
      {
        Serial.write("+O");
        Serial.write(a+1);
        float_2_bytes(lower_bound_occupied[a], false);
      }
  
      float lower_bound_unoccupied[number_of_addresses-1] = {20.0};
      for(int a=0; a<number_of_addresses-1; a++)
      {
        Serial.write("+U");
        Serial.write(a+1);
        float_2_bytes(lower_bound_unoccupied[a], false);
      }
  
      float costs[number_of_addresses-1] = {1.0};
      for(int a=0; a<number_of_addresses-1; a++)
      {
        Serial.write("+c");
        Serial.write(a+1);
        float_2_bytes(costs[a], false);
      }
      return true;
  } else if((char)welcome[0] == 'o') {
      if(addr_to_send == my_address) {
        if(occupancy != new_occupancy) { //Only if it's different I need to do another consensus
          occupancy = new_occupancy;
          prev_state = my_state;
          my_state = ready_consensus;
          lower_L_bound = occupancy == true ? lower_L_occupied : lower_L_unoccupied;
          consensus.Init(lower_L_bound, my_offset, my_gains_vect, my_cost, my_address, number_of_addresses, nodes_addresses);
          LOOP = false;
          SIMULATOR = false;
          msg_to_send = start_consensus;
          writeMsg(0, msg_to_send, my_address);
        }
        Serial.write("+o");
        Serial.write(welcome[1]);
        Serial.write(welcome[2]);
        Serial.write(welcome[3]);
      } else {
          msg_to_send = hub_set_occupancy;
          writeMsgWithFloat(addr_to_send, msg_to_send, my_address, new_bound);
      }
  } else if((char)welcome[0] == 'O') {
      if(addr_to_send == my_address) {
        if( (occupancy == true) and (lower_L_bound != new_bound) ) { //If my selected occupancy is unoccupied and the received is different from min, then a new consensus is needed
          prev_state = my_state;
          my_state = ready_consensus;
          lower_L_occupied = new_bound;
          lower_L_bound = occupancy == true ? lower_L_occupied : lower_L_unoccupied;
          consensus.Init(lower_L_bound, my_offset, my_gains_vect, my_cost, my_address, number_of_addresses, nodes_addresses);
          LOOP = false;
          SIMULATOR = false;
          msg_to_send = start_consensus;
          writeMsg(0, msg_to_send, my_address);
        }
        Serial.write("+O");
        Serial.write(welcome[1]);
        Serial.write(welcome[2]);
        Serial.write(welcome[3]);
        lower_L_occupied = new_bound;
      } else {
          msg_to_send = hub_set_bound_occupied;
          writeMsgWithFloat(addr_to_send, msg_to_send, my_address, new_bound);
      }
  } else if((char)welcome[0] == 'U') {
      if(addr_to_send == my_address) {
        if( (occupancy == false) and (lower_L_bound != new_bound) ) { //If my selected occupancy is unoccupied and the received is different from min, then a new consensus is needed
          prev_state = my_state;
          my_state = ready_consensus;
          lower_L_unoccupied = new_bound;
          lower_L_bound = occupancy == true ? lower_L_occupied : lower_L_unoccupied;
          consensus.Init(lower_L_bound, my_offset, my_gains_vect, my_cost, my_address, number_of_addresses, nodes_addresses);
          LOOP = false;
          SIMULATOR = false;
          msg_to_send = start_consensus;
          writeMsg(0, msg_to_send, my_address);
        }
        Serial.write("+");
        Serial.write("U");
        Serial.write((char)addr_to_send);
        Serial.write(welcome[2]);
        Serial.write(welcome[3]);
        lower_L_unoccupied = new_bound;
      } else {
          msg_to_send = hub_set_bound_unoccupied;
          writeMsgWithFloat(addr_to_send, msg_to_send, my_address, new_bound);
      }
  } else if((char)welcome[0] == 'c') {
      if(addr_to_send == my_address) {
          Serial.write("+c");
          Serial.write(welcome[1]);
          Serial.write(welcome[2]);
          Serial.write(welcome[3]);
          prev_state = my_state;
          my_state = ready_consensus;
          my_cost = new_bound;
          lower_L_bound = occupancy == true ? lower_L_occupied : lower_L_unoccupied;
          consensus.Init(lower_L_bound, my_offset, my_gains_vect, my_cost, my_address, number_of_addresses, nodes_addresses);
          LOOP = false;
          SIMULATOR = false;
          msg_to_send = start_consensus;
          writeMsg(0, msg_to_send, my_address);
      } else {
          msg_to_send = hub_set_cost;
          writeMsgWithFloat(addr_to_send, msg_to_send, my_address, new_bound);
      }
  } else {
      Serial.write("+");
      Serial.write(welcome[1]);
      Serial.write(welcome[1]);
      Serial.write(welcome[2]);
      Serial.write(welcome[3]);
  } 
  return false;
}

/*
 * This function is used to represent a float number in 2 bytes.
 * 
 * The 12 most significatives bits represents the integer part with resolution 0:4095
 * The 4  less significatives bits represents the decimal part with resolution 0:9
 * 
 * Sends '*' when there is nothing to be said, which corresnponds to 10 in the 4 less significative bits
 */
 //if flag = true: The converted float is returned(for canbus msgs), if flag=false: writes to serial(for hub msgs)
byte* float_2_bytes(float fnum, bool flag)
{   
    // the maximum admissive value is 4095.94(9)
    fnum = fnum < pow(2,12)-0.05 ? fnum : pow(2,12)-1+0.9;  // 2^12-1 == 4095, 12 bits representation + 4bits to decimal representation(0.1 -> 0.9)
  
    uint16_t output = fnum < 0; // flag that represents if fnum is negative
    uint16_t inum = fnum;  // integer part
    fnum = fnum-inum; // floating part
    uint8_t dnum = round(10*fnum);  // integer decimal part

    // increments one when the decimal part rounds up
    if(dnum == 10){
        dnum = 0;
        inum++;
    }
    
    inum = inum<<4; // the integer will be represented in the first 1,5 bits
    output = output ? 15: inum + dnum; // send 15 when fnum is negative, which should not be possible

    if(flag) {
      byte* number = (byte*)malloc(2*sizeof(byte));
      number[0] = (byte) output;
      number[1] = (byte) (output>>8);
      return number;
    } else {
      Serial.write( (byte) (output>>8) ); // write second byte - DEBUG: Serial.println((byte) (output>>8))
      Serial.write( (byte) output );  // write first byte - DEBUG: Serial.println((byte) output);
    }
    return 0;
}

/*
 * Sends the first message to the server with the format: "A<_number_of_desks_>:)"
 */
void greeting(int numLamps)
{
    numLamps = numLamps < 0 ? 0 : numLamps > 255 ? 255 : numLamps;  // normalize number of desks.
    Serial.write("+");
    Serial.write("A");
    Serial.write(numLamps); // number of arduinos
    Serial.write(":)");
}

// the time spent occupates 2.5 bytes with integer number and 4bit with float
void send_time()
{   
    double time_ = millis() * 1e-3;
    Serial.write("+");
    Serial.write('t');
    Serial.write( ( (unsigned long)round(time_) & 0xFF000)  >> 12 );
    float_2_bytes( time_ - ((unsigned long)round(time_) & 0xFF000) , false );
}

float bytes2float(byte * myBytes){
  float welit = 0;
  byte mask_decimal = B00001111;
  byte mask_int = B11110000;
  // get decimal part of the number
  byte decimal = myBytes[1] & mask_decimal;
  int decimal_int = decimal;

  // get decimal part of the number
  byte split_int = (myBytes[1] & mask_int) >> 4;
  int integer = (myBytes[0] << 4) + split_int;

  welit = integer + decimal_int/10.0;
  
  return welit;
}


//Same conversion function, but for consensus we need 2 decimal cases, max number is 511.99, 9 bits for int and 7 for floating point
byte* float_2_bytes_2decimals(float fnum)
{   
    // the maximum admissive value is 511.99
    fnum = fnum < pow(2,9)-0.005 ? fnum : pow(2,9)-1+0.99;  // 2^12-1 == 4095, 12 bits representation + 4bits to decimal representation(0.1 -> 0.9)
  
    uint16_t output = fnum < 0; // flag that represents if fnum is negative
    uint16_t inum = fnum;  // integer part
    fnum = fnum-inum; // floating part
    uint8_t dnum = round(100*fnum);  // integer decimal part

    // increments one when the decimal part rounds up
    if(dnum == 100){
        dnum = 0;
        inum++;
    }
    
    inum = inum<<7; // the float will be represented in the first 1,8 bits
    output = output ? 127: inum + dnum; // send 127(1111111) when fnum is negative, which should not be possible

    byte* number = (byte*)malloc(2*sizeof(byte));
    number[0] = (byte) output;
    number[1] = (byte) (output>>8);
    return number;
}

float bytes_2_float_2decimals(byte * myBytes){
  float welit = 0;
  byte mask_decimal = B01111111;
  byte mask_int = B10000000;
  // get decimal part of the number
  byte decimal = myBytes[1] & mask_decimal;
  int decimal_int = decimal;

  // get integer part of the number
  byte split_int = (myBytes[1] & mask_int) >> 7;
  int integer = (myBytes[0] << 1) + split_int;

  welit = integer + decimal_int/100.0;
  
  return welit;
}

//Return -1 if askedLux < offset
//Return 1 if askedLux > max
//Return 0 if no problems
byte checkBoundsOnAskedLux(float askedLux) {
  if(askedLux < my_offset) {
    return -1;
  }
  float maxLux = my_offset;
  for(byte i=0; i<number_of_addresses-1; i++) {
    maxLux += 255*my_gains_vect[i];
  }
  if(askedLux > maxLux) {
    return 1;
  }
  return 0;
}
