#include "controller.h"
#include <SPI.h>
#include <mcp2515.h>
#include "can_buffer.cpp"
#include "comms.h"
#include "hub.hpp"
#include "consensus.hpp"



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
boolean DEBUG = true;

/*-------------------------------------|
 * GENERIC VARIABLES                   |
---------------------------------------|*/
float my_cost = 1;
float lower_bound_L = 20.0;


/*-------------------------------------|
 * STATE MACHINE OF THE PROGRAM        |
---------------------------------------|*/
enum state_machine {booting = 0, w8ing_olleh = 1, standard = 2, w8ing_ack = 3 , turn_off = 4, read_offset = 5, calibration = 6, w8ing_ldr_read = 7, ready_consensus=8};
state_machine my_state = booting;
state_machine prev_state = booting;

/*------------------------|
 * TYPE OF MESSAGES       |
--------------------------|*/
enum msg_types {hello=1, olleh=2, ack=3, turn_off_led=4, read_offset_value=5, read_gain=6, your_time_master=7, start_consensus=8};
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
bool transmiting = false;
double counter = 0;

/********************************
 * CONSENSUS VARIABLES
********************************/
Consensus consensus;

/*-----------------------------------------------------|
 * FUNCTIONS HEADERS                                     |
-------------------------------------------------------|*/
MCP2515::ERROR write(uint32_t id, uint32_t val);
void writeMsg(int id, byte msg_type, byte sender_address, byte dimming=-1, byte index=-1, float value=0);
void smallMsg(int id, byte msg_type, byte sender_address);
float getValueMsg(can_frame frame);
can_frame* readMsg(int * frames_arr_size);

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
void writeMsg(int id, byte msg_type, byte sender_address, byte dimming=-1, byte index=-1, float value = 0){
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
  } else if( (dimming != -1) and (index != -1) ) {
    //Write msg used for consensus
    unsigned long val = (unsigned long)msg_type;
    val += (unsigned long)my_address<<8;
    val += (unsigned long)dimming<<16;
    val += (unsigned long)index<<24;
    if ( write( id , val ) != MCP2515::ERROR_OK )
      Serial.println( "\t\t\t\tMCP2515 TX Buf Full" );
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

void readMsg(int *frames_size, can_frame frames[20]){
  int counter = 0;
  if ( mcp2515_overflow ) {
    Serial.println( "\t\t\t\tMCP2516 RX Buf Overflow" );
    mcp2515_overflow = false;
  }

  if( arduino_overflow ) {
    Serial.println( "\t\t\t\tArduino Buffers Overflow" );
    arduino_overflow = false;
  }
  
  bool has_data;
  can_frame frame;
  cli(); has_data = cf_stream.get( frame ); sei();
  while( has_data ) {
      counter++;
      frames[counter-1] = frame;
      Serial.println("REC: " + String(frames[counter-1].can_id) + " " + String(frames[counter-1].data[0]) + " " + String(frames[counter-1].data[1]));
      cli(); has_data = cf_stream.get( frame ); sei();
    }
  *frames_size = counter;
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
  
  pid.ldr.setGain( pid.getLedPin(), m, b );
  pid.ldr.t_tau_up.setParametersABC( 29.207246, -0.024485, 11.085226); // values computed in the python file
  pid.ldr.t_tau_down.setParametersABC( 15.402250,  -0.015674, 8.313158); // values computed in the python file

  if(DEBUG)
    Serial.println("Set up completed");
  
  //pid.setReferenceLux( 30 ); // sets the minimum value in the led ( zero instant )
  //if(pid.has_feedback()){initInterrupt1();}
  //initInterrupt1();
  nodes_addresses[0] = 0;
  nodes_addresses[1] = my_address;
  number_of_addresses = 2;  
  delay(8000);
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
          prev_state = my_state;
          my_state = ready_consensus;
          consensus.Init(lower_bound_L, my_offset, my_gains_vect, my_cost);
          msg_to_send = start_consensus;
          writeMsg(0, msg_to_send, my_address);
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
      msg_to_send = ack;
      writeMsg(new_msg.data[1], msg_to_send, my_address);
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
      consensus.Init(lower_bound_L, my_offset, my_gains_vect, my_cost);
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
      if( (millis() - waiting_time >= 1000) ) {
        prev_state = my_state;
        my_state = calibration;
        waiting_time = 0;
      }
    }break;
    case ready_consensus:{
      //TODO:
      //while(num_iterações < max_iter):
        //Calcula o seu vetor de dimmings propostos e guarda em dimmings[seu_index]
        //Enviar e Vai recebendo as propostas dos outros (can_id=0, msg_type=consensus_val, my_address, dimming=x, index) e guarda em dimmings[retrieveIndex(my_address)-1][retrieveIndex(index)-1] = x
        //Só qnd receber num_consensus_val == (number_of_addresses-2)*3 é que segue (e dá reset)
        //Calcula o novo vetor de avg
        //Calcula os novos multiplicadores de lagrange
        //incrementa num_iterações e segue para a próxima
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

  //if(Serial.available()){ hub(); } 

  if (LOOP){
    //pid.led.setBrightness( pid.getU() );

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
      counter++;
      // pwm
      Serial.write("+");
      Serial.write("s");
      Serial.write(1);
      // float_2_bytes( pid.ldr.luxToOutputVoltage( 5.0*analogRead( pid.getLdrPin() ) / 1023.0, true) );
      // float_2_bytes( 100.0*pid.getU()/255.0  );
      float_2_bytes(counter);
      float_2_bytes(counter);
    }
} 
