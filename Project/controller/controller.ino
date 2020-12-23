#include "controller.h"
#include <SPI.h>
#include <mcp2515.h>
#include "can_buffer.cpp"
#include "circular_buffer.h"
#include "comms.h"

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


/*-------------------------------------|
 * STATE MACHINE OF THE PROGRAM        |
---------------------------------------|*/
enum state_machine {booting=0, standard=1, w8ing_msg = 2, ready_for_calib=3, calibration=4};
state_machine my_state = booting;

//number of nodes in ready_for_calib state
int ready_number=0;

/*------------------------|
 * TYPE OF MESSAGES       |
--------------------------|*/
enum msg_types {hello=0, olleh=1, Ready=2, read_value=3, my_read = 4,turnOn_led=5, offset_finish=6};
msg_types msg_to_send;

/*-------------------------------------------
 * VARIABLES FOR THE CALIBRATION            |
*-------------------------------------------*/
enum calibration_machine {enter=0, turn_led=1, read_light = 2, w8ting_nodes=3, computing_offset = 4,compute_k = 5, calibrated = 6};
calibration_machine calib_machine = enter;

//number of offsets recieved
int n_offsets = 0;
//flag to know if the offset is done
bool offset_done = false;
float my_reading = 0.0;

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
  Serial.println("NEW MESSAGE:\n");
  Serial.println("ID: " + String(frame.can_id));
  for ( int i = 0; i < 4; i++ ){ //prepare can message
    frame.data[i] = msg.bytes[i];
    Serial.println(msg.bytes[i]);
  }
  //send data
  return mcp2515.sendMessage(&frame);
}
/*
 * Message with no values only msg_type and sender_address
*/
void writeMsg(int id, byte msg_type, byte sender_address, float value = 0){
  if(value){
    //write message with values
    byte* inBytes = (byte*)malloc(2*sizeof(byte));
    
    inBytes = float2bytes(value);
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

/*-----------------------------------------------|
  Function when the node is in booting state     |
-------------------------------------------------|*/
void booting_function(int msg_to_send){
  msg_to_send = hello;
  smallMsg(0, msg_to_send, my_address);
  Serial.println("Booting Up - Sended HELLO msg");
  my_state = w8ing_msg;
}
state_machine waiting_function(int msg_to_send, int starting_time, int *num_of_responses){
  if(micros() - starting_time >= 10000000){
    Serial.println("Entrei 1if");
    //Time has passed, I'm the only node on grid and I'm ready for calib
    nodes_addresses = (byte*)malloc(2*sizeof(byte)); //Because we will be address '1', cause '0' is for broadcast
    nodes_addresses[0] = 0;
    nodes_addresses[1] = my_address;
    number_of_addresses = 2;
    my_state = ready_for_calib;
    msg_to_send = Ready;
    smallMsg(0, msg_to_send, my_address);
  }
  //If I received response msg, then there are other nodes on the grid
  for(int i=0; i < frames_arr_size; i++) {
    if( (frames[i].data[0] == olleh) and ( frames[i].can_id == my_address ) ) {
      Serial.println("RECEBI olleh");
      *num_of_responses += 1;
      // (2 + num_responses) because nodes_addresses[0] is for broadcast and nodes_addresses[1] is my addr
      nodes_addresses = (byte*)realloc(nodes_addresses, (2 + *num_of_responses)*sizeof(byte)); 
      //So 1st received response goes to index 2 cause 0 and 1 is for broadcast and my address
      nodes_addresses[*num_of_responses + 1] = frames[i].data[1];
    }
  }
  if(*num_of_responses > 0) {
    Serial.println("Entrei 2if");
    number_of_addresses = 2 + *num_of_responses;
    nodes_addresses[0] = 0;
    nodes_addresses[1] = my_address;
    bubbleSort(nodes_addresses, number_of_addresses);
    
    //send broadcast READY_CALIB
    msg_to_send = Ready;
    smallMsg(0, msg_to_send, my_address);
    my_state = ready_for_calib;
  }
  return my_state;
}

void setup() {
  Serial.begin(19200);

  int eeprom_addr = 0;
  EEPROM.get(eeprom_addr, my_address);
  eeprom_addr += sizeof(byte);
  EEPROM.get(eeprom_addr, m);
  eeprom_addr += sizeof(float);
  EEPROM.get(eeprom_addr, b);

  Serial.print(my_address);
  Serial.print(" - ");
  Serial.print(m);
  Serial.print(" - ");
  Serial.println(b);

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
  Serial.println("Set up completed");
  
  //pid.setReferenceLux( 30 ); // sets the minimum value in the led ( zero instant )
  //if(pid.has_feedback()){initInterrupt1();}
  
}
/*
 * FUNCTION to compute K such as luminance  = K*pwm + offset
*/
void initCalibration(float **offset){
  //array with the sorted addresses [0, 1, 2, 3, ...]
  
  bool master = false;
  bool know_position = false;
  int my_position = -1;
  
  //make a master
  if(my_address == nodes_addresses[1])
    master = true;
  Serial.println("NUMBER OF ADDRESS: " + String(number_of_addresses));
  switch(calib_machine){
    case enter:{
      calib_machine = computing_offset;
      //send the offset value in the box
      my_reading = pid.led.getBrightness();
      msg_to_send = my_read;
      Serial.println("Address " + String(my_address) + "READING: " + String(my_reading));
      writeMsg(0, msg_to_send, my_address, my_reading);
    }break;
    case computing_offset:{
      Serial.println("\n-----------COMPUTING OFFSET---------\n");
      if(master){
        Serial.println("IM MASTER");
        Serial.println("READING ON COMPUTING: " + String(my_reading));
        
        *offset[0] = my_reading;
        for(int i =0; i < frames_arr_size; i++){
          Serial.println("TIPO DE MENSAGEM RECEBIDO: " + String(frames[i].data[0]));
          if(frames[i].data[0] == my_read){
            Serial.println("ENTREI");
          
            //get position of address array
            for(int j=0; j < number_of_addresses; j++){
              if(frames[i].data[1] == nodes_addresses[j]){
                *offset[j-1] = getValueMsg(frames[i]);
                Serial.println("j: " + String(j) + "--" + String(*offset[j-1]));
                n_offsets++;
              }
            }
          }
        }
      }
      if(!master){
        Serial.println("I'M NOT MASTER");
        for(int i =0; i < frames_arr_size; i++){
          Serial.println("Message Type: " + String(frames[i].data[0]));
          if(frames[i].data[0] == offset_finish){
            Serial.println("OFFSET FINISH");
            calib_machine = calibrated;
            break;
          }
        }
      }
      if(n_offsets == number_of_addresses-2){
        calib_machine = calibrated;
        msg_to_send = offset_finish;
        writeMsg(0, msg_to_send, my_address);
      }
    }break;
    case calibrated:{
      if(master){
        Serial.println("OFFSET COMPUTED");
        for(int i=0; i < number_of_addresses-1; i++){
          Serial.print(*offset[i]);
          Serial.print("-");   
        }
        Serial.println();
      }
        offset_done=true; 
    }break;
    default:{
    }break;
  }
}
/*
 * FUNCTION to get the float value of CAN message
*/
float getValueMsg(can_frame frame){
  byte value[2];    
  value[0] = frame.data[2];
  value[1] = frame.data[3];
  Serial.println("3rd byte: " + String(value[0]) + "\t4th byte: " + String(value[1]));
  float x = bytes2float(value);
  Serial.println("VALOR Recebido: " + String(x));
  return x;
}


void loop() {
  frames_arr_size = 0;
  frames = readMsg(&frames_arr_size);
  float* offset;
  switch(my_state){
    case booting:{
      booting_function(msg_to_send);
    }break;
    case w8ing_msg:{
      int num_of_responses = 0;
      starting_time = micros();
      //my_state = waiting_function(msg_to_send, starting_time, &num_of_responses);
      if(micros() - starting_time >= 10000000){
        Serial.println("Entrei 1if");
        //Time has passed, I'm the only node on grid and I'm ready for calib
        nodes_addresses = (byte*)malloc(2*sizeof(byte)); //Because we will be address '1', cause '0' is for broadcast
        nodes_addresses[0] = 0;
        nodes_addresses[1] = my_address;
        number_of_addresses = 2;
        my_state = ready_for_calib;
        msg_to_send = Ready;
        smallMsg(0, msg_to_send, my_address);
      }
      //If I received response msg, then there are other nodes on the grid
      for(int i=0; i < frames_arr_size; i++) {
        if( (frames[i].data[0] == olleh) and ( frames[i].can_id == my_address ) ) {
          Serial.println("RECEBI olleh");
          num_of_responses += 1;
          // (2 + num_responses) because nodes_addresses[0] is for broadcast and nodes_addresses[1] is my addr
          nodes_addresses = (byte*)realloc(nodes_addresses, (2 + num_of_responses)*sizeof(byte)); 
          //So 1st received response goes to index 2 cause 0 and 1 is for broadcast and my address
          nodes_addresses[num_of_responses + 1] = frames[i].data[1];
        }
      }
      if(num_of_responses > 0) {
        Serial.println("Entrei 2if");
        number_of_addresses = 2 + num_of_responses;
        nodes_addresses[0] = 0;
        nodes_addresses[1] = my_address;
        bubbleSort(nodes_addresses, number_of_addresses);
        
        //send broadcast READY_CALIB
        msg_to_send = Ready;
        smallMsg(0, msg_to_send, my_address);
        my_state = ready_for_calib;
      }
      if(my_state==ready_for_calib){
        Serial.println("N Responses after w8: " + String(number_of_addresses));
        if(number_of_addresses>0){
          for(int i=0; i<number_of_addresses; i++) {
            Serial.print(nodes_addresses[i]);
            Serial.print(" ");
          }
          Serial.println();
        }
      }
    }break; 
    case ready_for_calib:{
      //send a Ready broadcast message
      for(int i=0; i < frames_arr_size; i++){
        if(frames[i].data[0] == Ready){
          ready_number+=1;
        }
      }
      if(ready_number==number_of_addresses-2){
        my_state=calibration;
        ready_number=0;
        offset = (float*)calloc((number_of_addresses-1), sizeof(float));
        Serial.println("OFFSET CALLOC: " + String(offset[0]));
        Serial.println("CALIBRATION BABY");
      }
    }break;
    case calibration:{
      while(true){}
      initCalibration(&offset);
      while(offset_done){}
    }break;
    default:{
      Serial.println("DEFAULT");
    }break;
  }
    
  /*Serial.print(my_state);
  Serial.print(" - ");
  Serial.print(my_address);
  Serial.println();*/
  

  //Check messages received from the can-bus  
  for(int i=0; i < frames_arr_size; i++) {
    if((frames[i].data[0] == hello)) { //Check if received msg is from a new entering node...
      bool already_on_addresses = false;
      for(int j=0; j < number_of_addresses; j++) {
        if(frames[i].data[1] == nodes_addresses[j]) {
          already_on_addresses = true;
        }
      }
      if(already_on_addresses == false) { //If this address have never been to my addresse's vector
        Serial.println("Recebi HELLO da Address: " + String(frames[i].data[1]));
        number_of_addresses += 1;
        nodes_addresses = (byte*)realloc(nodes_addresses, number_of_addresses*sizeof(byte));
        nodes_addresses[number_of_addresses-1] = frames[i].data[1];
        bubbleSort(nodes_addresses, number_of_addresses);
      }
      delay(1);
      Serial.println("Quando mando olleh N addresses " + String(number_of_addresses));
      Serial.println("PARA ARDUINO: " + String(frames[i].data[1]));
      msg_to_send = olleh;
      smallMsg(frames[i].data[1], msg_to_send, my_address);
    }
  }
  

//Check messages received from the Serial bus
//Doing it in a while() for now, but surely not the best solution because it stops the code on the while
//I'm doint it cause the serial messages are really small (4bytes)
  /*while(Serial.available() > 0) { //If received message from RPI via Serial...
    byte inByte = Serial.read();
    serial_buff_size++;
    serial_buffer = (byte*)realloc(serial_buffer, sizeof(byte)*serial_buff_size);
    serial_buffer[serial_buff_size-1] = inByte;
  }
  if(serial_buff_size > 0) {
    if(serial_buff_size == 3 and char(serial_buffer[0]) == 'R' and char(serial_buffer[1]) == 'P' and char(serial_buffer[2]) == 'i') {
      //Then I received the msg from Rpi saying I'm the hub...so I send an ACK msg
      Serial.write("ACK");
      im_hub = true;
    }

    serial_buff_size = 0;
    serial_buffer = (byte*)realloc(serial_buffer, sizeof(byte)*serial_buff_size);
  }*/



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
