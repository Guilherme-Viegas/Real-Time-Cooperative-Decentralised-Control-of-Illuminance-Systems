#include "controller.h"
#include <SPI.h>
#include <mcp2515.h>
#include "can_buffer.cpp"


MCP2515 mcp2515(10);
// INIT PID
ControllerPid pid(3, A0);

// Global variables

boolean LOOP = false;
boolean SIMULATOR = false;

volatile can_frame_stream cf_stream;

volatile bool interrupt = false;
volatile bool mcp2515_overflow = false;
volatile bool arduino_overflow = false;
int counter = 0;

enum state_machine {booting=0, standard=1, w8ing_msg = 2, ready_for_calib=3};
state_machine my_state = booting;

byte *nodes_addresses;  //array of variable size depending on number of nodes
int number_of_addresses = 0;

int frames_arr_size = 0;
can_frame *frames;

byte message_types[] = {7,9};

float m;
float b;
byte my_address;

long starting_time = 0;

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

MCP2515::ERROR write(uint32_t id, uint32_t val) {
  can_frame frame;
  frame.can_id = id;
  frame.can_dlc = 4;
  my_can_msg msg;
  msg.value = val; //pack data
  for ( int i = 0; i < 4; i++ ) //prepare can message
    frame.data[i] = msg.bytes[i];
  //send data
  return mcp2515.sendMessage(&frame);
}

void writeMsg(int id, byte msg_type, byte sender_address) {
  int val = (sender_address<<8)+msg_type;  //Convert 2 bytes into 1 int, with byte 0 being msg_type and byte 1 being sender addr
  if ( write( id , val ) != MCP2515::ERROR_OK )
      Serial.println( "\t\t\t\tMCP2515 TX Buf Full" );
}



can_frame* readMsg(int *frames_arr_size) {
  if ( interrupt ) {
    interrupt = false;
    can_frame *frames;
    int counter = 0;
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
    while( has_data ) {
        counter++;
        frames = (can_frame*)realloc(frames,sizeof(can_frame)*counter);
        frames[counter-1] = frame;
        cli(); has_data = cf_stream.get( frame ); sei();
      }
    *frames_arr_size = counter;
    return frames;
  }
}


void setup() {
  Serial.begin(2000000);

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
  writeMsg(0, message_types[0], my_address);
  Serial.println("Set up completed");
  
  
  //pid.setReferenceLux( 30 ); // sets the minimum value in the led ( zero instant )
  //if(pid.has_feedback()){initInterrupt1();}
  
}


void loop() {
  frames_arr_size = 0;
  frames = readMsg(&frames_arr_size);
  /*Steps for connecting to grid:
   * Sends 'hello' broadcast msg with eeprom address to canbus and enters state '2'
   * "waits" 1s for incoming msgs with the other's addresses
   * if(not receivesMsg): enters in state '3' (ready for calibration)
   * else if(receivesMsg):
   * - Saves multiple addresses that received in msgs on nodes_addresses vector
   * - Enters state '3' (ready for calibration)
   */
   //If already on system
  //If receives 'hello' msg via broadcast it sends it's address to that new node address received
  //Saves the new address on vector (has to realloc it)

  switch(my_state) {
    case booting:
      writeMsg(0, message_types[0], my_address);
      Serial.println("Booting Up - Sended HELLO msg");
      starting_time = micros();
      my_state = w8ing_msg;
    case w8ing_msg:
      if(micros() - starting_time >= 1000000) {
        //Time has passed, I'm the only node on grid and I'm ready for calib
        nodes_addresses = (byte*)malloc(2*sizeof(byte)); //Because we will be address '1', cause '0' is for broadcast
        nodes_addresses[0] = 0;
        nodes_addresses[1] = my_address;
        number_of_addresses = 2;
        my_state = ready_for_calib;
      }
      //If I received response msg, then there are other nodes on the grid
      int num_of_responses = 0;
      for(int i=0; i<frames_arr_size; i++) {
        if( (frames[i].data[0] == message_types[1]) and ( frames[i].can_id == my_address ) ) { //TODO: Explicitly read only responses directed to my address
          num_of_responses += 1;
          nodes_addresses = (byte*)realloc(nodes_addresses, (2 + num_of_responses)*sizeof(byte)); // (2 + num_responses) because nodes_addresses[0] is for broadcast and nodes_addresses[1] is my addr
          nodes_addresses[num_of_responses + 1] = frames[i].data[1]; //So 1st received response goes to index 2 cause 0 and 1 is for broadcast and my address
        }
      }
      if(num_of_responses > 0) {
        number_of_addresses = 2 + num_of_responses;
        nodes_addresses[0] = 0;
        nodes_addresses[1] = my_address;
        my_state = ready_for_calib;
      }
    case ready_for_calib:
      break;
    default:
      break;
  }

  Serial.print(my_state);
  Serial.print(" - ");
  Serial.print(my_address);
  Serial.print(" - ");
  Serial.print(frames_arr_size);
  Serial.print(" - ");
  Serial.println(number_of_addresses);
  for(int i=0; i<number_of_addresses; i++) {
    Serial.print(nodes_addresses[i]);
    Serial.print(" ");
  }
  Serial.println();
  

  //Check if received msg es from a new entering node...
  for(int i=0; i<frames_arr_size; i++) {
    if((frames[i].data[0] == message_types[0])) {
      bool already_on_addresses = false;
      for(int j=0; j<number_of_addresses; j++) {
        if(frames[i].data[1] == nodes_addresses[j]) {
          already_on_addresses = true;
        }
      }
      if(already_on_addresses == false) { //If this address have never been to my addresse's vector
        number_of_addresses += 1;
        nodes_addresses = (byte*)realloc(nodes_addresses, number_of_addresses*sizeof(byte));
        nodes_addresses[number_of_addresses-1] = frames[i].data[1];
      }
      //delay(1);
      writeMsg(frames[i].data[1], message_types[1], my_address);
    }
  }
  

  if (LOOP){
  
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
  }                                 
}


// interrupt service routine 
ISR(TIMER1_COMPA_vect)        
{ 
  pid.computeFeedbackGain( analogRead( pid.getLdrPin() ) );
  // Serial.println( millis() );
} 
