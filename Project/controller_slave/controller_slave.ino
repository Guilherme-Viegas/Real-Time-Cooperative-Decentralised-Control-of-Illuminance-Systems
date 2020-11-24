#include "controller.h"
#include "can_buffer.cpp"

MCP2515 mcp2515(10);

// Global variables to be declared in both setup() and loop()

// INIT PID
ControllerPid pid(3, A0);

// Global variables

boolean LOOP = false;
boolean SIMULATOR = true;


volatile can_frame_stream cf_stream;
volatile float gain_array[9];
volatile bool interrupt = false;
volatile bool mcp2515_overflow = false;
volatile bool arduino_overflow = false;
int counter = 0;
int id_arduino = 1;
union my_can_msg {
  unsigned long value;
  unsigned char bytes[4];
};

MCP2515::ERROR write(uint32_t id, uint32_t val, int id_transmissor) {
  can_frame frame;
  frame.can_id = id;
  frame.can_dlc = 4;
  my_can_msg msg;
  msg.value = val; //pack data
  frame.data[0] = msg.bytes[0];
  
  for ( int i = 1; i < 4; i++ ) //prepare can message
    frame.data[i] = msg.bytes[i];
  //send data
  return mcp2515.sendMessage(&frame);
}

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



void setup() {
  float tmp_m;
  float tmp_b;
  int eeprom_addr = 0;
  EEPROM.get(eeprom_addr, tmp_m);
  eeprom_addr += sizeof(float);
  EEPROM.get(eeprom_addr, tmp_b);
  
  Serial.begin(2000000);
  SPI.begin();
  attachInterrupt(0, irqHandler, FALLING);
  //Must tell SPI lib that ISR for interrupt vector zero will be using SPI
  SPI.usingInterrupt(0);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_16MHZ);
  mcp2515.setNormalMode();
  //mcp2515.setLoopbackMode(); //for local testing
  
  delay(500);
  pid.ldr.t_tau_up.setParametersABC( 29.207246, -0.024485, 11.085226); // values computed in the python file
  pid.ldr.t_tau_down.setParametersABC( 15.402250,  -0.015674, 8.313158); // values computed in the python file
  pid.ldr.setGain( pid.getLedPin(), tmp_m, tmp_b );
  
  //pid.led.setBrightness(100);
  //pidB.led.setBrightness(0);
  //pidC.led.setBrightness(0);

  delay(500);

  //send a few msgs in a burst
  /*for ( int i = 0; i < 4 ; i++ ) {
    Serial.print( "Sending: " );
    Serial.println( counter );
    if ( write( i , counter++ ) != MCP2515::ERROR_OK )
      Serial.println( "\t\t\t\tMCP2515 TX Buf Full" );
  }*/

  /*
  Serial.println("Arduino A");
  Serial.println(pid.ldr.luxToOutputVoltage(pid.ldr.getOutputVoltage(), true));
  Serial.println(pid.ldr.luxToOutputVoltage(pid.ldr.getOutputVoltage(), true));
  Serial.println(pid.ldr.luxToOutputVoltage(pid.ldr.getOutputVoltage(), true));

  Serial.println("Arduino B");
  Serial.println(pidB.ldr.luxToOutputVoltage(pidB.ldr.getOutputVoltage(), true));
  Serial.println(pidB.ldr.luxToOutputVoltage(pidB.ldr.getOutputVoltage(), true));
  Serial.println(pidB.ldr.luxToOutputVoltage(pidB.ldr.getOutputVoltage(), true));

  Serial.println("Arduino C");
  Serial.println(pidC.ldr.luxToOutputVoltage(pidC.ldr.getOutputVoltage(), true));
  Serial.println(pidC.ldr.luxToOutputVoltage(pidC.ldr.getOutputVoltage(), true));
  Serial.println(pidC.ldr.luxToOutputVoltage(pidC.ldr.getOutputVoltage(), true));
  */
  Serial.println("Set up completed");
  
  
  //pid.setReferenceLux( 30 ); // sets the minimum value in the led ( zero instant )
  //if(pid.has_feedback()){initInterrupt1();}
  
}


void loop() {

  if ( interrupt ) {
    interrupt = false;
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
      my_can_msg msg;
      if(int(frame.can_id) == id_arduino){
        for( int i = 0 ; i < 4 ; i++ )
          msg.bytes[ i ] = frame.data[ i ];
        Serial.print( "\t\tReceiving: " ); Serial.println( msg.value );
        if(float(msg.value) == 0){
          write(1, pid.ldr.luxToOutputVoltage(pid.ldr.getOutputVoltage(), true), id_arduino);
        }
        pid.led.setBrightness(float(msg.value));
      }
        
      cli(); has_data = cf_stream.get( frame ); sei();
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
