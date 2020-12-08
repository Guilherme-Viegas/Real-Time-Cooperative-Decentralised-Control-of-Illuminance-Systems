#include <SPI.h>
#include <mcp2515.h>
#include "can_buffer.cpp"
#include "message.h"
MCP2515 mcp2515(10);

volatile can_frame_stream cf_stream;

volatile bool interrupt = false;
volatile bool mcp2515_overflow = false;
volatile bool arduino_overflow = false;
int counter = 0;


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
  can_frame frame = broadcast_message();
  frame.can_id = id;
  frame.can_dlc = 4;
  my_can_msg msg;
  msg.value = val; //pack data
  for ( int i = 0; i < 4; i++ ){ //prepare can message
    frame.data[i] = msg.bytes[i];
  }
  //send data
  return mcp2515.sendMessage(&frame);
}



void setup() {
  Serial.begin(2000000);
  SPI.begin();
  attachInterrupt(0, irqHandler, FALLING);
  //Must tell SPI lib that ISR for interrupt vector zero will be using SPI
  SPI.usingInterrupt(0);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_16MHZ);
  mcp2515.setNormalMode();
  //mcp2515.setLoopbackMode(); //for local testing
}

void loop() {
  //send a few msgs in a burst
  can_frame test = broadcast_message();
  code2Ascii(test);
  Serial.println("TESTEID: " + String(test.can_id, HEX));
  
  mcp2515.sendMessage(&broadcast_message());
  /*for ( int i = 0; i < 4 ; i++ ) {
    Serial.print( "Sending: " );
    Serial.println( counter );
    if (  mcp2515.sendMessage(&broadcast_message()) != MCP2515::ERROR_OK )
      Serial.println( "\t\t\t\tMCP2515 TX Buf Full" );
  }*/
  if ( interrupt ){
    interrupt = false;
    if ( mcp2515_overflow ){
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
      for( int i = 0 ; i < 4 ; i++ )
        msg.bytes[ i ] = frame.data[ i ];
      Serial.print( "\t\tReceiving: " ); Serial.println( msg.value );
      cli(); has_data = cf_stream.get( frame ); sei();
    }
  }
  while(true){}
  delay(1);
}
