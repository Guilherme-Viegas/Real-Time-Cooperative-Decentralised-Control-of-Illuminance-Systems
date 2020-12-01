#include <SPI.h>
#include <mcp2515.h>

class can_frame_stream {
    //10 slots buffer - increase if needed
    static const int buffsize = 10;
    can_frame cf_buffer[ buffsize ];
    int read_index; //where to read next message
    int write_index; //where to write next message
    bool write_lock; //buffer full
  public:
    can_frame_stream() : read_index( 0 ) ,
      write_index( 0 ), write_lock( false ) {
    };
    int put( can_frame & );
    int get( can_frame & );
}; //create one object to use
