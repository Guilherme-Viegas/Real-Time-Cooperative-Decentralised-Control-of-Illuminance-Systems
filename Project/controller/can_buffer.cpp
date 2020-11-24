#include "can_buffer.h"

inline int can_frame_stream::put( can_frame &frame ) {
  if ( write_lock )
    return 0; //buffer full
  cf_buffer[ write_index ] = frame;
  write_index = ( ++write_index ) % buffsize;
  if ( write_index == read_index)
    write_lock = true; //cannot write more
  return 1;
}

inline int can_frame_stream::get( can_frame &frame ) {
  if ( !write_lock && ( read_index == write_index ) )
    return 0; //empty buffer
  if ( write_lock && ( read_index == write_index ) )
    write_lock = false; //release lock
  frame = cf_buffer[ read_index ];
  read_index = ( ++read_index ) % buffsize;
  return 1;
}
