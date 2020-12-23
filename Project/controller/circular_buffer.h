#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER
#include <mcp2515.h>

class circular_buffer {
  private:
    //smart pointer
  	can_frame* buf_;
  	size_t head_ = 0;
  	size_t tail_ = 0;
  	const size_t max_size_;
  	bool full_ = 0;
  
  public:
      //constructor
  	explicit circular_buffer(size_t size) :
  		buf_((can_frame*)malloc(size*sizeof(can_frame))),
  		max_size_(size)
  	{ // empty 
  	}
  
  	void put(can_frame item);
  	can_frame get();
  	void reset();
  	bool empty();
  	bool full();
  	size_t capacity();
  	size_t size();
};

#endif
