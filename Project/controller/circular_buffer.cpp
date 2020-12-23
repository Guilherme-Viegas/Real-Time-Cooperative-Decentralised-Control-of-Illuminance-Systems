#include "circular_buffer.h"

//puts the buffer back to empty state(head==tail && full_==false)
void circular_buffer::reset()
{
	head_ = tail_;
	full_ = false;
}

bool circular_buffer::empty()
{
	//if head and tail are equal, we are empty
	return (!full_ && (head_ == tail_));
}

bool circular_buffer::full()
{
	//If tail is ahead the head by 1, we are full
	return full_;
}

size_t circular_buffer::capacity()
{
	return max_size_;
}

size_t circular_buffer::size()
{
	size_t size = max_size_;

	if(!full_){
		if(head_ >= tail_){
			size = head_ - tail_;
		}
		else{
			size = max_size_ + head_ - tail_;
		}
	}
	return size;
}

void circular_buffer::put(can_frame item){
	buf_[head_] = item;

	if(full_){
		tail_ = (tail_ + 1) % max_size_;
	}
	head_ = (head_ + 1) % max_size_;
	full_ = head_ == tail_;
}

can_frame circular_buffer::get(){
	if(empty()){
		return can_frame();
	}
	//Read data and advance the tail (we now have a free space)
	can_frame val = buf_[tail_];
	full_ = false;
	tail_ = (tail_ + 1) % max_size_;

	return val;
}
