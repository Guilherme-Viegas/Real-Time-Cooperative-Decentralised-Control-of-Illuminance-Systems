#include "circularbuffer.hpp"


// template<class T>
// void circular_array<T>::insert_newest( T new_item )
// {
//     std::lock_guard<std::mutex> lock(t_mutex);

//     t_head = ++t_head % t_array_size;   // updates head value

// 	t_ring[t_head] = new_item;  // adds new value

//     t_is_full = t_head == t_tail;// checks if is now full

// 	if( t_is_full ){ t_tail = ++t_tail % t_array_size; 

//         sleep(10);
//         std::cout << "Báu cheio!! :)\n";
    
//     }    // only updates tail if the array is full
// }

// template<class T>
// T circular_array<T>::get_newest()
// {
// 	//std::lock_guard<std::mutex> lock(t_mutex);

// 	if( is_empty() ){ return T(); }// empty

// 	return T();//t_ring[t_head];
// }

// template<class T>
// T* circular_array<T>::get_all()
// {
// 	std::lock_guard<std::mutex> lock(t_mutex);

// 	if( !t_is_full && !is_empty()){ return T(); } // neither full nor empty

//     T* ring_out {t_array_size};
//     for(int i = 0; i < t_array_size; i++)
//     {
//         ring_out[i] = t_ring[ (i+t_tail)%t_array_size ];
//     }

// 	return ring_out;
// }