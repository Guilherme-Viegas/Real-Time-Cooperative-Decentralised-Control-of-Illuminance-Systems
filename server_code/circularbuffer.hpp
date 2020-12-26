#ifndef CIRCULAR_BUFFER_HPP
#define CIRCULAR_BUFFER_HPP

#include <iostream>

#define DEBUG 1

template <class T>  // template class in order to support any type of data, e.g. float, int, unit8_t
class circular_array
{

private:    // this things are private

    std::unique_ptr<T[]> t_array;
    const size_t t_array_size;

public:     // this things are public
    
    // https://stackoverflow.com/questions/21488744/how-to-defined-constructor-outside-of-template-class
    circular_array( size_t size ) : t_array_size( size )
    {   
        if(DEBUG) std::cout << "It was created a circular array with size: " <<  t_array_size << std::endl;
        t_array = std::unique_ptr<T[]>(new T[t_array_size]);
    }
    
    ~circular_array()
    {
        if(DEBUG) std::cout << "It was deleted a circular array with size: " <<  t_array_size << std::endl;
    };

};

#endif

// https://embeddedartistry.com/blog/2017/05/17/creating-a-circular-buffer-in-c-and-c/