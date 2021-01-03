#ifndef CIRCULAR_BUFFER_HPP
#define CIRCULAR_BUFFER_HPP

#include <iostream>
#include <mutex>
#include <unistd.h>
#include <memory>

#define DEBUG 1

template <class T> // template class in order to support any type of data, e.g. float, int, unit8_t
class circular_array
{

private: // this things are private
    std::unique_ptr<T[]> t_ring;
    const size_t t_array_size = 0;
    bool t_is_full = false;
    bool t_is_empty = true;
    size_t t_head = 0;
    size_t t_tail = 0;
    std::mutex t_mutex;

public: // this things are public
    // https://stackoverflow.com/questions/21488744/how-to-defined-constructor-outside-of-template-class
    circular_array(size_t size) : t_array_size(size)
    {
        if (DEBUG)
            std::cout << "It was created a circular array with size: " << t_array_size << std::endl;
        t_ring = std::unique_ptr<T[]>(new T[t_array_size]);
    }

    ~circular_array()
    {
        if (DEBUG)
            std::cout << "It was deleted a circular array with size: " << t_array_size << std::endl;
    };

    bool is_empty() const { return t_is_empty; }
    bool is_full() const { return t_is_full; }
    size_t get_size() const { return (t_is_full ? t_array_size : t_head + 1); };

    void insert_newest(T new_item)
    {
        std::lock_guard<std::mutex> lock(t_mutex);

        if (t_is_empty)
        {
            t_ring[t_head] = new_item;
            t_is_empty = false;
        }
        else
        {
            t_head = (t_head + 1) % t_array_size; // updates head value

            t_ring[t_head] = new_item; // adds new value

            if (t_head == t_tail && t_is_full)
            {
                t_tail = (t_tail + 1) % t_array_size;
            } // only updates tail if the array is full

            t_is_full = (t_head + 1 - t_tail) % t_array_size == 0; // checks if is now full
        }
    }

    T get_newest()
    {
        std::lock_guard<std::mutex> lock(t_mutex);

        if (is_empty())
        {
            return T();
        } // empty

        return t_ring[t_head];
    }

    std::unique_ptr<T[]> get_all()
    {
        std::lock_guard<std::mutex> lock(t_mutex);

        std::unique_ptr<T[]> ring_out = std::unique_ptr<T[]>(new T[t_array_size]);

        for (int i = 0; i < (int)t_array_size; i++)
        {
            ring_out[i] = t_ring[(i + t_tail) % t_array_size];
        }

        return ring_out;
    }

    void clear_all()
    {
        std::lock_guard<std::mutex> lock(t_mutex);

        for (int i = 0; i < (int)t_array_size; i++)
        {
            t_ring[i] = 0;
        }
        t_array_size = 0;
        t_is_full = false;
        t_is_empty = true;
        t_head = 0;
        t_tail = 0;
    }
};

#endif

// https://embeddedartistry.com/blog/2017/05/17/creating-a-circular-buffer-in-c-and-c/
