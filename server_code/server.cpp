#include "serial.hpp"
#include "async_server.hpp"

#include <thread>

#define PORT 18700 // https://stackoverflow.com/questions/3855127/find-and-kill-process-locking-port-3000-on-mac

#define INIT_COMMAND "+RPiS"
#define TIME_TO_SHUT_DOWN 0
#define NUM_THREADS 2

// global variable to control whether the server runs or not
boost::asio::io_context io;

// times to close the program
void start_timer(boost::asio::steady_timer *timer)
{
    if (!TIME_TO_SHUT_DOWN)
    {
        return;
    } // disable
    timer->expires_after(boost::asio::chrono::seconds{TIME_TO_SHUT_DOWN});
    timer->async_wait([](const boost::system::error_code &t_ec) {
        io.stop();
    });
}

int main()
{
    // close server after x seconds
    boost::asio::steady_timer timer{io};
    start_timer(&timer);
    
    std::signal(SIGINT, [](int sig) {
        std::cout << "\t Querias batinhas com enguias ...\n";
        io.stop();
        });

    // init serial
    communications the_serial{&io};

    uint8_t num_lamps = the_serial.has_hub();
    if( num_lamps <= 0 )
    {
        std::cout << "Early exit with" << (int)num_lamps << " lamps" << std::endl;
    }
    office the_office{num_lamps};

    tcp_server server_tcp{&io, PORT, &the_office, &the_serial};
    udp_server server_udp{&io, PORT + 1, &the_office};

    the_serial.write_command(INIT_COMMAND);
    the_serial.read_until_asynchronous(&the_office, '+');

    // while (!stop_server)
    //threads_service(&io); //https://www.boost.org/doc/libs/1_66_0/doc/html/boost_asio/reference/io_service.html

    std::thread threads[NUM_THREADS];
    for (int i = 0; i < NUM_THREADS; i++)
    {
        threads[i] = std::thread{ [](){ io.run(); } };
    }

    for (int i = 0; i < NUM_THREADS; i++)
    {
        threads[i].join();
    }


    return 0;
}
