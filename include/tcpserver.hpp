#pragma once

#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <fstream>
#include <types.hpp>

using namespace boost::asio;
using ip::tcp;
using std::cout;
using std::endl;
using std::string;

class TCPServer
{
public:
    TCPServer(int port = 8888);
    ~TCPServer();
    void set_port(int port);
    void run();
    string get_data() const;
    Odometry get_odometry();
    void send_data(const string &data);

private:
    int _port;
    boost::asio::io_context _io_context;
    tcp::acceptor _acceptor;
    tcp::socket _socket;
    std::string _data;
    boost::system::error_code _ec;
};