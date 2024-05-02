#include <tcpserver.hpp>

TCPServer::TCPServer(int port) : _port(port),
                                 _acceptor(_io_context, tcp::endpoint(tcp::v4(), port)),
                                 _socket(_io_context)
{
    std::cout << "Server started on port " << port << '\n';
}

TCPServer::~TCPServer()
{
    if (_socket.is_open())
        _socket.close();
}

void TCPServer::set_port(int port)
{
    if (_port != port)
    {
        if (_socket.is_open())
            _socket.close();
        _port = port;
        _acceptor = tcp::acceptor(_io_context, tcp::endpoint(tcp::v4(), _port));
    }
}

Odometry TCPServer::get_odometry()
{
    try
    {
        _socket = tcp::socket(_io_context);
        _acceptor.accept(_socket);

        char _temp[1024];
        size_t bytes_received = _socket.read_some(boost::asio::buffer(_temp), _ec);
        if (_ec)
            throw boost::system::system_error(_ec);

        _data = std::string(_temp, bytes_received);
        std::vector<double> double_array;
        std::stringstream ss(_data);
        std::string token;

        while (std::getline(ss, token, ','))
            double_array.push_back(std::stod(token));
        boost::asio::write(_socket, boost::asio::buffer("Ok\n"));

        return {Position(double_array[0], double_array[1], double_array[2]), Quaternion(double_array[3], double_array[4], double_array[5], double_array[6])};
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error in run(): " << e.what() << std::endl;
    }
    return {Position(0.0f, 0.0f, 0.0f), Quaternion(0.0f, 0.0f, 0.0f, 0.0f)};
}

void TCPServer::send_data(const std::string &data)
{
    try
    {
        if (!_socket.is_open())
        {
            _socket = tcp::socket(_io_context);
            _acceptor.accept(_socket);
        }
        boost::asio::write(_socket, boost::asio::buffer(data));
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error in send_data(): " << e.what() << std::endl;
    }
}
