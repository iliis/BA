
#ifndef TCP_SERVER_HPP_
#define TCP_SERVER_HPP_

#include <boost/asio.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "ip_data_definitions.hpp"

class TcpServer {
public:

	TcpServer(boost::asio::io_service& io_service, unsigned short int port);
	virtual ~TcpServer();
	void sendNetworkData(volatile char* data, IpComm::Header& header);

	bool hasConnection(){
		return connected_;
	}

protected:
	void acceptConnection();
	void handleClientConnection(const boost::system::error_code& error);
	void clientDisconnected();

protected:
	boost::asio::io_service& io_service_;
	boost::scoped_ptr<boost::asio::ip::tcp::socket> socket_;
	boost::shared_mutex m_socket_shutdown_;

	boost::asio::ip::tcp::acceptor acceptor_;
	volatile bool connected_;
	int port_;

private:
	void readNetworkHandler(const boost::system::error_code& error, std::size_t bytes_transferred);
	boost::array<uint32_t, 3> read_buffer_;
};

#endif /* TCP_SERVER_HPP_ */
