#include <iostream>
#include <string>
#include <ctime>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread/mutex.hpp>

#include "tcp_server.hpp"

using boost::asio::ip::tcp;

TcpServer::TcpServer(boost::asio::io_service& io_service, unsigned short int port)
: io_service_(io_service),
  acceptor_(io_service, tcp::endpoint(tcp::v4(), port)),
  connected_(false),
  port_(port)
{
	//std::cout << "max_iov_len=" << boost::asio::detail::max_iov_len << std::endl;

  //async. wait for connection
	acceptConnection();
}

TcpServer::~TcpServer()
{
  connected_ = false;
}

void TcpServer::acceptConnection() {
	//create new socket
  socket_.reset(new boost::asio::ip::tcp::socket(io_service_));

  //setup connection acceptor
	acceptor_.set_option(tcp::no_delay(true));
	acceptor_.async_accept(*socket_,	boost::bind(&TcpServer::handleClientConnection, this, boost::asio::placeholders::error));

	printf("TcpServer (%u): accepting connection on port %u\n", port_, port_);
}

void TcpServer::handleClientConnection(const boost::system::error_code& error) {
	// On error, return early.
	if (error) {
		std::cout << "TcpServer (" << port_ << "): ERROR: client connection error: " << error.message() << std::endl;
		return;
	}

	//ready to send data
	printf("TcpServer (%u): client connected from %s\n", port_, socket_->remote_endpoint().address().to_string().c_str());
	connected_ = true;

  //setup async reader to detect client disconnects fast (not really used to read data...)
  async_read(*socket_, boost::asio::buffer(read_buffer_), boost::asio::transfer_all(),
             boost::bind(&TcpServer::readNetworkHandler, this, boost::asio::placeholders::error,
                         boost::asio::placeholders::bytes_transferred));
}

void TcpServer::clientDisconnected(void)
{
  //cleanup the socket
  try{
    connected_ = false;

    //protect socket against destruction while txfer active (get exclusive access)
    boost::unique_lock< boost::shared_mutex > lock(m_socket_shutdown_);

    //let all pending write operations finish before we close the socket
    boost::system::error_code err;
    socket_->shutdown(boost::asio::ip::tcp::socket::shutdown_send, err);

    socket_.reset();
  } catch (const std::exception &ex) {
    printf("TcpServer::clientDisconnected exception: %s\n", ex.what());
  }

  printf("TcpServer (%u): client disconnected!\n", port_);
}

void TcpServer::readNetworkHandler(const boost::system::error_code& error, std::size_t bytes_transferred)
{
  //check for client disconnects
  if (error == boost::asio::error::eof ||
      error == boost::asio::error::connection_reset ||
      error == boost::asio::error::operation_aborted ||
      error == boost::asio::error::connection_aborted ||
      error == boost::asio::error::interrupted)
  {
    clientDisconnected();
    acceptConnection();
  } else {
    printf("TcpServer (%u): unhandled error! (%s)\n", port_, error.message().c_str());
  }
}

void TcpServer::sendNetworkData(volatile char * data, IpComm::Header& header) {

	//only send data if a connection is established
	if( !hasConnection() )
		return;

	std::vector<boost::asio::const_buffer> buffers;
	buffers.push_back(boost::asio::buffer(header.getSerialized()));
	if(data != 0 && header.data_size > 0)
	  buffers.push_back(boost::asio::buffer((char*)data, header.data_size));

  //protect socket against destruction in other threads
  boost::shared_lock<boost::shared_mutex> lock(m_socket_shutdown_);

  //send data
  boost::system::error_code error;
  unsigned int bytes_sent = boost::asio::write(*socket_, buffers, error);

  //check for client disconnects
  if (error == boost::asio::error::eof ||
      error == boost::asio::error::connection_reset ||
      error == boost::asio::error::operation_aborted ||
      error == boost::asio::error::connection_aborted ||
      error == boost::asio::error::interrupted)
  {
    //the async read handler handles the client disconnect
    return;
  }

  //check tx bytes
  if(bytes_sent != sizeof(IpComm::HeaderPayload) + header.data_size)
    printf("TcpServer (%u): not correct number of bytes sent: %d != %d\n", port_, bytes_sent, sizeof(IpComm::HeaderPayload) + header.data_size);
}

