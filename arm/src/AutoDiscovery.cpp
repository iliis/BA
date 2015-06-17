/*
 * AutoDiscovery.cpp
 *
 *  Created on: Aug 19, 2013
 *      Author: pascal
 */

#include <boost/bind.hpp>
#include <boost/foreach.hpp>

#include "AutoDiscovery.hpp"

using boost::asio::ip::udp;

AutoDiscovery::AutoDiscovery(boost::asio::io_service& io_service, unsigned short int port)
: io_service_(io_service),
  socket_(io_service),
  port_(port){
  data_ = new char[12];
}

AutoDiscovery::~AutoDiscovery() {
  socket_.close();
  delete data_;
}

void AutoDiscovery::startService()
{
  printf("Auto discovery service running\n");

  boost::asio::ip::address ipAddr = boost::asio::ip::address_v4::any();
  boost::asio::ip::udp::endpoint listen_endpoint(ipAddr, port_);

  socket_.open(listen_endpoint.protocol());
  socket_.bind(listen_endpoint);

  socket_.async_receive_from(
      boost::asio::buffer(data_, 12), sender_,
      boost::bind(&AutoDiscovery::handle_receive, this,
        boost::asio::placeholders::error,
        boost::asio::placeholders::bytes_transferred));
}

void AutoDiscovery::handle_receive(const boost::system::error_code& error,
  size_t bytes_recvd)
{
std::cout << "receive" << bytes_recvd << data_ << std::endl;

std::string message= "Hello driver";
// find sensor ip address
boost::asio::ip::udp::socket test_socket(io_service_);
test_socket.connect(sender_);
test_socket.send(boost::asio::buffer(message));
boost::asio::ip::address addr = test_socket.local_endpoint().address();
test_socket.close();
std::cout << "local_endpoint: " << addr.to_string() << "sender_: " << sender_.address().to_string() << std::endl;

// answer with sensor ip
boost::system::error_code ec;
boost::asio::ip::udp::socket send_socket(io_service_);
send_socket.open(boost::asio::ip::udp::v4(), ec);

//boost::shared_ptr<std::string> message(new std::string(socket_.local_endpoint().address().to_string()));
boost::asio::ip::udp::endpoint send_endpoint(sender_.address(), 13775);
send_socket.send_to(boost::asio::buffer(message), send_endpoint);
send_socket.close();

//socket_.cancel();
// listen again
  socket_.async_receive_from(
      boost::asio::buffer(data_, 12), sender_,
      boost::bind(&AutoDiscovery::handle_receive, this,
        boost::asio::placeholders::error,
        boost::asio::placeholders::bytes_transferred));
}
