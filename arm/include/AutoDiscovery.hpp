/*
 * AutoDiscovery.hpp
 *
 *  Created on: Aug 19, 2013
 *      Author: pascal
 */

#ifndef AutoDiscovery_HPP_
#define AutoDiscovery_HPP_

#include <boost/asio.hpp>

class AutoDiscovery {
 public:
  AutoDiscovery(boost::asio::io_service& io_service, unsigned short int port);
  virtual ~AutoDiscovery();

  void startService();

 private:
  void handle_receive(const boost::system::error_code& error, size_t bytes_recvd);

 private:
  boost::asio::io_service& io_service_;
  boost::asio::ip::udp::socket socket_;
  unsigned short int port_;
  char* data_;
  boost::asio::ip::udp::endpoint sender_;
};

#endif /* AutoDiscovery_HPP_ */
