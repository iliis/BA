/*
 * SerialServer.hpp
 *
 *  Created on: Feb 25, 2014
 *      Author: schneith
 */

#ifndef SERIALSERVER_HPP_
#define SERIALSERVER_HPP_

#include <string>

#include <boost/function.hpp>
#include <boost/asio.hpp>
#include <boost/scoped_ptr.hpp>

//forward declartion
class SerialBridge;

class SerialServer {
 public:
  SerialServer(unsigned short int network_port, SerialBridge *parent_bridge);
  virtual ~SerialServer();
  void run();

  void sendSerialData(unsigned char port_id, const std::string &serial_data);
  void setCallback(const boost::function<void(unsigned char, std::string)>& callback);

 private:
  //network functions
  template<class StreambufType> bool readSocket(StreambufType buffer);
  template<class StreambufType> bool writeSocket(StreambufType buffer);
  void clientDisconnected();
  void acceptConnection();
  bool checkDisconnect(const boost::system::error_code &error);

  //protocol functions
  void receiveSerialData(unsigned char port_id, const std::string &serial_data);

 private:
  boost::asio::io_service io_service_;
  const unsigned short int port_;

  boost::scoped_ptr<boost::asio::ip::tcp::socket> serial_socket_;
  bool connected_;
  boost::function<void(unsigned char, std::string)> serial_callback_;

  SerialBridge *parent_bridge_;
};

#endif /* SERIALSERVER_HPP_ */
