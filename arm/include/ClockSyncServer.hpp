/*
 * ClockSyncServer.hpp
 *
 *  Created on: Aug 19, 2013
 *      Author: pascal
 */

#ifndef CLOCK_SYNC_SERVER_HPP_
#define CLOCK_SYNC_SERVER_HPP_

#include <boost/asio.hpp>

#include "sensors/TimingBlock.hpp"
#include "calibration/CalibrationStorage.hpp"

class ClockSyncServer {
 public:
  ClockSyncServer(boost::asio::io_service& io_service, unsigned short int port, boost::shared_ptr<TimingBlock> timing_block);
  virtual ~ClockSyncServer();

  void run();

 private:
  /*network functions*/
  template<class SocketType, class StreambufType> bool writeSocket(SocketType &socket, StreambufType buffer);
  template<class SocketType, class StreambufType> bool readSocket(SocketType &socket, StreambufType buffer);
  template<class SocketType> void clientDisconnected(SocketType &socket);
  template<class SocketType> void acceptConnection(SocketType &socket);
  bool checkDisconnect(const boost::system::error_code &error);

 private:
  boost::asio::io_service& io_service_;
  boost::asio::ip::tcp::socket socket_;
  boost::shared_ptr<TimingBlock> timing_block_;
  unsigned short int port_;
  bool connected_;
};

#endif /* CLOCK_SYNC_SERVER_HPP_ */
