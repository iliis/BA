/*
 * ClockSyncServer.cpp
 *
 *  Created on: Aug 19, 2013
 *      Author: pascal
 */

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>

#include "ClockSyncServer.hpp"
#include "ip_data_definitions.hpp"

using boost::asio::ip::tcp;

ClockSyncServer::ClockSyncServer(boost::asio::io_service& io_service, unsigned short int port, boost::shared_ptr<TimingBlock> timing_block)
: io_service_(io_service),
  socket_(io_service),
  timing_block_(timing_block),
  port_(port),
  connected_(false){
}

ClockSyncServer::~ClockSyncServer() {
}

template<class SocketType, class StreambufType>
bool ClockSyncServer::readSocket(SocketType &socket, StreambufType buffer)
{
  if(!connected_)
    return false;

  //try to read
  boost::system::error_code error;
  boost::asio::read(socket, buffer, error);

  if( checkDisconnect(error) )
  {
    clientDisconnected(socket);
    return false;
  }

  return true;
}

template<class SocketType, class StreambufType>
bool ClockSyncServer::writeSocket(SocketType &socket, StreambufType buffer)
{
  if(!connected_)
    return false;

  //try to read
  boost::system::error_code error;
  boost::asio::write(socket, buffer, error);

  if( checkDisconnect(error) )
  {
    clientDisconnected(socket);
    return false;
  }

  return true;
}

//true: disconnect, false: client connected
bool ClockSyncServer::checkDisconnect(const boost::system::error_code &error)
{
  //check if client has disconnected
  if (error == boost::asio::error::eof || error == boost::asio::error::connection_reset)
    return true;
  else
    return false;
}

template<class SocketType>
void ClockSyncServer::clientDisconnected(SocketType &socket)
{
  //clean up environment for new clients
  connected_=false;
  socket.close();
}

template<class SocketType>
void ClockSyncServer::acceptConnection(SocketType &socket)
{
  //accept connections
  tcp::endpoint endpoint = tcp::endpoint(tcp::v4(), port_);
  tcp::acceptor acceptor(io_service_, endpoint);
  acceptor.set_option(tcp::no_delay(true));
  printf("Clock Sync server: accepting connection on %u\n", port_);

  //set client read timeout (client disconnect timeout)
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 100000; //100ms
  setsockopt(socket.native(), SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  // stop here until client connects.
  acceptor.accept(socket);
  connected_ = true;
  printf("Clock Sync server: new connection\n");
}

void ClockSyncServer::run()
{
  IpComm::TimeSyncPayload inPackagePayload;
  IpComm::TimeSyncPayload outPackagePayload;

  //process incoming data
  while(1) {
    try
    {
      //wait for new client
      if(!connected_)
      {
        usleep(50);
        acceptConnection(socket_);
      }

      // read package
      readSocket(socket_, boost::asio::buffer(inPackagePayload));

      IpComm::TimeSync inPackage(inPackagePayload);

      // send answer back
      IpComm::TimeSync outPackage;
      outPackage.host_time = inPackage.host_time;
      outPackage.fpga_time = timing_block_->getTime();
      outPackagePayload = outPackage.getSerialized();
      writeSocket(socket_, boost::asio::buffer(outPackagePayload));

      timing_block_->signalSyncCompleted();

    } catch (std::exception& e) {
      std::cerr << "Clock Sync Server Error: " << e.what() << std::endl;
    }
  }
}
