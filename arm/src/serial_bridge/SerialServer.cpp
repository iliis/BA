/*
 * SerialServer.cpp
 *
 *  Created on: Feb 25, 2014
 *      Author: schneith
 */

#include <vector>
#include <string>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/thread.hpp>

#include "ip_data_definitions.hpp"
#include "serial_bridge/SerialBridge.hpp"
#include "serial_bridge/SerialServer.hpp"

using boost::asio::ip::tcp;

SerialServer::SerialServer(unsigned short int network_port, SerialBridge *parent_bridge)
: port_(network_port),
  connected_(false),
  parent_bridge_(parent_bridge)
{
  //start the processing thread
  boost::thread wt(boost::bind(&SerialServer::run, this));
}

SerialServer::~SerialServer() {
}

template<class StreambufType>
bool SerialServer::readSocket(StreambufType buffer)
{
  if(!connected_)
    return false;

  //try to read
  boost::system::error_code error;
  boost::asio::read(*serial_socket_, buffer, error);

  if( checkDisconnect(error) )
  {
    clientDisconnected();
    return false;
  }

  return true;
}

template<class StreambufType>
bool SerialServer::writeSocket(StreambufType buffer)
{
  if(!connected_)
    return false;

  //try to read
  boost::system::error_code error;
  boost::asio::write(*serial_socket_, buffer, error);

  if( checkDisconnect(error) )
  {
    clientDisconnected();
    return false;
  }

  return true;
}

//true: disconnect, false: client connected
bool SerialServer::checkDisconnect(const boost::system::error_code &error)
{
  //check if client has disconnected
  if (error == boost::asio::error::eof || error == boost::asio::error::connection_reset)
    return true;
  else
    return false;
}

void SerialServer::clientDisconnected()
{
  //close all serial ports
  for (auto& port : *(parent_bridge_->serial_ports_))
      port.handle->setClientConnectionStatus(false);

  //clean up environment for new clients
  connected_=false;
  serial_socket_.reset();

  printf("Serial server: client disconnected!\n");
}

void SerialServer::acceptConnection()
{
  //create new socket
  serial_socket_.reset(new boost::asio::ip::tcp::socket(io_service_));

  //accept connections
  tcp::endpoint endpoint = tcp::endpoint(tcp::v4(), port_);
  tcp::acceptor acceptor(io_service_, endpoint);
  acceptor.set_option(tcp::no_delay(true));
  printf("Serial server: accepting connection on %u\n", port_);

  //set client read timeout (client disconnect timeout)
  struct timeval tv;
  tv.tv_sec  = 0;
  tv.tv_usec = 100000;  //100ms
  setsockopt(serial_socket_->native(), SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  // stop here until client connects.
  acceptor.accept(*serial_socket_);
  connected_ = true;
  printf("Serial server: new connection\n");

  //set state to client connected (read serial ports and send out)
  //close all serial ports
  for (auto& port : *(parent_bridge_->serial_ports_))
      port.handle->setClientConnectionStatus(true);
}

//receive from serial, send to network
void SerialServer::sendSerialData(unsigned char port_id, const std::string &serial_data)
{
  //send data over network
  IpComm::Header header;
  header.timestamp = 0;
  header.data_size = serial_data.size() + 1; //first byte is port_id (char)
  header.data_id = IpComm::SERIAL_DATA;

  writeSocket(boost::asio::buffer(header.getSerialized()));

  //write payload (sensor_id + serial_data)
  writeSocket(boost::asio::buffer(&port_id, 1));
  writeSocket(boost::asio::buffer(serial_data));
}

//receive from network, send to serial port
void SerialServer::receiveSerialData(unsigned char port_id, const std::string &serial_data)
{
  if(serial_callback_ == NULL) //TODO(schneith): use assert...
  {
    printf("assert: set function callback ...");
    exit(1);
  }

  //pass data to serialbridge by invoking the registered callback
  serial_callback_(port_id, serial_data);
}

void SerialServer::setCallback(const boost::function<void(unsigned char, std::string)>& callback) {
  serial_callback_ = callback;
}

void SerialServer::run()
{
  //process incoming data
  while(1){

    try
    {
      //wait for new client
      if(!connected_)
      {
        usleep(50);
        acceptConnection();
      }

      //read header
      IpComm::HeaderPayload header_payload;
      if( !readSocket(boost::asio::buffer(header_payload, sizeof(header_payload))) )
        continue;

      //interpret header
      IpComm::Header header(header_payload);
//      printf("received header: data_id: %d data_size:%d\n",header.data_id, header.data_size );
//      printf("header_payload: %x %x %x\n",ntohl(header_payload[0]), ntohl(header_payload[1]), ntohl(header_payload[2]));

      //extract variable length network body
      std::vector<char> body_payload;
      body_payload.resize(header.data_size);

      if( !readSocket(boost::asio::buffer(body_payload)) )
        continue;

      switch(header.data_id)
      {
        case (IpComm::SERIAL_DATA):
        {
          //byte 0: port_id (char)
          unsigned int serial_portid = body_payload[0];//first byte is the serial port id

          //byte 1-N: serial data (char)
          std::string serial_data(body_payload.begin()+1, body_payload.end());

          //handle the data
          receiveSerialData(serial_portid, serial_data);
          break;
        }

        case (IpComm::SERIAL_SET_BAUDRATE):
        {
          //byte 0: port_id (char)
          unsigned int serial_portid = body_payload[0];//first byte is the serial port id

          //byte 1-N: serial data (char)
          unsigned int baudrate = *((unsigned int*)(&body_payload.front() + 1)); //byte 1-4: baudrate (uint32)

          //handle the data
          parent_bridge_->changeBaudrate(serial_portid, baudrate);

          break;
        }
        case (IpComm::SERIAL_SET_DELIMITER):
        {
          //byte 0: port_id (char)
          unsigned int serial_portid = body_payload[0];//first byte is the serial port id

          //byte 1-N: serial data (char)
          std::string delimiter(body_payload.begin()+1, body_payload.end());

          //handle the data
          parent_bridge_->setReadDelimiter(serial_portid, delimiter);

          break;
        }
        default:
        {
          printf("Serial server: ERROR: Received unknown header type: %d \n",header.data_id);
          break;
        }
      }
    }
    catch (std::exception& e)
    {
      std::cerr << "Serial Server Error: " << e.what() << std::endl;
    }
  }
}

