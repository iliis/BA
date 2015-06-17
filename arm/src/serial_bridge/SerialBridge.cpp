/*
 * SerialBridge.cpp
 *
 *  Created on: Feb 25, 2014
 *      Author: schneith
 */

#include <vector>
#include <string>
#include <iostream>
#include <algorithm>
#include <iterator>

#include <boost/bind.hpp>
#include <boost/tokenizer.hpp>

#include "serial_bridge/SerialServer.hpp"
#include "serial_bridge/SerialBridge.hpp"


unsigned char SerialBridge::SerialPortInfo::id_provider = 0;
const unsigned int SerialBridge::valid_baudrate_vals[] =
  {110, 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 56000,
    57600, 115200, 128000, 153600, 230400, 256000, 460800, 921600 };


SerialBridge::SerialBridge(const unsigned int network_port)
  : network_port_(network_port),
    serial_ports_(new PortList),
    server_(NULL)
{
  //initialize the the serial bridge
  initialize();
}

SerialBridge::~SerialBridge() {
  //close all serial ports
  for (auto& port : *serial_ports_)
      closePort(port);
}

bool SerialBridge::initialize(void)
{
  //obtain port list
  enumerateSerialPorts();

  //start server
  server_ = new SerialServer(network_port_, this);
  if(serial_ports_->size() == 0)
  {
    printf("SerialBridge: no ports found, shutting down server...\n");
  } else {
    printf("SerialBridge: %u ports initialized\n", serial_ports_->size());
  }

  //open each port with a standard baudrate of 115200
  for (auto& port : *serial_ports_)
      openPort(port);

  //set network callback
  server_->setCallback( boost::bind( &SerialBridge::networkCallback, this, _1, _2 ) );

  return true;
}

void SerialBridge::serialCallback(const unsigned char serial_id, const char *data, unsigned int len)
{
  //send data over network
  std::string payload(data, data+len);
  server_->sendSerialData(serial_id, payload);
}

//set a delimiter string. after the string has been received the data is sent to the callback
//(e.g. \r\n when using NMEA, --> callback is called for each message)
bool SerialBridge::setReadDelimiter(const unsigned char serial_id, std::string delimiter)
{
  //send data to serial port
  if(serial_id >= serial_ports_->size() )  //TODO(schneith): implement assert here...
  {
    printf("SerialBridge::setDelimiter: invalid port id\n");
    return false;
  }

  //limit size to 3 chars
  if(delimiter.length() > 3)
  {
    printf("SerialBridge::setDelimiter: failed, delimiter is too long\n");
    return false;
  }

  //set the delimiter
  (*serial_ports_)[serial_id].delimiter = delimiter;
  (*serial_ports_)[serial_id].handle->setDelimiter(delimiter);

  //debug msgs
  printf("SerialBridge:: setDelimiter on port %u to \"%s\"\n", serial_id, delimiter.c_str());

  return true;
}

bool SerialBridge::changeBaudrate(const unsigned char serial_id, unsigned int baudrate)
{
  //send data to serial port
  if(serial_id >= serial_ports_->size() )  //TODO(schneith): implement assert here...
  {
    printf("SerialBridge::changeBaudrate: invalid port id\n");
    return false;
  }

  //check for valid baudrates
  bool valid_baudrate = std::find(std::begin(valid_baudrate_vals), std::end(valid_baudrate_vals), baudrate) != std::end(valid_baudrate_vals);

  if(!valid_baudrate)
  {
    printf("SerialBridge::changeBaudrate on port %u to %u, invalid baudrate\n", serial_id, baudrate);
    return false;
  }

  //change baudrate
  closePort((*serial_ports_)[serial_id]);
  (*serial_ports_)[serial_id].baudrate = baudrate;
  openPort((*serial_ports_)[serial_id]);

  printf("SerialBridge::changeBaudrate on port %u to %u --> success: true\n", serial_id, baudrate);

  return true;
}

void SerialBridge::networkCallback(const unsigned char serial_id, std::string data)
{
  //send data to serial port
  if(serial_id >= serial_ports_->size() )  //TODO(schneith): implement assert here...
  {
    printf("SerialBridge::networkCallback: invalid port id");
    return;
  }

  //send the data to the serial port (if conneceted)
  if( (*serial_ports_)[serial_id].open )
    (*serial_ports_)[serial_id].handle->writeString( data );
}

void SerialBridge::openPort(SerialPortInfo &port)
{
  try {
    port.handle.reset( new CallbackAsyncSerial(port.path.c_str(), port.baudrate) );
    port.handle->setCallback( boost::bind( &SerialBridge::serialCallback, this, port.serial_id, _1, _2 ) );
    port.handle->setDelimiter(port.delimiter);
  } catch (std::exception& e) {
    std::cout << "port: " << port.path.c_str() << "\n";
    std::cerr << "Exception: " << e.what() << std::endl;
  }

  port.open = true;
}

void SerialBridge::closePort(SerialPortInfo &port)
{
  port.handle->setCallback( NULL );
  port.handle->close();
  port.handle.reset();
}

void SerialBridge::enumerateSerialPorts(void)
{
  //clear list
  serial_ports_->clear();

  //list /dev to find serial ports
  const std::string cmd("ls -1 /dev | grep 'ttyUSB\\|ttyACM'");
  FILE* pipe = popen(cmd.c_str(), "r");
  if (!pipe) return;
  char buffer[4096];
  std::string result = "";
  while(!feof(pipe)) {
    if(fgets(buffer, 128, pipe) != NULL)
      result += buffer;
  }
  pclose(pipe);

  //parse available usb serial ports
  if(result.length() > 0)
  {
    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
    boost::char_separator<char> sep("\n");
    tokenizer tokens(result, sep);

    for (tokenizer::iterator tok_iter = tokens.begin(); tok_iter != tokens.end(); ++tok_iter)
    {
      std::string port_name = "/dev/" + *tok_iter;
      printf("SerialBridge: adding serial port %s\n", port_name.c_str() );
      serial_ports_->push_back( SerialPortInfo(port_name, default_baudrate_) );
    }

  } else
  {
    printf("SerialBridge: no serial ports found !\n");
  }
}


