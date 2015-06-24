/*
 * SerialBridge.hpp
 *
 *  Created on: Feb 25, 2014
 *      Author: skybotix
 */

#ifndef SERIALBRIDGE_HPP_
#define SERIALBRIDGE_HPP_

#include <vector>
#include <string>

#include <boost/shared_ptr.hpp>

#include "serial_bridge/AsyncSerial.hpp"

//forward declartion
class SerialServer;

class SerialBridge {
  friend class SerialServer;

 public:
  struct SerialPortInfo;

  SerialBridge(const unsigned int network_port);
  virtual ~SerialBridge();

  bool setReadDelimiter(const unsigned char serial_id, std::string delimiter);
  bool changeBaudrate(const unsigned char serial_id, unsigned int baudrate);

 protected:
  void serialCallback(const unsigned char serial_id, const char *data, unsigned int len);
  void networkCallback(const unsigned char serial_id, std::string data);

 private:
  typedef std::vector<SerialPortInfo> PortList;
  typedef boost::scoped_ptr<PortList> PortListPtr;

  const unsigned int network_port_;
  const unsigned int default_baudrate_ = 115200;
  PortListPtr serial_ports_;

  bool initialize(void);
  void enumerateSerialPorts(void);

  void openPort(SerialPortInfo &ports);
  void closePort(SerialPortInfo &ports);

  static const unsigned int valid_baudrate_vals[];

  SerialServer *server_;
};


struct SerialBridge::SerialPortInfo
{
  static unsigned char id_provider;

  const unsigned char serial_id; //serial id
  std::string path; //file descriptor
  unsigned int baudrate;     //baudrate
  std::string delimiter;

  CallbackAsyncSerial::Ptr handle;  //port connection
  bool open;        //status

  SerialPortInfo(std::string path, unsigned int baudrate) : serial_id(id_provider++), path(path), baudrate(baudrate), delimiter(std::string("")), handle(NULL), open(false) {};
};



#endif /* SERIALBRIDGE_HPP_ */

