/*
 * ConfigServer.hpp
 *
 *  Created on: Aug 19, 2013
 *      Author: pascal
 */

#ifndef CONFIGSERVER_HPP_
#define CONFIGSERVER_HPP_

#include <vector>
#include <boost/asio.hpp>
#include <boost/scoped_ptr.hpp>

#include "sensors/Sensor.hpp"
#include "calibration/CalibrationStorage.hpp"

class ConfigServer {
 public:
  ConfigServer(boost::asio::io_service& io_service, unsigned short int port, Sensor::Map& sensors, CalibrationStorage* calib_provider);
  virtual ~ConfigServer();

  void run();
  bool getHostInitialized(){return host_initialized_;};
  bool hasConnection(void) { return connected_; };

 private:
  /*network functions*/
  template<class StreambufType> bool writeSocket(StreambufType buffer);
  template<class StreambufType> bool readSocket(StreambufType buffer);
  void clientDisconnected();
  void acceptConnection();
  bool checkDisconnect(const boost::system::error_code &error);

  /*protocol function*/
  void syncTime();
  void sendFpgaInfo();
  void sendSensorInfos();
  void readUbiRegister();
  void writeUbiRegister();
  void readFpgaRegister();
  void writeFpgaRegister();

  void sendCameraCalibration(unsigned int cam_id, unsigned int slot_id);
  void receiveCameraCalibration(void);

  void sendAck(void);
  void sendNack(void);

 private:
  boost::asio::io_service& io_service_;
  boost::scoped_ptr<boost::asio::ip::tcp::socket> config_socket_;
  Sensor::Map& sensors_;

  unsigned short int port_;
  bool host_initialized_;
  bool connected_;

  CalibrationStorage *calib_provider_;
};

#endif /* CONFIGSERVER_HPP_ */
