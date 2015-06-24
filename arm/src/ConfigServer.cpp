/*
 * ConfigServer.cpp
 *
 *  Created on: Aug 19, 2013
 *      Author: pascal
 */

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/pointer_cast.hpp>

#include "ConfigServer.hpp"
#include "ip_data_definitions.hpp"
#include "version.hpp"
#include "visensor/visensor_constants.hpp"

#include "sensors/MT9V034.hpp"
#include "sensors/TimingBlock.hpp"


#define CONFIG_PACKAGE_LENGTH 10

using boost::asio::ip::tcp;

ConfigServer::ConfigServer(boost::asio::io_service& io_service, unsigned short int port, Sensor::Map& sensors, CalibrationStorage* calib_provider)
: io_service_(io_service),
  sensors_(sensors),
  port_(port),
  host_initialized_(false),
  connected_(false),
  calib_provider_(calib_provider){
}

ConfigServer::~ConfigServer() {
}

template<class StreambufType>
bool ConfigServer::readSocket(StreambufType buffer)
{
  if(!connected_)
    return false;

  //try to read
  boost::system::error_code error;
  boost::asio::read(*config_socket_, buffer, error);

  if( checkDisconnect(error) )
  {
    clientDisconnected();
    return false;
  }

  return true;
}

template<class StreambufType>
bool ConfigServer::writeSocket(StreambufType buffer)
{
  if(!connected_)
    return false;

  //try to read
  boost::system::error_code error;
  boost::asio::write(*config_socket_, buffer, error);

  if( checkDisconnect(error) )
  {
    clientDisconnected();
    return false;
  }

  return true;
}

//true: disconnect, false: client connected
bool ConfigServer::checkDisconnect(const boost::system::error_code &error)
{
  //check if client has disconnected
  if (error == boost::asio::error::eof || error == boost::asio::error::connection_reset)
    return true;
  else
    return false;
}

void ConfigServer::clientDisconnected()
{
  //clean up environment for new clients
  host_initialized_=false;
  connected_=false;
  config_socket_.reset(); //free socket

  //turn off all sensors
  BOOST_FOREACH(const Sensor::Map::value_type& sensor_pair, sensors_)
    sensor_pair.second->off();

  printf("Config server: client disconnected!\n");
}

void ConfigServer::acceptConnection()
{
  //create new socket
  config_socket_.reset(new boost::asio::ip::tcp::socket(io_service_));

  //accept connections
  tcp::endpoint endpoint = tcp::endpoint(tcp::v4(), port_);
  tcp::acceptor acceptor(io_service_, endpoint);
  acceptor.set_option(tcp::no_delay(true));
  printf("Config server: accepting connection on %u\n", port_);

  //set client read timeout (client disconnect timeout)
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 100000; //100ms
  setsockopt(config_socket_->native(), SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  // stop here until client connects.
  acceptor.accept(*config_socket_);
  connected_ = true;
  printf("Config server: new connection\n");
}


void ConfigServer::sendAck(void)
{
  IpComm::Header header;
  header.data_size = 0;
  header.timestamp = 0;
  header.data_id = IpComm::ACK;

  writeSocket(boost::asio::buffer(header.getSerialized()));
}

void ConfigServer::sendNack(void)
{
  IpComm::Header header;
  header.data_size = 0;
  header.timestamp = 0;
  header.data_id = IpComm::NACK;

  writeSocket(boost::asio::buffer(header.getSerialized()));
}

void ConfigServer::sendCameraCalibration(unsigned int cam_id, unsigned int slot_id) {

  //create header
  IpComm::Header header;
  header.data_id = IpComm::READ_CAMERA_CALIBRATION;
  header.data_size = sizeof(IpComm::CameraCalibrationPayload);
  header.timestamp = 0;

  //fill calibration data
  IpComm::CameraCalibration calib;
  calib_provider_->loadCameraCalibration(cam_id, slot_id, calib); //TODO(schneith): use the returned bool for error handling...

  //send buffer
  std::vector<boost::asio::const_buffer> buffers;
  buffers.push_back(boost::asio::buffer(header.getSerialized()));
  buffers.push_back(boost::asio::buffer(calib.getSerialized()));
  writeSocket(buffers);
}

void ConfigServer::receiveCameraCalibration(void)
{
  //read payload package (sensor_id and rate)
  IpComm::CalibrationIdPayload inPackagePayload_id;
  bool success_net = readSocket(boost::asio::buffer(inPackagePayload_id));
  IpComm::CalibrationId calib_id(inPackagePayload_id);

  IpComm::CameraCalibrationPayload inPackagePayload_calib;
  success_net &= readSocket(boost::asio::buffer(inPackagePayload_calib));
  IpComm::CameraCalibration calib(inPackagePayload_calib);

  if(!success_net)
  {
    sendNack();
    return;
  }

  //write calibration
  bool success = calib_provider_->saveCameraCalibration(calib_id.port_id, calib_id.slot_id, calib);

  printf("write camera calibration - port_id: %d - slot_id: %d - success: %d\n", calib_id.port_id, calib_id.slot_id, success);

  //send (N)ACK
  if(success)
    sendAck();
  else
    sendNack();
}

void ConfigServer::sendFpgaInfo() {
  boost::shared_ptr<TimingBlock> timing_block = boost::dynamic_pointer_cast<
      TimingBlock>(sensors_.at(visensor::SensorId::SENSOR_CLOCK));

  IpComm::Header header;
  header.data_id = IpComm::FPGA_INFO;
  header.data_size = sizeof(IpComm::FpgaInfoPayload);
  header.timestamp = 0;

  IpComm::FpgaInfo info;
  if(timing_block != 0)
  {
    info.timestamp = timing_block->getTime();
    printf("Sent FPGA time: %u\n", info.timestamp);
  }
  else
  {
    printf("Timing Block not found, send FPGA timestamp zero\n");
    info.timestamp = 0;
  }
  info.fpgaId = 21;
  info.firmwareVersionMajor = FIRMWARE_VERSION_MAJOR;
  info.firmwareVersionMinor = FIRMWARE_VERSION_MINOR;
  info.firmwareVersionPatch = FIRMWARE_VERSION_PATCH;

  info.numOfSensors = sensors_.size();

  std::vector<boost::asio::const_buffer> buffers;
  buffers.push_back(boost::asio::buffer(header.getSerialized()));
  buffers.push_back(boost::asio::buffer(info.getSerialized()));
  writeSocket(buffers);
}

void ConfigServer::sendSensorInfos() {

  std::vector<boost::asio::const_buffer> buffers;

  IpComm::Header header;
  header.data_id = IpComm::FPGA_INFO;

  header.data_size = sensors_.size();
  writeSocket(boost::asio::buffer(header.getSerialized()));

  printf("=====================================\n");
  printf("Send detected sensors to driver:\n");

  BOOST_FOREACH(const Sensor::Map::value_type& sensor_pair, sensors_)
  {
    const Sensor::Ptr sensor = sensor_pair.second;

    IpComm::SensorInfo info;
    info.sensor_id = sensor->id();
    info.sensor_type = sensor->getSensorType();

    printf("id: %02d type: %02d\n", info.sensor_id, info.sensor_type);

    writeSocket(boost::asio::buffer( info.getSerialized() ));
  }
  printf("=====================================\n");
}

void ConfigServer::readUbiRegister() {
  IpComm::BusPackagePayload inPackagePayload;
  if( !readSocket(boost::asio::buffer(inPackagePayload)) )
    return;

  IpComm::BusPackage inPackage(inPackagePayload);

  visensor::SensorId::SensorId sensor_id =
      static_cast<visensor::SensorId::SensorId>(inPackage.sensor_id);

  // read on bus
  short value;
  try{
    sensors_.at(sensor_id)->readUbiRegister(inPackage.registerAddress, value);
  } catch (std::exception& e) {
    std::cout << "readUbiRegister exception: " << e.what() << std::endl;
  }

  // send answer back
  IpComm::Header header;
  header.data_id = IpComm::READ_UBI_REGISTER;
  header.data_size = 1;
  header.timestamp = 0;

  IpComm::BusPackage package;
  package.NumBits = inPackage.NumBits;
  package.registerAddress = inPackage.registerAddress;
  package.sensor_id = inPackage.sensor_id;
  package.value = value;

  std::vector<boost::asio::const_buffer> buffers;
  buffers.push_back(boost::asio::buffer(header.getSerialized()));
  buffers.push_back(boost::asio::buffer(package.getSerialized()));
  writeSocket(buffers);
}

void ConfigServer::writeUbiRegister() {
  IpComm::BusPackagePayload inPackagePayload;
  if( !readSocket(boost::asio::buffer(inPackagePayload)) )
    return;

  IpComm::BusPackage inPackage(inPackagePayload);

  // write on bus
//  printf("ubi write: strn: %d, reg: %x val: %x\n", inPackage.streamNumber, inPackage.registerAddress,
//                                                      inPackage.value);
//  printf("BusPackagePayload: %x %x %x %x\n",ntohl(inPackagePayload[0]), ntohl(inPackagePayload[1]), ntohl(inPackagePayload[2]), ntohl(inPackagePayload[3]));

  visensor::SensorId::SensorId sensor_id =
      static_cast<visensor::SensorId::SensorId>(inPackage.sensor_id);

  try{
    sensors_.at(sensor_id)->writeUbiRegister(inPackage.registerAddress, inPackage.value);
  } catch (std::exception& e) {
    std::cout << "writeUbiRegister exception: " << e.what() << std::endl;
  }
}

void ConfigServer::readFpgaRegister() {
  IpComm::BusPackagePayload inPackagePayload;
  if( !readSocket(boost::asio::buffer(inPackagePayload)) )
    return;

  IpComm::BusPackage inPackage(inPackagePayload);

  visensor::SensorId::SensorId sensor_id =
      static_cast<visensor::SensorId::SensorId>(inPackage.sensor_id);

  // read on bus
  int32_t value;

  try{
    sensors_.at(sensor_id)->readFpgaRegister(inPackage.registerAddress, value);
  } catch (std::exception& e) {
    std::cout << "readUbiRegister exception: " << e.what() << std::endl;
  }

  // send answer back
  IpComm::Header header;
  header.data_id = IpComm::READ_FPGA_REGISTER;
  header.data_size = 1;
  header.timestamp = 0;

  IpComm::BusPackage package;
  package.NumBits = inPackage.NumBits;
  package.registerAddress = inPackage.registerAddress;
  package.sensor_id = inPackage.sensor_id;
  package.value = value;

  std::vector<boost::asio::const_buffer> buffers;
  buffers.push_back(boost::asio::buffer(header.getSerialized()));
  buffers.push_back(boost::asio::buffer(package.getSerialized()));
  writeSocket(buffers);
}

void ConfigServer::writeFpgaRegister() {
  IpComm::BusPackagePayload inPackagePayload;
  if( !readSocket(boost::asio::buffer(inPackagePayload)) )
    return;

  IpComm::BusPackage inPackage(inPackagePayload);
  // write on bus
  printf("fpga write: sensor_id: %d, reg: %x val: %d\n", inPackage.sensor_id, inPackage.registerAddress,
                                                      inPackage.value);

  visensor::SensorId::SensorId sensor_id =
      static_cast<visensor::SensorId::SensorId>(inPackage.sensor_id);

  try{
  sensors_.at(sensor_id)->writeFpgaRegister(inPackage.registerAddress,
                                                      inPackage.value);
  }
  catch (std::exception& e)
  {
    std::cout << "writeFpgaRegister exception: " << e.what() << std::endl;
  }
}

void ConfigServer::run()
{
  //process incoming data
  while(1) {
    try
    {
      //wait for new client
      if(!connected_)
      {
        usleep(50);
        acceptConnection();
      }

      //read headear
      IpComm::HeaderPayload header_payload;
      if( !readSocket(boost::asio::buffer(header_payload, sizeof(header_payload))) )
        continue;


      //process header
      IpComm::Header header(header_payload);
      printf("received header: data_id: %d data_size:%d\n",header.data_id, header.data_size );
      printf("header_payload: %x %x %x\n",ntohl(header_payload[0]), ntohl(header_payload[1]), ntohl(header_payload[2]));

      switch(header.data_id)
      {
        case (IpComm::REQUEST_FPGA_INFO):
        {
          sendFpgaInfo();
          break;
        }
        case (IpComm::REQUEST_SENSOR_INFO):
        {
          sendSensorInfos();
          break;
        }
        case (IpComm::READ_UBI_REGISTER):
        {
          readUbiRegister();
          break;
        }
        case (IpComm::WRITE_UBI_REGISTER):
        {
          writeUbiRegister();
          break;
        }
        case (IpComm::READ_FPGA_REGISTER):
        {
          readFpgaRegister();
          break;
        }
        case (IpComm::WRITE_FPGA_REGISTER):
        {
          writeFpgaRegister();
          break;
        }
        case (IpComm::READ_CAMERA_CALIBRATION):
        {
          //read payload package (sensor_id and rate)
          IpComm::CalibrationIdPayload inPackagePayload;
          if( !readSocket(boost::asio::buffer(inPackagePayload)) )
            continue;
          IpComm::CalibrationId inPackage(inPackagePayload);

          printf("reading camera calibration - port_id: %d - slot_id: %d\n", inPackage.port_id, inPackage.slot_id);

          //send calibration back to client
          sendCameraCalibration(inPackage.port_id, inPackage.slot_id);

          break;
        }
        case (IpComm::WRITE_CAMERA_CALIBRATION):
        {
          receiveCameraCalibration();
          break;
        }
        case (IpComm::START_SENSOR):
        {
          //read payload package (sensor_id and rate)
          IpComm::StartSensorPayload inPackagePayload;
          if( !readSocket(boost::asio::buffer(inPackagePayload)) )
            continue;

          IpComm::StartSensor inPackage(inPackagePayload);

          printf("Starting sensor %d with rate %d.\n", inPackage.id, inPackage.rate);
          //start sensor
          sensors_.at(static_cast<visensor::SensorId::SensorId>(inPackage.id))
              ->on(inPackage.rate);
          break;
        }
        case (IpComm::STOP_SENSOR): {
          sensors_.at(
              static_cast<visensor::SensorId::SensorId>(header.timestamp))->off();
          break;
        }
        case (IpComm::REQUEST_FILE):
        {
          //file_handler.sendFile();
          printf("FileTransfer not enabled at the moment!\n");
          break;
        }
        case (IpComm::SEND_FILE):
        {
          //file_handler.receiveFile();
          printf("FileTransfer not enabled at the moment!\n");
          break;
        }
        case (IpComm::HOST_INITIALIZED):
        {
          host_initialized_ = true;
          break;
        }
        default:
        {
          printf("ERROR: Received unknown header type: %d \n",header.data_id);
          break;
        }
      }
    } catch (std::exception& e) {
      std::cerr << "Config Server Error: " << e.what() << std::endl;
    }
  }
}
