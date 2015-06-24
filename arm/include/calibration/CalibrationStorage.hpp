/*
 * CalibrationStorage.hpp
 *
 *  Created on: Aug 19, 2013
 *      Author: schneith
 */

#ifndef CalibrationStorage_HPP_
#define CalibrationStorage_HPP_

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include "ip_data_definitions.hpp"


class CalibrationStorage {

 public:
  CalibrationStorage(std::string datafile);
  ~CalibrationStorage();

  bool saveCameraCalibration(const unsigned int cam_id, const unsigned int slot, const IpComm::CameraCalibration calib);
  bool loadCameraCalibration(const unsigned int cam_id, const unsigned int slot, IpComm::CameraCalibration &calib);


 private:
  bool initializePropertyTree();
  bool loadPropertyTree();
  bool savePropertyTree();

  //turn filesystem write protection on/off
  static const std::string FS_WRITE_PROTECTION_OFF;
  static const std::string FS_WRITE_PROTECTION_ON;
  static const unsigned int MAX_CAMERAS;
  static const unsigned int MAX_SLOTS;

  const std::string datafile_;
  boost::property_tree::ptree root_;


};


#endif /* CalibrationStorage_HPP_ */
