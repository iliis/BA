/*
 * CalibrationStorage.cpp
 *
 *  Created on: Aug 19, 2013
 *      Author: schneith
 */


#include <iostream>
#include <boost/lexical_cast.hpp>
#include "calibration/CalibrationStorage.hpp"

#define DEBUG

using boost::property_tree::ptree;
using boost::property_tree::read_xml;
using boost::property_tree::write_xml;

//system commands to enable/disable filesystem protection
const std::string CalibrationStorage::FS_WRITE_PROTECTION_OFF = "mount -o remount,rw /";
const std::string CalibrationStorage::FS_WRITE_PROTECTION_ON = "sync; mount -o remount,ro /";

const unsigned int CalibrationStorage::MAX_CAMERAS = 4;
const unsigned int CalibrationStorage::MAX_SLOTS = 10;

CalibrationStorage::CalibrationStorage(std::string datafile)
    : datafile_(datafile)
{
  loadPropertyTree();
}

CalibrationStorage::~CalibrationStorage()
{
}


bool CalibrationStorage::loadPropertyTree()
{
  try
  {
    read_xml(datafile_, root_, boost::property_tree::xml_parser::trim_whitespace );
  } catch(std::exception const&  ex)
  {
    #ifdef DEBUG
      std::cout << "Exception: " << ex.what() << "\n";
    #endif

    return false;
  }

  return true;
}

bool CalibrationStorage::savePropertyTree()
{
  try
  {
    //turn filesystem write protection on
    bool ret = system( FS_WRITE_PROTECTION_OFF.c_str() );

    //write to file
    boost::property_tree::xml_writer_settings<char> settings('\t', 1);
    write_xml(datafile_, root_, std::locale(), settings);

    //turn filesystem write protection on
    ret |= !system( FS_WRITE_PROTECTION_ON.c_str() );

  } catch(std::exception const&  ex)
  {
    #ifdef DEBUG
      std::cout << "Exception: " << ex.what() << "\n";
    #endif

    return false;
  }

  return true;
}

bool CalibrationStorage::saveCameraCalibration(const unsigned int cam_id, const unsigned int slot, const IpComm::CameraCalibration calib)
{
  //childtree
  if(cam_id > MAX_CAMERAS-1)
    return false;

  if(slot > MAX_SLOTS-1)
    return false;

  try
  {
    //build childtree name
    std::string cam_id_str = boost::lexical_cast<std::string>(cam_id);
    std::string slot_str = boost::lexical_cast<std::string>(slot);
    std::string child_tree = std::string("cam_") + cam_id_str + std::string("_") + slot_str + std::string(".");

    root_.put<double>(child_tree + "fu", calib.focal_point[0]);
    root_.put<double>(child_tree + "fv", calib.focal_point[1]);

    root_.put<double>(child_tree + "cu", calib.principal_point[0]);
    root_.put<double>(child_tree + "cv", calib.principal_point[1]);

    root_.put<double>(child_tree + "K0", calib.distortion[0]);
    root_.put<double>(child_tree + "K1", calib.distortion[1]);
    root_.put<double>(child_tree + "K2", calib.distortion[2]);
    root_.put<double>(child_tree + "K3", calib.distortion[3]);
    root_.put<double>(child_tree + "K4", calib.distortion[4]);

    root_.put<double>(child_tree + "R00", calib.R[0]);
    root_.put<double>(child_tree + "R01", calib.R[1]);
    root_.put<double>(child_tree + "R02", calib.R[2]);
    root_.put<double>(child_tree + "R10", calib.R[3]);
    root_.put<double>(child_tree + "R11", calib.R[4]);
    root_.put<double>(child_tree + "R12", calib.R[5]);
    root_.put<double>(child_tree + "R20", calib.R[6]);
    root_.put<double>(child_tree + "R21", calib.R[7]);
    root_.put<double>(child_tree + "R22", calib.R[8]);

    root_.put<double>(child_tree + "t0", calib.t[0]);
    root_.put<double>(child_tree + "t1", calib.t[1]);
    root_.put<double>(child_tree + "t2", calib.t[2]);

    //save to disk
    savePropertyTree();

  } catch(std::exception const&  ex)
  {
    #ifdef DEBUG
      std::cout << "Exception: " << ex.what() << "\n";
    #endif

    return false;
  }

  return true;
}

bool CalibrationStorage::loadCameraCalibration(const unsigned int cam_id, const unsigned int slot, IpComm::CameraCalibration &calib)
{
  //check camera id for range
  if(cam_id > MAX_CAMERAS-1)
    return false;

  if(slot > MAX_SLOTS-1)
    return false;

  try
  {
    //build childtree name
    std::string cam_id_str = boost::lexical_cast<std::string>(cam_id);
    std::string slot_str = boost::lexical_cast<std::string>(slot);
    std::string child_tree = std::string("cam_") + cam_id_str + std::string("_") + slot_str + std::string(".");

    //load data
    calib.focal_point[0] = root_.get<double>(child_tree + "fu");
    calib.focal_point[1] = root_.get<double>(child_tree + "fv");

    calib.principal_point[0] = root_.get<double>(child_tree + "cu");
    calib.principal_point[1] = root_.get<double>(child_tree + "cv");

    calib.distortion[0] = root_.get<double>(child_tree + "K0");
    calib.distortion[1] = root_.get<double>(child_tree + "K1");
    calib.distortion[2] = root_.get<double>(child_tree + "K2");
    calib.distortion[3] = root_.get<double>(child_tree + "K3");
    calib.distortion[4] = root_.get<double>(child_tree + "K4");

    calib.R[0] = root_.get<double>(child_tree + "R00");
    calib.R[1] = root_.get<double>(child_tree + "R01");
    calib.R[2] = root_.get<double>(child_tree + "R02");
    calib.R[3] = root_.get<double>(child_tree + "R10");
    calib.R[4] = root_.get<double>(child_tree + "R11");
    calib.R[5] = root_.get<double>(child_tree + "R12");
    calib.R[6] = root_.get<double>(child_tree + "R20");
    calib.R[7] = root_.get<double>(child_tree + "R21");
    calib.R[8] = root_.get<double>(child_tree + "R22");

    calib.t[0] = root_.get<double>(child_tree + "t0");
    calib.t[1] = root_.get<double>(child_tree + "t1");
    calib.t[2] = root_.get<double>(child_tree + "t2");



  } catch(std::exception const&  ex)
  {
    #ifdef DEBUG
      std::cout << "Exception: " << ex.what() << "\n";
    #endif

    return false;
  }

  //flag as valid
  calib.valid = 1;

  return true;
}


