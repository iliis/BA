/*
 * FileHandler.hpp
 *
 *  Created on: Nov 22, 2013
 *      Author: pascal
 */

#ifndef FILEHANDLER_HPP_
#define FILEHANDLER_HPP_

#include <boost/asio.hpp>
#include <boost/scoped_ptr.hpp>

class FileTransfer {
 public:
  FileTransfer(boost::scoped_ptr<boost::asio::ip::tcp::socket> socket);
  virtual ~FileTransfer();

  void sendFile();
  void receiveFile();

 private:
  std::string receivePathString();

 private:
  boost::scoped_ptr<boost::asio::ip::tcp::socket> socket_;
};

#endif /* FILEHANDLER_HPP_ */
