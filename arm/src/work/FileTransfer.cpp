/*
 * FileHandler.cpp
 *
 *  Created on: Nov 22, 2013
 *      Author: pascal
 */

#include <fstream>
#include <boost/array.hpp>

#include "FileTransfer.hpp"


namespace
{
const std::size_t CHUNK_SIZE = 8096;
}

FileTransfer::FileTransfer(boost::scoped_ptr<boost::asio::ip::tcp::socket> socket)
: socket_(socket){
}

FileTransfer::~FileTransfer() {
}

void FileTransfer::sendFile() {

  std::string path = receivePathString();

  std::ifstream stream(path.c_str(), std::ios::binary);
  if (!stream)
  {
    std::cout << "could not open file: " << path;
  }

  stream.seekg(0, std::ios::end);
  boost::uint64_t length = stream.tellg();
  stream.seekg(0);

  // file length
  boost::asio::write(*socket_, boost::asio::buffer(&length, sizeof(length)));

  // file content
  boost::array<char, CHUNK_SIZE> chunk;

  boost::uint64_t transferred = 0;

  while (transferred != length)
  {
    boost::uint64_t remaining = length - transferred;
    std::size_t chunk_size = (remaining > CHUNK_SIZE) ? CHUNK_SIZE : static_cast<std::size_t>(remaining);
    stream.read(&chunk[0], chunk_size);
    boost::asio::write(*socket_, boost::asio::buffer(chunk, chunk_size));
    transferred += chunk_size;
  }
}

void FileTransfer::receiveFile() {

  std::string path = receivePathString();

  std::ofstream stream(path.c_str(), std::ios::binary);
  if (!stream)
  {
    std::cout << "could not save file: " << path;
  }

  // file length
  boost::uint64_t length = 0;
  boost::asio::read(*socket_, boost::asio::buffer(&length, sizeof(length)));

  // file content
  boost::array<char, CHUNK_SIZE> chunk;

  boost::uint64_t transferred = 0;

  while (transferred != length)
  {
    boost::uint64_t remaining = length - transferred;
    std::size_t chunk_size = (remaining > CHUNK_SIZE) ? CHUNK_SIZE : static_cast<std::size_t>(remaining);
    boost::asio::read(*socket_, boost::asio::buffer(chunk, chunk_size));
    stream.write(&chunk[0], chunk_size);
    transferred += chunk_size;
  }
}

std::string FileTransfer::receivePathString() {
  // read string length
//  boost::uint64_t string_size = 0;
//  boost::asio::read(socket_, boost::asio::buffer(&string_size, sizeof(string_size)));
//  printf("got string size: %d\n", string_size);
//
//  // read string
//  std::string string(string_size, ' ');
//  boost::asio::read(socket_, boost::asio::buffer(&string, sizeof(string_size)));
//  printf("got string: %s\n", string.c_str());


  std::string string;

  boost::asio::streambuf buff;
  boost::asio::read_until(*socket_, buff, '\r');  // for example

  std::istream is(&buff);
  is >> string;

  printf("got string: %s\n", string.c_str());

  return string;
}
