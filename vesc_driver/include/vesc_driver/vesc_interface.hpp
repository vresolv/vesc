// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-

#ifndef VESC_DRIVER__VESC_INTERFACE_HPP_
#define VESC_DRIVER__VESC_INTERFACE_HPP_

#include "vesc_driver/vesc_packet.hpp"
#include "datatypes.hpp"

#include <exception>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <QTcpSocket>
#include <QSettings>
#include "vbytearray.h"

namespace vesc_driver
{

/**
 * Class providing an interface to the Vedder VESC motor controller via a serial port interface.
 */
class VescInterface
{
public:
  typedef std::function<void (const VescPacketConstPtr &)> PacketHandlerFunction;
  typedef std::function<void (const std::string &)> ErrorHandlerFunction;

  /**
   * Creates a VescInterface object. Opens the serial port interface to the VESC if @p port is not
   * empty, otherwise the serial port remains closed until connect() is called.
   *
   * @param port Address of the serial port, e.g. '/dev/ttyUSB0'.
   * @param packet_handler Function this class calls when a VESC packet is received.
   * @param error_handler Function this class calls when an error is detected, such as a bad
   *                      checksum.
   *
   * @throw SerialException
   */
  VescInterface(
    const std::string & port = std::string(),
    const PacketHandlerFunction & packet_handler = PacketHandlerFunction(),
    const ErrorHandlerFunction & error_handler = ErrorHandlerFunction());

  /**
   * Delete copy constructor and equals operator.
   */
  VescInterface(const VescInterface &) = delete;
  VescInterface & operator=(const VescInterface &) = delete;

  /**
   * VescInterface destructor.
   */
  ~VescInterface();

  /**
   * Sets / updates the function that this class calls when a VESC packet is received.
   */
  void setPacketHandler(const PacketHandlerFunction & handler);

  /**
   * Sets / updates the function that this class calls when an error is detected, such as a bad
   * checksum.
   */
  void setErrorHandler(const ErrorHandlerFunction & handler);

  /**
   * Opens the serial port interface to the VESC.
   *
   * @throw SerialException
   */
  void connect(const std::string & port);

  /**
   * Closes the serial port interface to the VESC.
   */
  void disconnect();

  /**
   * Gets the status of the serial interface to the VESC.
   *
   * @return Returns true if the serial port is open, false otherwise.
   */
  bool isConnected() const;

  /**
   * Send a VESC packet.
   */
  void send(const VescPacket & packet);

  void requestFWVersion();
  void requestState();
  void requestImuData();

  void setDutyCycle(double duty_cycle);
  void setCurrent(double current);
  void setBrake(double brake);
  void setSpeed(double speed);
  void setPosition(double position);
  void setServo(double servo);
  void setLastConnectionType(conn_t type);
  void packetReceived(QByteArray &packet);
  int try_decode_packet(unsigned char *buffer, unsigned int in_len, int *bytes_left, QVector<QByteArray> &decodedPackets);
  void processPacket(QByteArray data);

  Q_INVOKABLE bool updateTcpHubPassword(QString uuid, QString newPass);
  Q_INVOKABLE void storeSettings();
  Q_INVOKABLE void disconnectPort();
  Q_INVOKABLE void connectTcp(QString server, int port);
  Q_INVOKABLE void connectTcpHub(QString server, int port, QString id, QString pass);

private slots:
  //slot to receive tcp data
  void tcpInputDataAvailable();
  void timerSlot();

private:
  // Pimpl - hide serial port members from class users
  class Impl;
  std::unique_ptr<Impl> impl_;
  QTcpSocket *mTcpSocket;
  bool mTcpConnected;
  QTimer *mTimer;
  int mRxTimer;
  int mByteTimeout;
  unsigned int mRxReadPtr;
  unsigned int mRxWritePtr;
  int mBytesLeft;
  unsigned int mMaxPacketLen;
  unsigned int mBufferLen;
  unsigned char *mRxBuffer;

  QSettings mSettings;
  conn_t mLastConnType;
  QString mLastTcpServer;
  int mLastTcpPort;
  QString mLastTcpHubServer;
  int mLastTcpHubPort;
  QString mLastTcpHubVescID;
  QString mLastTcpHubVescPass;
  QVariantList mTcpHubDevs;

};

// todo: review
class SerialException : public std::exception
{
  // Disable copy constructors
  SerialException & operator=(const SerialException &);
  std::string e_what_;

public:
  explicit SerialException(const char * description)
  {
    std::stringstream ss;
    ss << "SerialException " << description << " failed.";
    e_what_ = ss.str();
  }
  SerialException(const SerialException & other)
  : e_what_(other.e_what_) {}
  virtual ~SerialException() throw() {}
  virtual const char * what() const throw()
  {
    return e_what_.c_str();
  }
};

}  // namespace vesc_driver

#endif  // VESC_DRIVER__VESC_INTERFACE_HPP_
